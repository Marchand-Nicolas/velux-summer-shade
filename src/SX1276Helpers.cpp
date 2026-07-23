/*
   Copyright (c) 2024. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#include <Arduino.h>

#include <SX1276Helpers.h>
#include <board-config.h>

#if defined(RADIO_SX127X)
#include <map>

#if defined(ESP8266)
    #include <TickerUs.h>
#elif defined(ESP32)
#include <TickerUsESP32.h>
#include <esp_task_wdt.h>
#include <SPI.h>
// #include <SPIeX.h>
#endif

namespace Radio {
    // The SX1276 supports a faster bus, but 1 MHz gives substantially more
    // margin on the integrated Heltec routing and during radio recovery.
    SPISettings SpiSettings(1000000, MSBFIRST, SPI_MODE0);

    namespace {
        bool softwareSpi = false;
        portMUX_TYPE spiMux = portMUX_INITIALIZER_UNLOCKED;
        constexpr uint8_t MODE_BITS_MASK = static_cast<uint8_t>(~RF_OPMODE_MASK);
        constexpr uint8_t PA_BOOST_10_DBM = RF_PACONFIG_PASELECT_PABOOST | 0x08;
        constexpr uint8_t OCP_100_MA = RF_OCP_ON | RF_OCP_TRIM_100_MA;
        constexpr uint8_t PA_DAC_NORMAL = 0x84;
        constexpr uint8_t PLL_HF_DEFAULT = RF_PLL_BANDWIDTH_300 | 0x10;

        uint8_t IRAM_ATTR transferByte(uint8_t outgoing) {
            if (!softwareSpi) {
                return SPI.transfer(outgoing);
            }

            uint8_t incoming = 0;
            for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
                digitalWrite(RADIO_SCLK, LOW);
                digitalWrite(RADIO_MOSI, (outgoing & mask) ? HIGH : LOW);
                delayMicroseconds(1);
                digitalWrite(RADIO_SCLK, HIGH);
                delayMicroseconds(1);
                incoming = static_cast<uint8_t>(
                    (incoming << 1) | (digitalRead(RADIO_MISO) ? 1 : 0));
            }
            digitalWrite(RADIO_SCLK, LOW);
            return incoming;
        }

        void enableSoftwareSpi() {
            SPI.end();
            delay(2);
            pinMode(RADIO_SCLK, OUTPUT);
            pinMode(RADIO_MOSI, OUTPUT);
            pinMode(RADIO_MISO, INPUT_PULLUP);
            pinMode(RADIO_NSS, OUTPUT);
            digitalWrite(RADIO_SCLK, LOW);
            digitalWrite(RADIO_MOSI, LOW);
            digitalWrite(RADIO_NSS, HIGH);
            softwareSpi = true;
            ets_printf("Radio: switched to software SPI fallback\n");
        }

        bool waitForMode(uint8_t expectedMode, uint8_t requiredFlags, uint32_t timeoutUs) {
            const uint64_t deadline = esp_timer_get_time() + timeoutUs;
            do {
                const uint8_t opMode = readByte(REG_OPMODE);
                const uint8_t irqFlags1 = readByte(REG_IRQFLAGS1);
                if ((opMode & MODE_BITS_MASK) == expectedMode &&
                    (irqFlags1 & requiredFlags) == requiredFlags) {
                    return true;
                }
                delayMicroseconds(10);
            } while (esp_timer_get_time() < deadline);
            return false;
        }

        bool waitForReceiveDomain(uint32_t timeoutUs) {
            const uint64_t deadline = esp_timer_get_time() + timeoutUs;
            do {
                const uint8_t opMode = readByte(REG_OPMODE) & MODE_BITS_MASK;
                const uint8_t irqFlags1 = readByte(REG_IRQFLAGS1);
                // With AGC/AFC triggered by PreambleDetect, the chip can
                // legitimately wait in FSRx until a signal arrives. Both
                // FSRx and Rx have the PA disabled; Tx/FSTx are rejected.
                if ((opMode == RF_OPMODE_SYNTHESIZER_RX ||
                     opMode == RF_OPMODE_RECEIVER) &&
                    (irqFlags1 & RF_IRQFLAGS1_PLLLOCK)) {
                    return true;
                }
                delayMicroseconds(10);
            } while (esp_timer_get_time() < deadline);
            return false;
        }

    }

    // Simplified bandwidth registries evaluation
    std::map<uint8_t, regBandWidth> __bw =
    {
        {25, {0x01, 0x04}}, // 25KHz
        {50, {0x01, 0x03}},
        {100, {0x01, 0x02}},
        {125, {0x00, 0x02}},
        {200, {0x01, 0x01}},
        {250, {0x00, 0x01}} // 250KHz
    };

/**
 * The function `SPI_beginTransaction` begins a SPI transaction and sets the RADIO_NSS pin to LOW.
 */
    void IRAM_ATTR SPI_beginTransaction() {
        portENTER_CRITICAL(&spiMux);
        if (!softwareSpi) {
            SPI.beginTransaction(Radio::SpiSettings);
        }
        digitalWrite(RADIO_NSS, LOW);
        if (softwareSpi) {
            delayMicroseconds(1);
        }
    }

/**
 * The function `SPI_endTransaction` ends the SPI transaction and sets the RADIO_NSS pin to HIGH.
 */
    void IRAM_ATTR SPI_endTransaction() {
        if (softwareSpi) {
            digitalWrite(RADIO_SCLK, LOW);
            delayMicroseconds(1);
        }
        digitalWrite(RADIO_NSS, HIGH);
        if (!softwareSpi) {
            SPI.endTransaction();
        }
        portEXIT_CRITICAL(&spiMux);
    }

/**
 * The function `initHardware` initializes the hardware for SPI communication with a radio chip, checks
 * the availability of the radio, configures SPI settings, and puts the radio chip in standby mode.
 */
    bool initHardware() {
        printf("\nSPI Init");
        softwareSpi = false;

        //gpio_pullup_en((gpio_num_t) RADIO_MISO);

        pinMode(RADIO_MISO, INPUT_PULLUP);

        // SPI pins configuration

        pinMode(RADIO_RESET, INPUT); // Connected to Reset; floating for POR

        // Check the availability of the Radio
        while (!digitalRead(RADIO_RESET)) {
#if defined(ESP32)
            esp_task_wdt_reset();
#endif
            delayMicroseconds(1);
        }
        delayMicroseconds(BOARD_READY_AFTER_POR);

        // Initialize SPI bus
#if defined(ESP32)
        // NSS is controlled manually. Passing it to SPI.begin() as a hardware
        // SS pin and then disabling hardware-CS proved unreliable after
        // repeated ESP32-only resets on this board.
        SPI.begin(RADIO_SCLK, RADIO_MISO, RADIO_MOSI, -1);
#endif
        // SPI.setFrequency(SPI_CLK_FRQ);
        // SPI.setDataMode(SPI_MODE0);
        // SPI.setBitOrder(MSBFIRST);
        // NSS is driven explicitly by SPI_begin/endTransaction. Enabling the
        // ESP32 hardware-CS at the same time can pulse NSS between bytes and
        // eventually leave the external SX1276 unreachable.
        SPI.setHwCs(false);

        // Disable SPI device
        // Disable device NRESET pin
        pinMode(RADIO_NSS, OUTPUT);
        pinMode(RADIO_RESET, OUTPUT);
        digitalWrite(RADIO_RESET, HIGH);
        digitalWrite(RADIO_NSS, HIGH);
        delayMicroseconds(BOARD_READY_AFTER_POR);

        // SPI.beginTransaction(Radio::SpiSettings);
        // SPI.endTransaction();

        // Reset the SX1276 explicitly. An ESP32 reset does not power-cycle the
        // radio, so it may otherwise retain a wedged TX/sequencer state.
        if (!hardReset()) {
            return false;
        }

        pinMode(SCAN_LED, OUTPUT);
        digitalWrite(SCAN_LED, 1);
        printf("\nRadio Chip is ready\n");
        return true;
    }

    bool hardReset() {
        const auto resetChip = []() {
            pinMode(RADIO_NSS, OUTPUT);
            digitalWrite(RADIO_NSS, HIGH);
            pinMode(RADIO_RESET, OUTPUT);
            digitalWrite(RADIO_RESET, LOW);
            delay(2);
            digitalWrite(RADIO_RESET, HIGH);
            delay(20);
            return readByte(REG_VERSION);
        };

        uint8_t version = resetChip();
        if (version != 0x12) {
            // The ESP32 hardware SPI controller can become unable to sample
            // MISO after a failed TX. Bit-banged mode uses the same four pins
            // but no SPI peripheral, so it remains able to reset and control
            // the radio instead of leaving it in a potentially jamming mode.
            enableSoftwareSpi();
            version = resetChip();
        }
        if (version != 0x12) {
            ets_printf("Radio: reset failed, unexpected version 0x%02X\n", version);
            return false;
        }

        // Reset leaves the FSK modem in Sleep. Image calibration only runs
        // once the oscillator is active in Standby.
        writeByte(REG_OPMODE, RF_OPMODE_STANDBY);
        if (!waitForMode(RF_OPMODE_STANDBY, RF_IRQFLAGS1_MODEREADY, 5000)) {
            ets_printf("Radio: failed to enter standby after reset\n");
            return false;
        }
        return true;
    }

bool setPreambleLength(uint16_t preambleLen) {
    const bool msbWritten =
        writeByte(REG_PREAMBLEMSB, (preambleLen >> 8) & 0xFF, true);
    const bool lsbWritten =
        writeByte(REG_PREAMBLELSB, preambleLen & 0xFF, true);
    ets_printf("Radio: Preamble length set to %u bytes\n", preambleLen);
    return msbWritten && lsbWritten;
}

/**
 * The `initRegisters` function initializes various registers of a radio module for both transmission
 * and reception in a C++ program.
 * 
 * @param maxPayloadLength The `maxPayloadLength` parameter in the `initRegisters` function is used to
 * set the maximum payload length for the radio communication. In this function, it is set to a default
 * value of `0xff` (255 in decimal). This parameter is used to configure the radio module to handle
 * packets
 */
    bool initRegisters(uint8_t maxPayloadLength) {
        bool configured = true;
        const auto setRegister = [&configured](uint8_t reg, uint8_t value) {
            if (!writeByte(reg, value, true)) {
                ets_printf("Radio: register 0x%02X rejected value 0x%02X\n", reg, value);
                configured = false;
            }
        };

        // Firstly put radio in StandBy mode as some parameters cannot be changed differently
        setRegister(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_STANDBY);

        // ---------------- Common Register init section ----------------
        // Switch-off clockout
        setRegister(REG_OSC, RF_OSC_CLKOUT_OFF); // This only give power saveing maybe we can use it as ticker µs

        // Variable packet lenght, generates working CRC.
        // Packet mode, IoHomeOn, IoHomePowerFrame to be added (0x10) to avoid rx to newly detect the preamble during tx radio shutdown
        // Must CRCAUTOCLEAR_ON or do full clean FIFO !
        setRegister(
            REG_PACKETCONFIG1,
            RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RF_PACKETCONFIG1_DCFREE_OFF | RF_PACKETCONFIG1_CRC_ON |
            RF_PACKETCONFIG1_CRCAUTOCLEAR_ON | RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT |
            RF_PACKETCONFIG1_ADDRSFILTERING_OFF);
        setRegister(
            REG_PACKETCONFIG2,
            RF_PACKETCONFIG2_DATAMODE_PACKET | RF_PACKETCONFIG2_IOHOME_ON | RF_PACKETCONFIG2_IOHOME_POWERFRAME);
        // Is IoHomePowerFrame useful ?

        // Preamble shall be set to AA for packets to be received by appliances. Sync word shall be set with different values if Rx or Tx
        setRegister(
            REG_SYNCCONFIG,
            RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_SYNC_ON);
        //0x51); // 0x91); // TODOVERIFY 0x92
        //RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON | RF_SYNCCONFIG_PREAMBLEPOLARITY_AA | RF_SYNCCONFIG_SYNC_ON);

        // Set Sync word to 0xff33 both for rx and tx
        setRegister(REG_SYNCVALUE1, SYNC_BYTE_1);
        setRegister(REG_SYNCVALUE2, SYNC_BYTE_2);

        // Mapping of pins DIO0 to DIO3
        // DIO0: PayloadReady|PacketSent    DIO1: FIFO empty    DIO2: Sync   | DIO3: TxReady
        // Mapping of pins DIO4 and DIO5
        // DIO4: PreambleDetect  DIO5: Data
        // DIO Mapping Data Packet Table 30 Page 69
        setRegister(
            REG_DIOMAPPING1,
            RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_01 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_01); // Org
        //        writeByte(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_01 | RF_DIOMAPPING1_DIO2_10 | RF_DIOMAPPING1_DIO3_01); // timeout on DIO2 for test
        setRegister(REG_DIOMAPPING2, RF_DIOMAPPING2_MAP_PREAMBLEDETECT | RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_DIO5_10);
        // Preamble on DIO4

        // Enable Fast Hoping (frequency change) // Not needed all the time
        // Not using that, as it miss a lot of frames
        if (MAX_FREQS != 1)
            setRegister(REG_PLLHOP, readByte(REG_PLLHOP) | RF_PLLHOP_FASTHOP_ON);

        // ---------------- TX Register init section ----------------
        // PA boost maximum power
        // writeByte(REG_PACONFIG, RF_PACONFIG_PASELECT_MASK | RF_PACONFIG_PASELECT_PABOOST);
        // writeByte(REG_OCP, RF_OCP_TRIM_240_MA); // 0x37); //200mA
        // writeByte(REG_PADAC, 0x87); // turn 20dBm mode on

        // PA Ramp: No Shaping, Ramp up/down 15us
        setRegister(REG_PARAMP, RF_PARAMP_MODULATIONSHAPING_00 | RF_PARAMP_0012_US); //_0015_US); //_0031_US); //
        // Setting Preamble Length
        setRegister(REG_PREAMBLEMSB, PREAMBLE_MSB);
        setRegister(REG_PREAMBLELSB, PREAMBLE_LSB);
        // FIFO Threshold - currently useless
        setRegister(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY);

        // ---------------- RX Register init section ----------------
        // Length filtering is intentionally disabled. Using maxPayloadLength
        // here prevents PayloadReady for valid variable-length io-homecontrol
        // frames on this modem.
        (void)maxPayloadLength;
        setRegister(REG_PAYLOADLENGTH, 0xff);
        // RSSI precision +-2dBm
        setRegister(REG_RSSICONFIG, RF_RSSICONFIG_SMOOTHING_8); // 8->0.512 ms // _128); // _32); //_256); //
        // Activates Timeout interrupt on Preamble
        setRegister(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT | RF_RXCONFIG_RESTARTRXONCOLLISION_ON);
        // 250KHz BW with AFC
        setRegister(REG_AFCBW, RF_AFCBW_MANTAFC_16 | RF_AFCBW_EXPAFC_1);

        setRegister(REG_AFCFEI, 0x01);
        // if AGC_AUTO_ON, RF_LNA_GAIN_XX do nothing
        setRegister(REG_LNA, RF_LNA_BOOST_ON | RF_LNA_GAIN_G1); // 0xC3) ;

        // Enables Preamble Detect, 2 bytes
        setRegister(
            REG_PREAMBLEDETECT,
            RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 | RF_PREAMBLEDETECT_DETECTORTOL_10);

        // +2 dBm reached PacketSent but not the installed Velux reliably.
        // +10 dBm restores practical indoor link margin while remaining well
        // below the former +20 dBm setting that destabilized the 3V3 rail.
        setRegister(REG_PACONFIG, PA_BOOST_10_DBM);
        setRegister(REG_OCP, OCP_100_MA);
        setRegister(REG_PADAC, PA_DAC_NORMAL);

        // RegPll is not restored reliably by every NRESET recovery observed
        // on the board. Program the documented HF default explicitly.
        setRegister(REG_PLL, PLL_HF_DEFAULT);
        return configured;
    }

/**
 * The `calibrate` function in C++ performs radio calibration by adjusting power levels and setting the
 * frequency band.
 */
    bool calibrate(uint32_t timeoutUs) {
        const auto waitForCalibration = [timeoutUs]() {
            const uint64_t deadline = esp_timer_get_time() + timeoutUs;
            while ((readByte(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) ==
                   RF_IMAGECAL_IMAGECAL_RUNNING) {
                if (esp_timer_get_time() >= deadline) {
                    ets_printf("Radio: image calibration timed out\n");
                    return false;
                }
                delayMicroseconds(50);
            }
            return true;
        };

        // Save context
        uint8_t regPaConfigInitVal = readByte(REG_PACONFIG);

        // Cut the PA just in case, RFO output, power = -1 dBm
        writeByte(REG_PACONFIG, RF_PACONFIG_PASELECT_RFO);
        // RC Calibration (only call after setting correct frequency band)
        writeByte(REG_OSC, RF_OSC_RCCALSTART);
        // Start image and RSSI calibration
        writeByte(REG_IMAGECAL,
                  (readByte(REG_IMAGECAL) & RF_IMAGECAL_AUTOIMAGECAL_MASK &
                   RF_IMAGECAL_IMAGECAL_MASK) |
                  RF_IMAGECAL_IMAGECAL_START);
        // Wait end of calibration
        if (!waitForCalibration()) {
            writeByte(REG_PACONFIG, regPaConfigInitVal);
            return false;
        }
        // Set a Frequency in HF band
        Radio::setCarrier(Radio::Carrier::Frequency, 868000000);
        // Start image and RSSI calibration
        writeByte(REG_IMAGECAL,
                  (readByte(REG_IMAGECAL) & RF_IMAGECAL_AUTOIMAGECAL_MASK &
                   RF_IMAGECAL_IMAGECAL_MASK) |
                  RF_IMAGECAL_IMAGECAL_START);
        // Wait end of calibration
        if (!waitForCalibration()) {
            writeByte(REG_PACONFIG, regPaConfigInitVal);
            return false;
        }

        // Restore context
        writeByte(REG_PACONFIG, regPaConfigInitVal);
        return true;
    }

    /*!
     * Performs the Rx chain calibration for LF and HF bands
     * \remark Must be called just after the reset so all registers are at their
     *         default values
     */
    // void RxChainCalibration( void ) {
    //     uint8_t regPaConfigInitVal;
    //     uint32_t initialFreq;

    //     // Save context
    //     regPaConfigInitVal = readByte( REG_PACONFIG );
    //     initialFreq = ( double )( ( ( uint32_t )readByte( REG_FRFMSB ) << 16 ) |
    //                               ( ( uint32_t )readByte( REG_FRFMID ) << 8 ) |
    //                               ( ( uint32_t )readByte( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    //     // Cut the PA just in case, RFO output, power = -1 dBm
    //     writeByte( REG_PACONFIG, 0x00 );

    //     // Launch Rx chain calibration for LF band
    //     writeByte ( REG_IMAGECAL, ( readByte( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    //     while( ( readByte( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    //     {
    //     }

    //     // Sets a Frequency in HF band
    //     SetChannel( 868000000 );

    //     // Launch Rx chain calibration for HF band
    //     writeByte ( REG_IMAGECAL, ( readByte( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    //     while( ( readByte( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    //     {
    //     }

    //     // Restore context
    //     writeByte( REG_PACONFIG, regPaConfigInitVal );
    //     SetChannel( initialFreq );
    // }
    bool IRAM_ATTR setStandby(uint32_t readyTimeoutUs) {
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_STANDBY);
        return waitForMode(RF_OPMODE_STANDBY, RF_IRQFLAGS1_MODEREADY, readyTimeoutUs);
    }

    bool IRAM_ATTR setTx(uint32_t readyTimeoutUs) {
        // Uncommon and incompatible settings
        // Enabling Sync word - Size must be set to SYNCSIZE_2 (0x01 in header file)
        writeByte(REG_SYNCCONFIG, (readByte(REG_SYNCCONFIG) & RF_SYNCCONFIG_SYNCSIZE_MASK) | RF_SYNCCONFIG_SYNCSIZE_2);

        writeByte(
            REG_OPMODE,
            (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_TRANSMITTER);

        return waitForMode(
            RF_OPMODE_TRANSMITTER,
            RF_IRQFLAGS1_TXREADY,
            readyTimeoutUs);
    }

    bool IRAM_ATTR setRx(uint32_t readyTimeoutUs) {
        // Uncommon and incompatible settings
        writeByte(REG_SYNCCONFIG, (readByte(REG_SYNCCONFIG) & RF_SYNCCONFIG_SYNCSIZE_MASK) | RF_SYNCCONFIG_SYNCSIZE_3);
        writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_RECEIVER);

        // PllLock alone is also high in TX. Combining it with an exact FSRx
        // or Rx RegOpMode value proves that the PA is disabled.
        return waitForReceiveDomain(readyTimeoutUs);
        /*
                // Start Sequencer
                writeByte(REG_OPMODE, (readByte(REG_OPMODE) & RF_OPMODE_MASK) | RF_OPMODE_RECEIVER);
                writeByte(REG_SEQCONFIG1, readByte(REG_SEQCONFIG1 | RF_SEQCONFIG1_SEQUENCER_START));
        */
    }


    void readBurst(uint8_t regAddr, uint8_t *buffer, uint8_t size) {
        for (uint8_t i = 0; i < size; ++i) {
            buffer[i] = readByte(regAddr + i);
        }
    } // Clears FIFO at startup to avoid dirty reads
    // void clearBuffer() {
    //     for (uint8_t idx=0; idx <= 64; ++idx)
    //         readByte(REG_FIFO);
    // }
    void clearBuffer() {
        // Taille du buffer FIFO du SX1276
        const uint8_t bufferSize = 64;

        // Lire le buffer par paquets de 32 octets
        for (uint8_t i = 0; i < bufferSize; i += 32) {
            uint8_t buffer[32]; // Tableau temporaire pour stocker les octets lus
            readBytes/*Burst*/(REG_FIFO, buffer, sizeof(buffer)); // Lire 32 octets à la fois
        }
    }

    //     void clearFlags() {
    //         uint8_t out[2] = {0xff, 0xff};
    //         writeBytes(REG_IRQFLAGS1, out, 2);
    //     }
    // void clearFlags_A() {
    //   uint8_t flags = readByte(REG_IRQFLAGS1);
    //   flags &= ~0xFF; // Efface tous les drapeaux
    //   writeByte(REG_IRQFLAGS1, flags);
    // }
    void IRAM_ATTR clearFlags() {
        // Clear the write-to-clear IRQ bits. Live packet-mode status bits are
        // cleared by the surrounding FIFO drain or RX/TX mode transition.
        uint8_t flags[2] = {0xFF, 0xFF};
        writeBytes(REG_IRQFLAGS1, flags, sizeof(flags));
    }

    bool IRAM_ATTR preambleDetected() {
        return readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_PREAMBLEDETECT;
    }

    bool IRAM_ATTR syncedAddress() {
        return readByte(REG_IRQFLAGS1) & RF_IRQFLAGS1_SYNCADDRESSMATCH;
    }

    bool IRAM_ATTR dataAvail() {
        return (readByte(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) == 0; //?false:true;
    }

    uint8_t IRAM_ATTR readByte(uint8_t regAddr) {
        uint8_t getByte;
        readBytes(regAddr, &getByte, 1);

        return (getByte);
    }

    void IRAM_ATTR readBytes(uint8_t regAddr, uint8_t *out, uint8_t len) {
        SPI_beginTransaction();
        transferByte(regAddr); // Send Address
        for (uint8_t idx = 0; idx < len; ++idx) {
            out[idx] = transferByte(0x00); // Clock out register data
        }
        SPI_endTransaction();
    }

    bool IRAM_ATTR writeByte(uint8_t regAddr, uint8_t data, bool check) {
        return writeBytes(regAddr, &data, 1, check);
    }

    auto IRAM_ATTR writeBytes(uint8_t regAddr, uint8_t *in, uint8_t len, bool check) -> bool {
        constexpr uint8_t maxAttempts = 3;
        for (uint8_t attempt = 0; attempt < maxAttempts; ++attempt) {
            SPI_beginTransaction();
            transferByte(regAddr | SPI_Write);
            for (uint8_t idx = 0; idx < len; ++idx) {
                transferByte(in[idx]);
            }
            SPI_endTransaction();

            if (!check) {
                return true;
            }

            bool matches = true;
            SPI_beginTransaction();
            transferByte(regAddr);
            for (uint8_t idx = 0; idx < len; ++idx) {
                if (transferByte(0x00) != in[idx]) {
                    matches = false;
                }
            }
            SPI_endTransaction();
            if (matches) {
                return true;
            }
            delayMicroseconds(20);
        }

        return false;
    }

    uint16_t IRAM_ATTR readWord(uint8_t regAddr) {
        uint8_t lowByte = readByte(regAddr);
        uint8_t highByte = readByte(regAddr + 1);
        return (highByte << 8) | lowByte;
    }

    void IRAM_ATTR writeWord(uint8_t regAddr, uint16_t value) {
        writeByte(regAddr, value >> 8);
        writeByte(regAddr + 1, value & 0xFF);
    }

    bool IRAM_ATTR inStdbyOrSleep() {
        uint8_t data = readByte(REG_OPMODE);
        data &= ~RF_OPMODE_MASK;
        if ((data == RF_OPMODE_SLEEP) || (data == RF_OPMODE_STANDBY))
            return true;

        return false;
    }

    bool IRAM_ATTR setCarrier(Carrier param, uint32_t value) {
        uint32_t tmpVal;
        uint8_t out[4];
        regBandWidth bw{};

        //  Change of Frequency can be done while the radio is working thanks to Freq Hopping
        if (!inStdbyOrSleep())
            if (param != Carrier::Frequency)
                return false;

        switch (param) {
            case Carrier::Frequency:
                /*uint32_t FRF = (newFreq * (uint32_t(1) << RADIOLIB_SX127X_DIV_EXPONENT)) / RADIOLIB_SX127X_CRYSTAL_FREQ;*/
                tmpVal = static_cast<uint32_t>((static_cast<float_t>(value) / FXOSC) * (1 << 19));
                out[0] = (tmpVal & 0x00ff0000) >> 16;
                out[1] = (tmpVal & 0x0000ff00) >> 8;
                out[2] = (tmpVal & 0x000000ff); // If Radio is active writing LSB triggers frequency change
                return writeBytes(REG_FRFMSB, out, 3, true);
            case Carrier::Bandwidth:
                bw = bwRegs(value);
                return writeByte(REG_RXBW, bw.Mant | bw.Exp, true) &&
                       writeByte(REG_AFCBW, bw.Mant | bw.Exp, true);
            case Carrier::Deviation:
                tmpVal = static_cast<uint32_t>((static_cast<float_t>(value) / FXOSC) * (1 << 19));
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                return writeBytes(REG_FDEVMSB, out, 2, true);
            //                writeByte(REG_BITRATEFRAC, 5); // Little more precision
            case Carrier::Modulation:
                switch (value) {
                    case Modulation::FSK: {
                        uint8_t rfOpMode = readByte(REG_OPMODE);
                        rfOpMode &= RF_OPMODE_LONGRANGEMODE_MASK;
                        rfOpMode |= RF_OPMODE_LONGRANGEMODE_OFF;
                        rfOpMode &= RF_OPMODE_MODULATIONTYPE_MASK;
                        rfOpMode |= RF_OPMODE_MODULATIONTYPE_FSK;
                        rfOpMode &= RF_OPMODE_MASK;
                        rfOpMode |= RF_OPMODE_STANDBY;
                        rfOpMode &= ~0x08;
                        return writeByte(REG_OPMODE, rfOpMode, true);
                    }
                    case Modulation::LoRa:
                    case Modulation::OOK:
                    default: return false;
                }
            case Carrier::Bitrate:
                tmpVal = FXOSC / value;
                out[0] = (tmpVal & 0x0000ff00) >> 8;
                out[1] = (tmpVal & 0x000000ff);
                return writeBytes(REG_BITRATEMSB, out, 2, true);
        }

        return false;
    }

    bool validateConfiguration(uint32_t expectedFrequency) {
        bool valid = true;
        const auto expect = [&valid](uint8_t reg, uint8_t expected, uint8_t mask = 0xFF) {
            const uint8_t actual = readByte(reg);
            if ((actual & mask) != (expected & mask)) {
                ets_printf("Radio: invalid reg 0x%02X, got 0x%02X expected 0x%02X mask 0x%02X\n",
                           reg, actual, expected, mask);
                valid = false;
            }
        };

        expect(REG_VERSION, 0x12);
        expect(REG_PACONFIG, PA_BOOST_10_DBM);
        expect(REG_OCP, OCP_100_MA);
        expect(REG_PADAC, PA_DAC_NORMAL);
        expect(REG_PLL, PLL_HF_DEFAULT);
        expect(REG_BITRATEMSB, 0x03);
        expect(REG_BITRATELSB, 0x41);
        expect(REG_FDEVMSB, 0x01);
        expect(REG_FDEVLSB, 0x3A);
        expect(REG_PACKETCONFIG2,
               RF_PACKETCONFIG2_DATAMODE_PACKET |
               RF_PACKETCONFIG2_IOHOME_ON |
               RF_PACKETCONFIG2_IOHOME_POWERFRAME);

        if (expectedFrequency != 0) {
            const uint32_t frf =
                static_cast<uint32_t>((static_cast<float_t>(expectedFrequency) / FXOSC) * (1 << 19));
            expect(REG_FRFMSB, (frf >> 16) & 0xFF);
            expect(REG_FRFMID, (frf >> 8) & 0xFF);
            expect(REG_FRFLSB, frf & 0xFF);
        }
        return valid;
    }

    regBandWidth bwRegs(uint8_t bandwidth) {
        for (auto &it: __bw)
            if (it.first == bandwidth)
                return it.second;

        return __bw.rbegin()->second;
    }

    void dump() {
        uint8_t idx = 0;

        Serial.printf("#Type\tRegister Name\tAddress[Hex]\tValue[Hex]\n");
        do {
            Serial.printf("REG\tname\t0x%2.2x\t0x%2.2x\n", idx, readByte(idx));
            idx += 1;
        } while (idx < 0x7f);
        Serial.printf("PKT\tFalse;False;255;0;\nXTAL\t32000000\n");
        // Serial.printf("\n");
        dumpReal();
    }

    void dumpReal() {
        uint8_t registers[0x80];
        registers[0] = 0x00;
        readBytes(0x01, registers + 1, 0x7F);
        // sx127x_dump_registers(registers, device);
        for (int idx = 0; idx < sizeof(registers); idx++) {
            if (idx != 0) {
                printf(",");
            }
            printf("0x%2.2x", registers[idx]);
        }
        printf("\n");

        dump_fsk_registers(registers);
    }
}
#endif
