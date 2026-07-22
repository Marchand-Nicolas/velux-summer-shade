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

#include <esp32-hal-gpio.h>
#include <map>
#include "esp_log.h"
#include <queue>

#include <iohcRadio.h>
#include <utility>
#include <log_buffer.h>

namespace {
    constexpr uint16_t LONG_PREAMBLE_BYTES = 1920;
    constexpr uint16_t SHORT_PREAMBLE_BYTES = 40;
    constexpr uint32_t IOHC_BITRATE_BPS = 38400;
    constexpr uint64_t TX_WATCHDOG_MARGIN_US = 250000;
    constexpr uint64_t TX_WAIT_LOG_INTERVAL_US = 250000;
    constexpr uint8_t MAX_TX_RECOVERY_ATTEMPTS = 1;
    constexpr TickType_t RX_PREAMBLE_TIMEOUT_TICKS = pdMS_TO_TICKS(800);
    constexpr TickType_t RADIO_HEALTH_INTERVAL_TICKS = pdMS_TO_TICKS(50);
    constexpr uint8_t MAX_CONSECUTIVE_SPI_FAILURES = 3;
    constexpr uint8_t MODE_BITS_MASK = static_cast<uint8_t>(~RF_OPMODE_MASK);

    uint64_t txWatchdogDurationUs(uint16_t preambleBytes, uint8_t payloadBytes) {
        // FSK RegPreambleSize is expressed in bytes. IO-homecontrol adds start
        // and stop bits, so each byte occupies ten bits on air. Include sync,
        // length and CRC bytes, then leave a margin for task scheduling.
        const uint64_t bytesOnAir = preambleBytes + payloadBytes + 5ULL;
        return ((bytesOnAir * 10ULL * 1000000ULL) / IOHC_BITRATE_BPS) +
               TX_WATCHDOG_MARGIN_US;
    }
}

namespace IOHC {
    iohcRadio *iohcRadio::_iohcRadio = nullptr;
    volatile unsigned long iohcRadio::_g_payload_millis = 0L;
    uint8_t iohcRadio::_flags[2] = {0, 0};
    volatile bool iohcRadio::send_lock = false;
    volatile iohcRadio::RadioState iohcRadio::radioState = iohcRadio::RadioState::IDLE;
    volatile bool iohcRadio::txComplete = false;


    TaskHandle_t handle_interrupt;
    TaskHandle_t callbackTask = NULL;
    QueueHandle_t callbackQueue = NULL;
    struct Callback {
        IohcPacketDelegate *callback;
        iohcPacket *packet;
    };


    /**
     * The function `handle_interrupt_task` waits for a notification and then calls the `tickerCounter`
     * function if certain conditions are met.
     *
     * @param pvParameters The `pvParameters` parameter in the `handle_interrupt_task` function is a void
     * pointer that can be used to pass any data or object to the task when it is created. In this specific
     * function, it is being cast to a pointer of type `iohcRadio` and then passed to the
     */
    void IRAM_ATTR handle_interrupt_task(void *pvParameters) {
        static uint32_t thread_notification;
        while (true) {
            const bool waitingForPayload =
                iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE;
            thread_notification = ulTaskNotifyTake(
                pdTRUE,
                waitingForPayload ? RX_PREAMBLE_TIMEOUT_TICKS : RADIO_HEALTH_INTERVAL_TICKS);
            if (thread_notification &&
                (iohcRadio::radioState == iohcRadio::RadioState::PAYLOAD ||
                 iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE)) {
                iohcRadio::tickerCounter((iohcRadio *) pvParameters);
            } else if (!thread_notification &&
                       iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE) {
                // A noise-triggered preamble may never be followed by DIO0.
                // Do not leave the software state machine wedged forever.
                Radio::setStandby();
                Radio::clearFlags();
                if (Radio::setRx()) {
                    iohcRadio::setRadioState(iohcRadio::RadioState::RX);
                } else {
                    iohcRadio::setRadioState(iohcRadio::RadioState::ERROR);
                    ets_printf("Radio: failed to recover after preamble timeout\n");
                }
            }
            ((iohcRadio *) pvParameters)->monitorRadioHealth();
        }

    }

    /**
     * The function `handle_interrupt_fromisr` reads digital inputs and notifies a thread to wake up when
     * the interrupt service routine is complete.
     */
    void IRAM_ATTR handle_interrupt_fromisr() {
        // DIO0 is PacketSent while transmitting and PayloadReady while
        // receiving. Never let an RX event handler change the state or touch
        // SPI while a TX is still in progress.
        if (iohcRadio::radioState == iohcRadio::RadioState::TX) {
            // This ISR is attached on the rising edge. DIO4 has no active TX
            // mapping in our configuration, so a TX-time edge is PacketSent.
            iohcRadio::txComplete = true;
            return;
        }

        if (iohcRadio::radioState == iohcRadio::RadioState::ERROR ||
            iohcRadio::radioState == iohcRadio::RadioState::IDLE) {
            return;
        }

        const bool payload = digitalRead(RADIO_PACKET_AVAIL);
        const bool preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
        if (payload) {
            iohcRadio::setRadioState(iohcRadio::RadioState::PAYLOAD);
        } else if (preamble) {
            iohcRadio::setRadioState(iohcRadio::RadioState::PREAMBLE);
        } else {
            return;
        }

        // Notify de RX state machine
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(handle_interrupt, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void callbackTaskLoop(void *parameters) {
        Callback *callback = NULL;
        while (true) {
            if (xQueueReceive(callbackQueue, &callback, portMAX_DELAY) == pdPASS && callback != NULL) {
                (*callback->callback)(callback->packet);
                delete callback->packet;
                vPortFree(callback);
            }
        }
    }

    bool iohcRadio::configureRadio() {
        if (!Radio::calibrate()) {
            return false;
        }

        Radio::initRegisters(MAX_FRAME_LEN);
        Radio::setCarrier(Radio::Carrier::Deviation, 19200);
        Radio::setCarrier(Radio::Carrier::Bitrate, 38400);
        Radio::setCarrier(Radio::Carrier::Bandwidth, 250);
        Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);
        return true;
    }

    iohcRadio::iohcRadio() {
        sendMutex = xSemaphoreCreateMutex();
        if (sendMutex == nullptr) {
            setRadioState(RadioState::ERROR);
            ets_printf("Radio: failed to create TX queue mutex\n");
            return;
        }

        if (!Radio::initHardware() || !configureRadio()) {
            setRadioState(RadioState::ERROR);
            ets_printf("Radio: initialization failed\n");
            return;
        }

        // Attach interrupts to Preamble detected and end of packet sent/received
        /* TODO this is wrongly named and/or assigned, but work like that*/
        //        printf("Starting TickTimer Handler...\n");
        //        TickTimer.attach_us(SM_GRANULARITY_US/*SM_GRANULARITY_MS*/, tickerCounter, this);
#if defined(RADIO_SX127X)
        //        attachInterrupt(RADIO_PACKET_AVAIL, i_payload, CHANGE); //
        //        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, CHANGE); //
        attachInterrupt(RADIO_DIO0_PIN, handle_interrupt_fromisr, RISING); //CHANGE); //
        //        attachInterrupt(RADIO_DIO1_PIN, handle_interrupt_fromisr, RISING); // CHANGE); //
        attachInterrupt(RADIO_DIO2_PIN, handle_interrupt_fromisr, RISING); //CHANGE); //
#elif defined(CC1101)
        attachInterrupt(RADIO_PREAMBLE_DETECTED, i_preamble, RISING);
#endif

        callbackQueue = xQueueCreate(20, sizeof(struct Callback *));
        auto callbackTaskCode = xTaskCreatePinnedToCore(callbackTaskLoop, "CallbackTask", 4096, NULL, 5, &callbackTask, 0);
        if (callbackTaskCode != pdPASS || callbackQueue == NULL) {
            printf("ERROR: Can't create callback-task or corresponding queue %d\n", callbackTaskCode);
            // sx127x_destroy(device);
            return;
        }

        // start state machine
        printf("Starting Interrupt Handler...\n");
        BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task, "handle_interrupt_task", 8192,
                                                       this /*nullptr*//*device*/, /*tskIDLE_PRIORITY*/4,
                                                       &handle_interrupt, /*tskNO_AFFINITY*/xPortGetCoreID());
        if (task_code != pdPASS) {
            printf("ERROR STATEMACHINE Can't create task %d\n", task_code);
            // sx127x_destroy(device);
            return;
        }
    }

    /**
     * @brief The function `iohcRadio::getInstance()` returns a pointer to a single instance of the `iohcRadio`
     * class, creating it if it doesn't already exist.
     *
     * @return An instance of the `iohcRadio` class is being returned.
     */
    iohcRadio *iohcRadio::getInstance() {
        if (!_iohcRadio)
            _iohcRadio = new iohcRadio();
        return _iohcRadio;
    }

/**
 * The `start` function initializes the radio with specified parameters and sets it to receive mode.
 * 
 * @param num_freqs The `num_freqs` parameter in the `start` function represents the number of
 * frequencies to scan. It is of type `uint8_t`, which means it is an unsigned 8-bit integer. This
 * parameter specifies how many frequencies the radio will scan during operation.
 * @param scan_freqs The `scan_freqs` parameter is an array of `uint32_t` values that represent the
 * frequencies to be scanned during the radio operation. The `start` function initializes the radio
 * with the provided frequencies for scanning.
 * @param scanTimeUs The `scanTimeUs` parameter in the `start` function of the `iohcRadio` class
 * represents the time interval in microseconds for scanning frequencies. If a specific value is
 * provided for `scanTimeUs`, it will be used as the scan interval. Otherwise, the default scan
 * interval defined as
 * @param rxCallback The `rxCallback` parameter is of type `IohcPacketDelegate`, which is a delegate or
 * function pointer that will be called when a packet is received by the radio. It is set to `nullptr`
 * by default if not provided during the function call.
 * @param txCallback The `txCallback` parameter in the `start` function of the `iohcRadio` class is of
 * type `IohcPacketDelegate`. It is a callback function that will be called when a packet is
 * transmitted by the radio. This callback function can be provided by the user of the `
 */
    void iohcRadio::start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs,
                          IohcPacketDelegate rxCallback = nullptr, IohcPacketDelegate txCallback = nullptr) {
        this->num_freqs = num_freqs;
        this->scan_freqs = scan_freqs;
        this->scanTimeUs = scanTimeUs ? scanTimeUs : DEFAULT_SCAN_INTERVAL_US;
        this->rxCB = std::move(rxCallback);
        this->txCB = std::move(txCallback);

        if (radioState == RadioState::ERROR) {
            ets_printf("Radio: startup skipped after initialization failure\n");
            return;
        }

        Radio::clearBuffer();
        Radio::clearFlags();
        /* We always start at freq[0] the 1W/2W channel*/
        Radio::setCarrier(Radio::Carrier::Frequency, scan_freqs[0]); //868950000);
        // Radio::calibrate();
        if (Radio::setRx()) {
            setRadioState(RadioState::RX);
        } else {
            setRadioState(RadioState::ERROR);
            ets_printf("Radio: failed to enter RX during startup\n");
        }
    }

/**
 * The `tickerCounter` function in C++ handles various radio operations based on different conditions
 * and configurations for SX127X and CC1101 radios.
 * 
 * @param radio The `radio` parameter in the `iohcRadio::tickerCounter` function is a pointer to an
 * instance of the `iohcRadio` class. This pointer is used to access and modify the properties and
 * methods of the `iohcRadio` object within the function. The function uses this pointer
 * 
 * @return In the provided code snippet, the function `tickerCounter` is returning different values
 * based on the conditions met within the function. Here is a breakdown of the possible return
 * scenarios:
 */
    void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio *radio) {
        // Not need to put in IRAM as we reuse task for µs instead ISR
#if defined(RADIO_SX127X)
        Radio::readBytes(REG_IRQFLAGS1, _flags, sizeof(_flags));

        // If Int of PayLoad
        if (radioState == iohcRadio::RadioState::PAYLOAD) {
            radio->receive(false);
            Radio::clearFlags();
            radio->setRadioState(iohcRadio::RadioState::RX);
            radio->tickCounter = 0;
            radio->preCounter = 0;
            return;
        }

        if (radioState == iohcRadio::RadioState::PREAMBLE) {
            radio->tickCounter = 0;
            radio->preCounter = radio->preCounter + 1;
            //radio->preCounter += 1;

            //            if (_flags[0] & RF_IRQFLAGS1_SYNCADDRESSMATCH) radio->preCounter = 0;
            // In case of Sync received resets the preamble duration
            if ((radio->preCounter * SM_GRANULARITY_US) >= SM_PREAMBLE_RECOVERY_TIMEOUT_US) {
                // Avoid hanging on a too long preamble detect
                Radio::clearFlags();
                radio->preCounter = 0;
            }
        }

        if (radioState != iohcRadio::RadioState::RX) return;

        //if (++radio->tickCounter * SM_GRANULARITY_US < radio->scanTimeUs) return;
        radio->tickCounter = radio->tickCounter + 1;
        if (radio->tickCounter * SM_GRANULARITY_US < radio->scanTimeUs) return;

        radio->tickCounter = 0;

        if (radio->num_freqs == 1) return;

        radio->currentFreqIdx += 1;
        if (radio->currentFreqIdx >= radio->num_freqs)
            radio->currentFreqIdx = 0;

        Radio::setCarrier(Radio::Carrier::Frequency, radio->scan_freqs[radio->currentFreqIdx]);

#elif defined(CC1101)
        if (__g_preamble){
            radio->receive();
            radio->tickCounter = 0;
            radio->preCounter = 0;
            return;
        }

        if (radioState != iohcRadio::RadioState::RX)
            return;

        if ((++radio->tickCounter * SM_GRANULARITY_US) < radio->scanTimeUs)
            return;
#endif
    }

    /**
     * The `send` function in the `iohcRadio` class sends packets stored in a vector with a specified
     * repeat time.
     *
     * @param iohcTx `iohcTx` is a reference to a vector of pointers to `iohcPacket` objects.
     *
     * @return If `txMode` is true, the `send` function will return early without executing the rest of the
     * code inside the function.
     */

    /**  
    void iohcRadio::send(std::vector<iohcPacket *> &iohcTx) {
        if (radioState == iohcRadio::RadioState::TX) return;

        packets2send = iohcTx; //std::move(iohcTx); //
        iohcTx.clear();

        txCounter = 0;
        setRadioState(iohcRadio::RadioState::TX);
        Sender.attach_ms(packets2send[txCounter]->repeatTime, packetSender, this);
    }
    */

void iohcRadio::queueSend(std::vector<iohcPacket *> &iohcTx) {
    if (iohcTx.empty()) {
        return;
    }
    if (xSemaphoreTake(sendMutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    sendQueue.push(std::move(iohcTx));
    ets_printf("TX: Queued send batch. Queue depth=%d\n", static_cast<int>(sendQueue.size()));
    xSemaphoreGive(sendMutex);
}

void iohcRadio::startQueuedSend() {
    if (radioState == RadioState::TX || radioState == RadioState::ERROR ||
        !packets2send.empty()) {
        return;
    }

    if (xSemaphoreTake(sendMutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (radioState == RadioState::TX || radioState == RadioState::ERROR ||
        !packets2send.empty() || sendQueue.empty()) {
        xSemaphoreGive(sendMutex);
        return;
    }

    packets2send = std::move(sendQueue.front());
    sendQueue.pop();
    xSemaphoreGive(sendMutex);
    txCounter = 0;
    txComplete = false;
    txRecoveryAttempts = 0;
    ets_printf("TX: Preparing %d packet(s)\n", packets2send.size());
    if (!beginCurrentTransmission(LONG_PREAMBLE_BYTES, true)) {
        handleTxFailure("radio did not enter TX");
    }
}

void iohcRadio::send(iohcPacket *packet) {
    std::vector<iohcPacket *> packets = { packet };
    send(packets);
}

void iohcRadio::send(std::vector<iohcPacket *> &iohcTx) {
    if (radioState == RadioState::ERROR) {
        ets_printf("TX: Dropping %d packet(s): radio unavailable\n", iohcTx.size());
        for (auto *packet : iohcTx) {
            delete packet;
        }
        iohcTx.clear();
        return;
    }
    queueSend(iohcTx);
    startQueuedSend();
}

bool iohcRadio::beginCurrentTransmission(uint16_t preambleBytes, bool armTicker) {
    if (packets2send.empty() || txCounter >= packets2send.size()) {
        return false;
    }

    auto *packet = packets2send[txCounter];
    const uint32_t frequency = packet->frequency ? packet->frequency : scan_freqs[currentFreqIdx];

    txComplete = false;
    setRadioState(RadioState::TX);
    if (!Radio::setStandby()) {
        return false;
    }
    Radio::clearFlags();
    Radio::setCarrier(Radio::Carrier::Frequency, frequency);
    Radio::setPreambleLength(preambleBytes);
    Radio::writeBytes(REG_FIFO, packet->payload.buffer, packet->buffer_length);

    txStartedAtUs = esp_timer_get_time();
    txDeadlineAtUs = txStartedAtUs + txWatchdogDurationUs(preambleBytes, packet->buffer_length);
    txLastWaitLogAtUs = txStartedAtUs;
    if (!Radio::setTx()) {
        return false;
    }

    ets_printf("TX: Started packet %d/%d, preamble=%u bytes, timeout=%llu ms\n",
               txCounter + 1,
               packets2send.size(),
               preambleBytes,
               (txDeadlineAtUs - txStartedAtUs + 999ULL) / 1000ULL);

    if (armTicker) {
        const uint32_t intervalMs = packet->repeatTime ? packet->repeatTime : 1;
        Sender.attach_ms(intervalMs, &iohcRadio::onTxTicker, (void*)this);
    }
    return true;
}

bool iohcRadio::resetRadio() {
    if (!Radio::hardReset()) {
        return false;
    }

    if (!configureRadio()) {
        return false;
    }
    Radio::clearBuffer();
    Radio::clearFlags();
    Radio::setCarrier(Radio::Carrier::Frequency, scan_freqs[currentFreqIdx]);
    return Radio::setRx();
}

void iohcRadio::abortCurrentBatch() {
    for (size_t i = txCounter; i < packets2send.size(); ++i) {
        delete packets2send[i];
    }
    packets2send.clear();
    txCounter = 0;
    txComplete = false;
}

void iohcRadio::handleTxFailure(const char *reason) {
    if (!beginRecovery()) {
        return;
    }
    const uint8_t irqFlags1 = Radio::readByte(REG_IRQFLAGS1);
    const uint8_t irqFlags2 = Radio::readByte(REG_IRQFLAGS2);
    const uint8_t opMode = Radio::readByte(REG_OPMODE);
    ets_printf("TX: FAILURE: %s (state=%s opmode=0x%02X irq1=0x%02X irq2=0x%02X)\n",
               reason,
               radioStateToString(radioState),
               opMode,
               irqFlags1,
               irqFlags2);
    addLogMessage(String("TX failure: ") + reason);

    Sender.detach();
    txComplete = false;
    setRadioState(RadioState::ERROR);

    bool recovered = resetRadio();
    if (recovered && txRecoveryAttempts < MAX_TX_RECOVERY_ATTEMPTS &&
        !packets2send.empty() && txCounter < packets2send.size()) {
        ++txRecoveryAttempts;
        ets_printf("TX: Radio recovered, retrying current packet (%u/%u)\n",
                   txRecoveryAttempts,
                   MAX_TX_RECOVERY_ATTEMPTS);
        if (beginCurrentTransmission(LONG_PREAMBLE_BYTES, true)) {
            endRecovery();
            return;
        }
        ets_printf("TX: Retry could not enter TX\n");
        // The failed retry may have changed RegOpMode even though readiness
        // was never reached. Reset once more before exposing a non-TX state.
        recovered = resetRadio();
    }

    ets_printf("TX: Aborting current batch; radio recovered=%s\n", recovered ? "true" : "false");
    abortCurrentBatch();
    setRadioState(recovered ? RadioState::RX : RadioState::ERROR);
    endRecovery();
    if (recovered) {
        startQueuedSend();
    }
}

bool iohcRadio::beginRecovery() {
    taskENTER_CRITICAL(&recoveryMux);
    if (recoveryInProgress) {
        taskEXIT_CRITICAL(&recoveryMux);
        return false;
    }
    recoveryInProgress = true;
    taskEXIT_CRITICAL(&recoveryMux);
    return true;
}

void iohcRadio::endRecovery() {
    taskENTER_CRITICAL(&recoveryMux);
    recoveryInProgress = false;
    taskEXIT_CRITICAL(&recoveryMux);
}

void iohcRadio::monitorRadioHealth() {
    if (radioState == RadioState::IDLE || radioState == RadioState::ERROR ||
        recoveryInProgress) {
        return;
    }

    const uint8_t version = Radio::readByte(REG_VERSION);
    const uint8_t opMode = Radio::readByte(REG_OPMODE) & MODE_BITS_MASK;
    if (version != 0x12) {
        if (++consecutiveSpiFailures < MAX_CONSECUTIVE_SPI_FAILURES) {
            return;
        }
        consecutiveSpiFailures = 0;
        if (radioState == RadioState::TX) {
            handleTxFailure("SPI link lost during TX");
            return;
        }

        if (!beginRecovery()) {
            return;
        }
        ets_printf("Radio: SPI health check failed (version=0x%02X), resetting\n", version);
        setRadioState(RadioState::ERROR);
        const bool recovered = resetRadio();
        setRadioState(recovered ? RadioState::RX : RadioState::ERROR);
        endRecovery();
        if (recovered) {
            startQueuedSend();
        }
        return;
    }
    consecutiveSpiFailures = 0;

    if (radioState == RadioState::TX) {
        // Reaching the receive domain while software still reports TX means
        // the hardware has already left TX and the PA is off.
        if (opMode == RF_OPMODE_SYNTHESIZER_RX || opMode == RF_OPMODE_RECEIVER) {
            txComplete = true;
        } else if (esp_timer_get_time() >= txDeadlineAtUs) {
            handleTxFailure("independent TX watchdog timeout");
        }
        return;
    }

    // No software state other than TX is ever allowed to leave the PA or its
    // TX synthesizer enabled. Recover immediately if that invariant breaks.
    if (opMode == RF_OPMODE_TRANSMITTER || opMode == RF_OPMODE_SYNTHESIZER_TX) {
        if (!beginRecovery()) {
            return;
        }
        ets_printf("Radio: unsafe TX mode 0x%02X while state=%s; resetting\n",
                   opMode,
                   radioStateToString(radioState));
        Sender.detach();
        setRadioState(RadioState::ERROR);
        const bool recovered = resetRadio();
        setRadioState(recovered ? RadioState::RX : RadioState::ERROR);
        endRecovery();
        if (recovered) {
            startQueuedSend();
        }
    }
}

void iohcRadio::onTxTicker(void *arg) {
    iohcRadio *radio = (iohcRadio *)arg;
    if (radio->packets2send.empty() || radio->txCounter >= radio->packets2send.size()) {
        radio->handleTxFailure("invalid TX queue state");
        return;
    }
    auto packet = radio->packets2send[radio->txCounter];

    // Poll PacketSent as a fallback if the DIO0 edge was missed.
    const uint8_t irqFlags2 = Radio::readByte(REG_IRQFLAGS2);
    const uint8_t opMode = Radio::readByte(REG_OPMODE) & MODE_BITS_MASK;
    if ((irqFlags2 & RF_IRQFLAGS2_PACKETSENT) ||
        opMode == RF_OPMODE_SYNTHESIZER_RX || opMode == RF_OPMODE_RECEIVER) {
        txComplete = true;
    }

    if (!radio->txComplete) {
        const uint64_t now = esp_timer_get_time();
        if (now >= radio->txDeadlineAtUs) {
            radio->handleTxFailure("PacketSent timeout");
            return;
        }
        if (now - radio->txLastWaitLogAtUs >= TX_WAIT_LOG_INTERVAL_US) {
            radio->txLastWaitLogAtUs = now;
            ets_printf("TX: Waiting for PacketSent (%llu ms remaining)\n",
                       (radio->txDeadlineAtUs - now + 999ULL) / 1000ULL);
        }
        return;
    }

    ESP_LOGD("RADIO", "PacketSent received after %llu ms\n",
             (esp_timer_get_time() - radio->txStartedAtUs) / 1000ULL);

    if (packet->repeat > 0) {
        packet->repeat--;
        ets_printf("TX: Repeating current packet (%d repeats left)\n", packet->repeat);
    } else {
        // inform callback we finished sending this packet, this transfers ownership of the packet to the callback queue
        radio->sent(packet);

        radio->txCounter++;
        radio->txRecoveryAttempts = 0;


        if (radio->txCounter == radio->packets2send.size()) {
            ets_printf("TX: All packets sent. Stopping Ticker.\n");
            radio->Sender.detach();
            radio->packets2send.clear();
            if (Radio::setRx() || radio->resetRadio()) {
                radio->setRadioState(RadioState::RX);
            } else {
                radio->setRadioState(RadioState::ERROR);
                addLogMessage("Radio failed to return to RX after TX");
            }
            radio->startQueuedSend();
            return;
        }

        packet = radio->packets2send[radio->txCounter];
        ets_printf("TX: Moving to next packet %d/%d (repeat=%d)\n",
                    radio->txCounter + 1,
                    radio->packets2send.size(),
                    packet->repeat);
    }

    if (!radio->beginCurrentTransmission(SHORT_PREAMBLE_BYTES, false)) {
        radio->handleTxFailure("radio did not enter TX for repeat");
    }
}

bool queueCallback(IohcPacketDelegate* callback, iohcPacket* packet) {
    Callback *callbackData = (Callback*) pvPortMalloc(sizeof(Callback));
    if (callbackData == NULL) {
        return false;
    }

    callbackData->callback = callback;
    callbackData->packet = packet;

    if (xQueueSendToBack(callbackQueue, &callbackData, 0) != pdPASS) {
        vPortFree(callbackData);
        return false;
    }
    return true;
}

/**
 * The `sent` function in the `iohcRadio` class checks if a callback function `txCB` is set and calls
 * it with a packet as a parameter, returning the result.
 * 
 * @param packet The `packet` parameter is a pointer to an object of type `iohcPacket`.
 * 
 * @return The `sent` function is returning a boolean value, which is determined by the result of
 * calling the `txCB` function with the `packet` parameter. If `txCB` is not null, the return value
 * will be the result of calling `txCB(packet)`, otherwise it will be `false`.
 */
    bool IRAM_ATTR iohcRadio::sent(iohcPacket *packet) {
        bool ret = false;
        if (packet) {
            packetStamp = esp_timer_get_time();
            packet->decode(true);
            addLogMessage(String(packet->decodeToString(true).c_str()));
        }
        if (txCB && !queueCallback(&txCB, packet)) {
            delete packet;
        }
        return ret;
    }

    //    static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };
/**
 * The `iohcRadio::receive` function in C++ toggles an LED, reads radio data, processes it, and
 * triggers a callback function.
 * 
 * @param stats The `stats` parameter in the `iohcRadio::receive` function is a boolean parameter that
 * is used to determine whether to gather additional statistics during the radio reception process. If
 * `stats` is set to `true`, the function will collect and process additional information such as RSSI
 * (Received Signal
 * 
 * @return The function `iohcRadio::receive` is returning a boolean value `true`.
 */
    bool IRAM_ATTR iohcRadio::receive(bool stats = false) {
        digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
        // bool frmErr = false;
        auto iohc = new iohcPacket;
        iohc->buffer_length = 0;
        iohc->frequency = scan_freqs[currentFreqIdx];

        _g_payload_millis = esp_timer_get_time();
        packetStamp = _g_payload_millis;
#if defined(RADIO_SX127X)
        if (stats) {
            iohc->rssi = static_cast<float>(Radio::readByte(REG_RSSIVALUE)) / -2.0f;
            int16_t thres = Radio::readByte(REG_RSSITHRESH);
            iohc->snr = iohc->rssi > thres ? 0 : (thres - iohc->rssi);
            //            iohc->lna = RF96lnaMap[ (Radio::readByte(REG_LNA) >> 5) & 0x7 ];
            int16_t f = (uint16_t) Radio::readByte(REG_AFCMSB);
            f = (f << 8) | (uint16_t) Radio::readByte(REG_AFCLSB);
            //            iohc->afc = f * (32000000.0 / 524288.0); // static_cast<float>(1 << 19));
            iohc->afc = /*(int32_t)*/f * 61.0;
            //            iohc->rssiAt = micros();
        }
#elif defined(CC1101)
        __g_preamble = false;

        uint8_t tmprssi=Radio::SPIgetRegValue(REG_RSSI);
        if (tmprssi>=128)
            iohc->rssi = (float)((tmprssi-256)/2)-74;
        else
            iohc->rssi = (float)(tmprssi/2)-74;

        uint8_t bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
        size_t readBytes = 0;
        uint32_t lastPop = millis();
#endif

#if defined(RADIO_SX127X)

        while (Radio::dataAvail()) {
            iohc->payload.buffer[iohc->buffer_length++] = Radio::readByte(REG_FIFO);
        }

#elif defined(CC1101)
        uint8_t lenghtFrameCoded = 0xFF;
        uint8_t tmpBuffer[64]={0x00};
        while (readBytes < lenghtFrameCoded) {
            if ( (readBytes>=1) && (lenghtFrameCoded==0xFF) ){ // Obtain frame lenght
                lenghtFrame = (Radio::reverseByte( ((uint8_t)(tmpBuffer[0]<<4) | (uint8_t)(tmpBuffer[1]>>4)))) & 0b00011111;
                lenghtFrameCoded = ((lenghtFrame + 2 + 1)*8) + ((lenghtFrame + 2 + 1)*2);   // Calculate Num of bits of encoded frame (add 2 bit per byte)
                lenghtFrameCoded = ceil((float)lenghtFrameCoded/8);                         // divide by 8 bits per byte and round to up
                Radio::setPktLenght(lenghtFrameCoded);
                //Serial.printf("BytesReaded: %d\tlenghtFrame: 0x%d\t lenghtFrameCoded: 0x%d\n", readBytes, lenghtFrame,  lenghtFrameCoded);
            }

            if (bytesInFIFO == 0) {
                if (millis() - lastPop > 5) {
                    // readData was required to read a packet longer than the one received.
                    //Serial.println("No data for more than 5mS. Stop here.");
                    break;
                } else {
                    delay(1);
                    bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
                    continue;
                }
            }

            // read the minimum between "remaining length" and bytesInFifo
            uint8_t bytesToRead = (((uint8_t)(lenghtFrameCoded - readBytes))<(bytesInFIFO)?((uint8_t)(lenghtFrameCoded - readBytes)):(bytesInFIFO));
            Radio::SPIreadRegisterBurst(REG_FIFO, bytesToRead, &(tmpBuffer[readBytes]));
            readBytes += bytesToRead;
            lastPop = millis();

            // Get how many bytes are left in FIFO.
            bytesInFIFO = Radio::SPIgetRegValue(REG_RXBYTES, 6, 0);
        }


        frmErr=true;
        if (lenghtFrameCoded<255){
            int8_t lenFuncDecodeFrame = Radio::decodeFrame(tmpBuffer, lenghtFrameCoded);
            if (lenFuncDecodeFrame>0 && lenFuncDecodeFrame<=MAX_FRAME_LEN){
                if (iohcUtils::radioPacketComputeCrc(tmpBuffer, lenFuncDecodeFrame) == 0 ){
                    iohc->buffer_length = lenFuncDecodeFrame;
                    memcpy(iohc->payload.buffer, tmpBuffer, lenFuncDecodeFrame);  // volcamos el resultado al array de origen
                    frmErr=false;
                }
            }
        }

        // Flush then standby according to RXOFF_MODE (default: RADIOLIB_CC1101_RXOFF_IDLE)
        if (Radio::SPIgetRegValue(REG_MCSM1, 3, 2) == RF_RXOFF_IDLE) {
            Radio::SPIsendCommand(CMD_IDLE);                    // set mode to standby
            Radio::SPIsendCommand(CMD_FLUSH_RX | CMD_READ);     // flush Rx FIFO
        }

        Radio::SPIsendCommand(CMD_RX);
        setRadioState(iohcRadio::RadioState::RX);

#endif

        // Radio::clearFlags();
        iohc->decode(true); //stats);
        addLogMessage(String(iohc->decodeToString(true).c_str()));

        if (rxCB && !queueCallback(&rxCB, iohc)) {
            delete iohc;
        }
        digitalWrite(RX_LED, false);
        return true;
    }

/**
 * The `i_preamble` interrupt handler updates the radio state when a preamble is
 * detected on the current channel.
 */
    void IRAM_ATTR iohcRadio::i_preamble() {
#if defined(RADIO_SX127X)
        bool preamble = digitalRead(RADIO_PREAMBLE_DETECTED);
#elif defined(CC1101)
        __g_preamble = true;
        bool preamble = __g_preamble;
#endif
        iohcRadio::setRadioState(preamble ? iohcRadio::RadioState::PREAMBLE : iohcRadio::RadioState::RX);
    }

/**
 * The `i_payload` interrupt handler reads the payload detection pin and sets
 * the radio state accordingly.
 */
    void IRAM_ATTR iohcRadio::i_payload() {
#if defined(RADIO_SX127X)
        bool payload = digitalRead(RADIO_PACKET_AVAIL);
        iohcRadio::setRadioState(payload ? iohcRadio::RadioState::PAYLOAD : iohcRadio::RadioState::RX);
#endif
    }

    const char* iohcRadio::radioStateToString(RadioState state) {
    switch (state) {
        case RadioState::IDLE:     return "IDLE";
        case RadioState::RX:       return "RX";
        case RadioState::TX:       return "TX";
        case RadioState::PREAMBLE: return "PREAMBLE";
        case RadioState::PAYLOAD:  return "PAYLOAD";
        case RadioState::LOCKED:   return "LOCKED";
        case RadioState::ERROR:    return "ERROR";
        default:                   return "UNKNOWN";
    }
    }


    void IRAM_ATTR iohcRadio::setRadioState(RadioState newState) {
        radioState = newState;
        // Optional debug:
        //printf("State changed to: %d\n", static_cast<int>(newState));
        ets_printf("State: %s\n", radioStateToString(newState));
    }
}
