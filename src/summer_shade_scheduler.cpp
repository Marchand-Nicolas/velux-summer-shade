#include <summer_shade_scheduler.h>

#if defined(SUMMER_SHADE_AUTOMATION)

#include <Arduino.h>
#include <WiFi.h>
#include <iohcCryptoHelpers.h>
#include <iohcRemote1W.h>
#include <log_buffer.h>
#include <nvs_helpers.h>
#include <sys/time.h>
#include <time.h>

namespace {
constexpr const char *kTimeZoneParis = "CET-1CEST,M3.5.0/2,M10.5.0/3";
constexpr uint32_t kCheckIntervalMs = 60UL * 1000UL;
constexpr uint32_t kNtpRetryIntervalMs = 10UL * 60UL * 1000UL;
constexpr uint32_t kTimePersistIntervalMs = 30UL * 60UL * 1000UL;
constexpr uint32_t kStartupGraceMs = 10UL * 1000UL;
constexpr time_t kValidEpochThreshold = 1700000000; // 2023-11-14

TaskHandle_t s_schedulerTask = nullptr;
uint32_t s_lastNtpConfigMs = 0;
uint32_t s_lastTimePersistMs = 0;
uint64_t s_lastPersistedEpoch = 0;
bool s_ntpConfigured = false;
bool s_timeFallbackAttempted = false;
int s_lastAppliedYday = -1;
int s_lastAppliedOpenPercent = -1;

struct ShadeTarget {
    uint8_t hour;
    uint8_t openPercent;
};

constexpr ShadeTarget kSchedule[] = {
    {10, 67}, // one third closed
    {11, 50}, // half closed
    {12, 34}, // two thirds closed
    {13, 20}, // 80% closed
    {15, 34}, // start reopening
    {16, 67},
    {17, 100},
};

bool isDateInSeason(const tm &localTime) {
    const int month = localTime.tm_mon + 1;
    const int day = localTime.tm_mday;

    if (month < 5 || month > 9) return false;
    if (month == 5 && day < 15) return false;
    if (month == 9 && day > 15) return false;
    return true;
}

bool targetForHour(int hour, int &openPercent) {
    if (hour < 10) return false;
    if (hour >= 18) return false;

    if (hour == 14) {
        openPercent = 20;
        return true;
    }

    for (const auto &entry : kSchedule) {
        if (entry.hour == hour) {
            openPercent = entry.openPercent;
            return true;
        }
    }

    return false;
}

bool localTimeReady(tm &localTime) {
    time_t now = time(nullptr);
    if (now < kValidEpochThreshold) return false;
    return localtime_r(&now, &localTime) != nullptr;
}

void restoreLastKnownTimeIfNeeded() {
    if (s_timeFallbackAttempted) return;
    s_timeFallbackAttempted = true;

    if (time(nullptr) >= kValidEpochThreshold) return;

    uint64_t savedEpoch = 0;
    if (!nvs_read_u64(NVS_KEY_LAST_EPOCH, savedEpoch) ||
        savedEpoch < static_cast<uint64_t>(kValidEpochThreshold)) {
        Serial.println("Summer shade: no saved time available for offline mode");
        return;
    }

    timeval tv {};
    tv.tv_sec = static_cast<time_t>(savedEpoch);
    settimeofday(&tv, nullptr);
    s_lastPersistedEpoch = savedEpoch;
    Serial.printf("Summer shade: restored offline clock from saved epoch %llu\n",
                  static_cast<unsigned long long>(savedEpoch));
}

void persistCurrentTimeIfNeeded(bool force = false) {
    const time_t now = time(nullptr);
    if (now < kValidEpochThreshold) return;

    const uint64_t epoch = static_cast<uint64_t>(now);
    const bool timeJumped =
        s_lastPersistedEpoch != 0 &&
        (epoch > s_lastPersistedEpoch + 120 || s_lastPersistedEpoch > epoch + 120);
    const uint32_t nowMs = millis();
    if (!force && !timeJumped && s_lastTimePersistMs != 0 &&
        static_cast<int32_t>(nowMs - s_lastTimePersistMs) < static_cast<int32_t>(kTimePersistIntervalMs)) {
        return;
    }

    nvs_write_u64(NVS_KEY_LAST_EPOCH, epoch);
    s_lastPersistedEpoch = epoch;
    s_lastTimePersistMs = nowMs;
}

void ensureNtpConfigured() {
    if (WiFi.status() != WL_CONNECTED) return;

    const uint32_t nowMs = millis();
    if (s_ntpConfigured &&
        static_cast<int32_t>(nowMs - s_lastNtpConfigMs) < static_cast<int32_t>(kNtpRetryIntervalMs)) {
        return;
    }

    configTzTime(kTimeZoneParis, "pool.ntp.org", "time.nist.gov");
    s_ntpConfigured = true;
    s_lastNtpConfigMs = nowMs;
    Serial.println("Summer shade: NTP time sync requested");
}

bool resolveTargetDescription(std::string &description) {
    const auto &remotes = IOHC::iohcRemote1W::getInstance()->getRemotes();
    for (const auto &remote : remotes) {
        if (remote.description == summer_shade_target || remote.name == summer_shade_target) {
            description = remote.description;
            return true;
        }
    }
    return false;
}

void sendTargetPosition(const std::string &description, int openPercent) {
    Tokens cmd;
    cmd.emplace_back("position");
    cmd.push_back(description);
    cmd.push_back(std::to_string(openPercent));

    Serial.printf("Summer shade: setting %s to %d%% open\n", description.c_str(), openPercent);
    addLogMessage(String("Summer shade: setting ") + description.c_str() +
                  " to " + String(openPercent) + "% open");
    IOHC::iohcRemote1W::getInstance()->cmd(IOHC::RemoteButton::Position, &cmd);
}

void schedulerTask(void *) {
    setenv("TZ", kTimeZoneParis, 1);
    tzset();
    restoreLastKnownTimeIfNeeded();
    // Let the ESP32 Wi-Fi and SX1276 initialization settle before the first
    // scheduled transmission.
    vTaskDelay(pdMS_TO_TICKS(kStartupGraceMs));

    while (true) {
        ensureNtpConfigured();

        tm localTime {};
        const bool timeReady = localTimeReady(localTime);
        if (timeReady) {
            persistCurrentTimeIfNeeded();
        }

        if (timeReady && isDateInSeason(localTime)) {
            int openPercent = 0;
            if (targetForHour(localTime.tm_hour, openPercent) &&
                (s_lastAppliedYday != localTime.tm_yday ||
                 s_lastAppliedOpenPercent != openPercent)) {
                std::string description;
                if (resolveTargetDescription(description)) {
                    sendTargetPosition(description, openPercent);
                    persistCurrentTimeIfNeeded(true);
                    s_lastAppliedYday = localTime.tm_yday;
                    s_lastAppliedOpenPercent = openPercent;
                } else {
                    Serial.printf("Summer shade: target '%s' not found\n", summer_shade_target.c_str());
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kCheckIntervalMs));
    }
}
} // namespace

void initSummerShadeScheduler() {
    if (s_schedulerTask) return;

    if (xTaskCreatePinnedToCore(schedulerTask, "summerShade", 4096, nullptr,
                                1, &s_schedulerTask, tskNO_AFFINITY) != pdPASS) {
        Serial.println("Summer shade: failed to create scheduler task");
        s_schedulerTask = nullptr;
        return;
    }

    Serial.printf("Summer shade: enabled for target '%s'\n", summer_shade_target.c_str());
}

#endif // SUMMER_SHADE_AUTOMATION
