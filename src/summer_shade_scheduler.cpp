#include <summer_shade_scheduler.h>

#if defined(SUMMER_SHADE_AUTOMATION)

#include <Arduino.h>
#include <WiFi.h>
#include <iohcCryptoHelpers.h>
#include <iohcRemote1W.h>
#include <log_buffer.h>
#include <time.h>

namespace {
constexpr const char *kTimeZoneParis = "CET-1CEST,M3.5.0/2,M10.5.0/3";
constexpr uint32_t kCheckIntervalMs = 60UL * 1000UL;
constexpr uint32_t kNtpRetryIntervalMs = 10UL * 60UL * 1000UL;
constexpr time_t kValidEpochThreshold = 1700000000; // 2023-11-14

TaskHandle_t s_schedulerTask = nullptr;
uint32_t s_lastNtpConfigMs = 0;
bool s_ntpConfigured = false;
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
    while (true) {
        ensureNtpConfigured();

        tm localTime {};
        if (localTimeReady(localTime) && isDateInSeason(localTime)) {
            int openPercent = 0;
            if (targetForHour(localTime.tm_hour, openPercent) &&
                (s_lastAppliedYday != localTime.tm_yday ||
                 s_lastAppliedOpenPercent != openPercent)) {
                std::string description;
                if (resolveTargetDescription(description)) {
                    sendTargetPosition(description, openPercent);
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
