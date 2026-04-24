/*
 * ================================================================
 *  SMART WET AND DRY SEGREGATOR DUSTBIN
 *  IoT-Based Smart Campus Waste Management System
 *  BMIT2123 Internet of Things — RDS1S3G2
 *
 *  LED Pattern Reference (Module 6 — no buzzer):
 *
 *    Event              LED Pattern
 *    ─────────────────────────────────────────
 *    Startup            Red → Yellow → Green cycling until WiFi/MQTT ready
 *    Ready              Green ON steady
 *    Sorting            Yellow ON steady
 *    Bin Full           Red ON steady
 *    Liquid Leak        Red ON Steady
 *    Fire Risk          Red + Yellow + Green ALL blink (200 ms)
 *    Temp Warning       Yellow MEDIUM blink (400 ms)
 *    High Humidity      Yellow MEDIUM blink (400 ms)
 *
 *  Wiring:
 *    Ultrasonic 1 TRIG/ECHO  → GPIO 16 / 17
 *    Ultrasonic 2 TRIG/ECHO  → GPIO 38 / 39  (wet bin)
 *    Ultrasonic 3 TRIG/ECHO  → GPIO 36 / 37  (dry bin)
 *    Moisture Sensor OUT     → GPIO 10
 *    Servo Signal            → GPIO 18
 *    DHT11 Data              → GPIO 4
 *    Float Switch            → GPIO 13        (INPUT_PULLUP)
 *    LED Green               → GPIO 20
 *    LED Yellow              → GPIO 21
 *    LED Red                 → GPIO 47
 * ================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ================================================================
//  SECTION 1 — CREDENTIALS
// ================================================================

#define WIFI_SSID       "B100M"      //A0901-TIME2.4Ghz    //B100M
#define WIFI_PASSWORD   "12345678"   //a0901bcd            //12345678

// ── EMQX Cloud (TLS/SSL port 8883) ──────────────────────────────
#define MQTT_BROKER_IP  "m2e87f21.ala.asia-southeast1.emqxsl.com"
#define MQTT_PORT       8883
#define MQTT_CLIENT_ID  "SmartBin_ESP32"
#define MQTT_USER       "smartbin"
#define MQTT_PASS       "smartbin06$"

// ── EMQX CA Certificate (DigiCert Global Root G2) ───────────────
// Used by WiFiClientSecure to verify the EMQX server identity.
static const char EMQX_CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

// ================================================================
//  SECTION 2 — MQTT TOPICS
// ================================================================

#define TOPIC_BIN_LEVELS  "smartbin/bin/levels"
#define TOPIC_ENV         "smartbin/environment"
#define TOPIC_ALERT       "smartbin/alerts"
#define TOPIC_SORT_EVENT  "smartbin/sort/event"
#define TOPIC_HEARTBEAT   "smartbin/system/heartbeat"

// Control topics — Node-RED dashboard sends commands to these
#define TOPIC_CTRL_SERVO    "smartbin/control/servo"
#define TOPIC_CTRL_DHT11    "smartbin/control/dht11"
#define TOPIC_CTRL_LED      "smartbin/control/led"
#define TOPIC_CTRL_MOISTURE "smartbin/control/moisture"

// ── Boot-time sync with Firebase (via Node-RED bridge) ──────────
// ESP32 publishes its local date; Node-RED replies with today's
// running totals so the counter survives reboots.
#define TOPIC_SYNC_REQUEST  "smartbin/request_sync"
#define TOPIC_SYNC_RESPONSE "smartbin/sync_response"
#define TOPIC_DAILY_RESET   "smartbin/daily_reset"

// ================================================================
//  SECTION 3 — PIN DEFINITIONS
// ================================================================

// Ultrasonic sensors
static constexpr uint8_t PIN_TRIG1 = 16, PIN_ECHO1 = 17;   // Module 1 — entry
static constexpr uint8_t PIN_TRIG2 = 38, PIN_ECHO2 = 39;   // Module 3 — wet bin
static constexpr uint8_t PIN_TRIG3 = 36, PIN_ECHO3 = 37;   // Module 3 — dry bin

// Module 2
static constexpr uint8_t PIN_MOISTURE = 10;   // Moisture sensor ADC
static constexpr uint8_t PIN_SERVO    = 18;

// Module 4
static constexpr uint8_t PIN_DHT   = 4;
static constexpr uint8_t PIN_FLOAT = 13;   // INPUT_PULLUP — HIGH = leak detected

// Module 6 — LEDs only, no buzzer
static constexpr uint8_t PIN_LED_GREEN  = 20;
static constexpr uint8_t PIN_LED_YELLOW = 21;
static constexpr uint8_t PIN_LED_RED    = 47;

// ================================================================
//  SECTION 4 — SYSTEM CONSTANTS
// ================================================================

// Ultrasonic
static constexpr uint8_t  US_SAMPLES         = 5;
static constexpr uint32_t US_TIMEOUT_US      = 30000;
static constexpr uint8_t  US_SAMPLE_DELAY_MS = 10;
static constexpr float    SPEED_OF_SOUND     = 0.0343f;

// Module 1 — entry detection
static constexpr float    ENTRY_THRESHOLD_CM = 20.0f;
static constexpr uint32_t ENTRY_STABLE_MS    = 600; //Object must stay for 0.6 second to confirm it is real waste
static constexpr uint16_t READY_POLL_MS      = 300;

// Module 2 — sorting
static constexpr uint16_t MOISTURE_THRESHOLD = 100;//Moisture reading above 100 = WET → servo goes to 150°. Below 100 = DRY → servo goes to 30°
static constexpr uint8_t  SERVO_CENTER_DEG   = 90; //Servo home position
static constexpr uint8_t  SERVO_WET_DEG      = 150;//wet degree 150
static constexpr uint8_t  SERVO_DRY_DEG      = 30;//dry degree 30
static constexpr uint32_t SERVO_OPEN_MS      = 3000;//servo open 2 seconds to throw
static constexpr uint16_t SERVO_RESET_MS     = 500;//Settle time after servo returns to center

// Module 3 — bin capacity
// BIN_DEPTH_CM  = distance sensor reads when the bin is completely EMPTY (0%)
// BIN_HEIGHT_CM = (BIN_DEPTH_CM − full-trigger cm) / 0.90  →  so that trigger cm = 90%
// formula: fill% = ((BIN_DEPTH_CM − reading) / BIN_HEIGHT_CM) × 100
// calibration: 17 cm = 0%,  3 cm ≈ 90% (alert),  ~6.2 cm ≈ 70% (near-full)
static constexpr float    BIN_DEPTH_CM  = 17.0f;  // Distance from sensor to bin bottom (empty = 0%)
static constexpr float    BIN_HEIGHT_CM = 12.0f;  // Usable fill height inside the bin
//Therefore, the full trigger distance ≈ 5 cm (when waste reaches near the sensor)

static constexpr float    BIN_FULL_PCT      = 90.0f; // >= 90% → Red LED + alert  (~3.0 cm reading)
static constexpr float    BIN_NEAR_FULL_PCT = 70.0f; // >= 70% → NEAR_FULL        (~6.2 cm reading)

// Module 4 — safety thresholds
static constexpr float    TEMP_FIRE_C     = 50.0f; //50℃ detection fire
static constexpr float    TEMP_WARN_C     = 45.0f; //45℃ detection warning
static constexpr float    HUMID_ALERT_PCT = 80.0f; //alert when huminity 80%

// Background timing
static constexpr uint32_t ENV_CHECK_MS    = 5000;  // Module 4: DHT11 + float every 5 s
static constexpr uint32_t MQTT_PUBLISH_MS = 10000; // Module 5: MQTT upload every 10 s

// LED blink speeds (milliseconds per half-cycle)
static constexpr uint16_t BLINK_MEDIUM = 400;   // Temp warn / High humidity → Yellow medium
static constexpr uint16_t BLINK_SLOW   = 500;   // Sorting       → Yellow slow
static constexpr uint16_t BLINK_FIRE   = 200;   // Fire risk     → All LEDs

// Startup — each LED lights this long before switching to the next
static constexpr uint16_t STARTUP_STEP_MS = 300;

// ================================================================
//  SECTION 5 — STATE MACHINE
// ================================================================

enum class SystemState : uint8_t {
    STARTUP,        // WiFi/MQTT connecting — Red→Yellow→Green cycling
    MONITORING,     // Idle — Green ON steady
    VALIDATING,     // Confirming 0.6 s presence — Green ON steady
    SORTING,        // Classifying waste — Yellow SLOW blink
    BIN_FULL,       // Bin >= 90% — Red ON steady
    LEAK,           // Float switch HIGH — Red Steady
    FIRE_RISK,      // Temp > 50 C — ALL LEDs blink
    TEMP_WARNING,   // Temp 45–50 C — Yellow MEDIUM blink
    HIGH_HUMIDITY   // Humidity > 80% — Yellow MEDIUM blink
};

// ================================================================
//  SECTION 6 — GLOBAL OBJECTS & STATE
// ================================================================

static Servo             gateServo;
static DHT               dht(PIN_DHT, DHT11);
static WiFiClientSecure  wifiClient;
static PubSubClient      mqttClient(wifiClient);

static SystemState  currentState    = SystemState::STARTUP;
static uint32_t     entryDetectedAt = 0;
static uint32_t     lastEnvCheck    = 0;
static uint32_t     lastMqttPub     = 0;
static uint32_t     totalSorted     = 0;
static uint32_t     totalWet        = 0;
static uint32_t     totalDry        = 0;

// Daily-reset tracking (populated after NTP sync)
static char         lastDateStr[11] = "";  // "YYYY-MM-DD" in MYT (UTC+8)
static uint32_t     lastDayCheck    = 0;   // millis() of last checkDailyReset call

// Cached sensor readings
static float lastTempC    = 0.0f;
static float lastHumidPct = 0.0f;
static bool  lastLeak     = false;
static float lastWetPct   = 0.0f;
static float lastDryPct   = 0.0f;
static bool  binFullWet   = false;
static bool  binFullDry   = false;

// ── Runtime control settings (configurable via Node-RED dashboard) ──
// All three components default ON.  If moisture is toggled OFF the system
// automatically classifies every item as DRY (dry-bin mode).
// Servo angles default to hardware constants; dashboard can override live.
static uint8_t cfgServoDryDeg  = SERVO_DRY_DEG;   // 30°  default
static uint8_t cfgServoWetDeg  = SERVO_WET_DEG;   // 150° default
static bool    cfgDht11En      = true;   // DHT11 ON by default
static bool    cfgLedEn        = true;   // LEDs  ON by default
static bool    cfgMoistureEn   = true;   // Moisture ON; OFF → all items = DRY

// ================================================================
//  SECTION 7 — LED HELPER FUNCTIONS  (Module 6)
// ================================================================

/**
 * ledsOff() — turn all three LEDs off
 */
static void ledsOff() {
    digitalWrite(PIN_LED_GREEN,  LOW);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED,    LOW);
}

/**
 * setLED() — one LED steady on, the other two off.
 * Respects cfgLedEn: if LEDs are disabled, turns all off.
 */
static void setLED(uint8_t activePin) {
    if (!cfgLedEn) { ledsOff(); return; }
    digitalWrite(PIN_LED_GREEN,  (activePin == PIN_LED_GREEN)  ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, (activePin == PIN_LED_YELLOW) ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,    (activePin == PIN_LED_RED)    ? HIGH : LOW);
}

/**
 * blinkOneLED()
 * Non-blocking blink of a single LED at intervalMs.
 * Uses (millis() / intervalMs) % 2 to decide ON or OFF.
 * The blink state is derived purely from the current time, so it is
 * always consistent regardless of how often or how irregularly the
 * function is called — no static ledOn flag that drifts out of sync
 * when delays inside classifyAndSort() interrupt the call pattern.
 */
static void blinkOneLED(uint8_t pin, uint16_t intervalMs) {
    if (!cfgLedEn) { ledsOff(); return; }
    bool ledOn = ((millis() / intervalMs) % 2 == 0);
    digitalWrite(PIN_LED_GREEN,  (pin == PIN_LED_GREEN  && ledOn) ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, (pin == PIN_LED_YELLOW && ledOn) ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,    (pin == PIN_LED_RED    && ledOn) ? HIGH : LOW);
}

/**
 * blinkAllLEDs()
 * All three LEDs blink together at intervalMs.
 * Same millis()-division approach as blinkOneLED — time-based,
 * never drifts regardless of call frequency.
 */
static void blinkAllLEDs(uint16_t intervalMs) {
    if (!cfgLedEn) { ledsOff(); return; }
    bool ledOn = ((millis() / intervalMs) % 2 == 0);
    digitalWrite(PIN_LED_GREEN,  ledOn ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, ledOn ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,    ledOn ? HIGH : LOW);
}

/**
 * startupBlink()
 * Cycles Red → Yellow → Green repeatedly during WiFi/MQTT connection.
 * Blocking per-step but short (STARTUP_STEP_MS = 300 ms each).
 * Called once per iteration of the startup while-loop in setup().
 * Note: bypasses cfgLedEn so the startup animation always runs.
 */
static void startupBlink() {
    // Direct digitalWrite — startup must always blink regardless of cfgLedEn
    digitalWrite(PIN_LED_RED, HIGH); digitalWrite(PIN_LED_YELLOW, LOW); digitalWrite(PIN_LED_GREEN, LOW);
    delay(STARTUP_STEP_MS);
    digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_YELLOW, HIGH); digitalWrite(PIN_LED_GREEN, LOW);
    delay(STARTUP_STEP_MS);
    digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_YELLOW, LOW); digitalWrite(PIN_LED_GREEN, HIGH);
    delay(STARTUP_STEP_MS);
    ledsOff();
    delay(100);
}

// ================================================================
//  SECTION 7b — MQTT CONTROL CALLBACK
//  MUST be placed here, AFTER ledsOff() is defined above.
// ================================================================

/**
 * mqttCallback()
 * Called by PubSubClient whenever a subscribed topic receives a message.
 * Registered via mqttClient.setCallback(mqttCallback) in setup().
 *
 * Topics handled:
 *   smartbin/control/servo    → {"dry_deg":30, "wet_deg":150}
 *   smartbin/control/dht11    → {"enabled":true}  or {"enabled":false}
 *   smartbin/control/led      → {"enabled":true}  or {"enabled":false}
 *   smartbin/control/moisture → {"enabled":true}  or {"enabled":false}
 */
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Copy payload bytes to a null-terminated string
    char buf[128] = {};
    memcpy(buf, payload, min((unsigned int)127, length));

    Serial.printf("[MQTT RECEIVED] Topic: %s | Payload: %s\n", topic, buf);

    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, buf) != DeserializationError::Ok) {
        Serial.printf("[Control] Bad JSON on topic: %s\n", topic);
        return;
    }

    // ── Servo lid angle ───────────────────────────────────────────
    if (strcmp(topic, TOPIC_CTRL_SERVO) == 0) {
        if (doc.containsKey("dry_deg")) {
            uint8_t d = doc["dry_deg"].as<uint8_t>();
            if (d >= 10 && d <= 90)  cfgServoDryDeg = d;
        }
        if (doc.containsKey("wet_deg")) {
            uint8_t w = doc["wet_deg"].as<uint8_t>();
            if (w >= 91 && w <= 180) cfgServoWetDeg = w;
        }
        Serial.printf("[Control] Servo: dry=%d deg  wet=%d deg\n",
                      cfgServoDryDeg, cfgServoWetDeg);
    }
    // ── DHT11 enable / disable ────────────────────────────────────
    else if (strcmp(topic, TOPIC_CTRL_DHT11) == 0) {
        cfgDht11En = doc["enabled"].as<bool>();
        if (!cfgDht11En) {
            lastTempC    = 0.0f;  // clear stale readings on dashboard
            lastHumidPct = 0.0f;
        }
        Serial.printf("[Control] DHT11: %s\n", cfgDht11En ? "ON" : "OFF");
    }
    // ── LED enable / disable ──────────────────────────────────────
    else if (strcmp(topic, TOPIC_CTRL_LED) == 0) {
        cfgLedEn = doc["enabled"].as<bool>();
        if (!cfgLedEn) ledsOff();          // ledsOff() is defined above — safe
        Serial.printf("[Control] LED: %s\n", cfgLedEn ? "ON" : "OFF");
    }
    // ── Moisture sensor enable / disable ─────────────────────────
    else if (strcmp(topic, TOPIC_CTRL_MOISTURE) == 0) {
        cfgMoistureEn = doc["enabled"].as<bool>();
        Serial.printf("[Control] Moisture: %s\n",
                      cfgMoistureEn ? "ON" : "OFF (forced DRY)");
    }
    // ── Boot-sync response from Firebase (via Node-RED) ──────────
    // Payload: {"total_sorted":N,"total_wet":N,"total_dry":N,"new_day":bool}
    else if (strcmp(topic, TOPIC_SYNC_RESPONSE) == 0) {
        totalSorted = doc["total_sorted"].as<uint32_t>();
        totalWet    = doc["total_wet"]   .as<uint32_t>();
        totalDry    = doc["total_dry"]   .as<uint32_t>();
        bool newDay = doc["new_day"]     .as<bool>();
        Serial.printf("[Sync] Restored — sorted=%lu  wet=%lu  dry=%lu  new_day=%d\n",
                      totalSorted, totalWet, totalDry, newDay);
    }
}

// ================================================================
//  SECTION 8 — HARDWARE ABSTRACTION
// ================================================================

/**
 * measureDistance()
 * Fires HC-SR04 US_SAMPLES times, returns averaged cm.
 * Returns -1.0f if no valid echo (sensor disconnected or out of range).
 */
static float measureDistance(uint8_t trigPin, uint8_t echoPin) {
    long    total = 0;
    uint8_t valid = 0;

    for (uint8_t i = 0; i < US_SAMPLES; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
        if (dur > 0) { total += dur; valid++; }
        delay(US_SAMPLE_DELAY_MS);
    }

    if (valid == 0) return -1.0f;
    return ((static_cast<float>(total) / valid) * SPEED_OF_SOUND) / 2.0f;
}

/**
 * binFillPercent()
 * Formula: fill% = ((BIN_DEPTH - distance) / BIN_HEIGHT) * 100
 * EXAMPLE fill% = ((17 - reading) / 15.5) × 100  →  17 cm = 0%,  3 cm ≈ 90% alert
 * Returns -1.0f if sensor read failed.
 */
static float binFillPercent(float distCm) {
    if (distCm < 0.0f) return -1.0f;
    return constrain(((BIN_DEPTH_CM - distCm) / BIN_HEIGHT_CM) * 100.0f,
                     0.0f, 100.0f);
}

// ================================================================
//  SECTION 9 — MQTT / WIFI  (Module 5)
// ================================================================

/**
 * connectWiFi()
 * Attempts WiFi connection while running the Red→Yellow→Green
 * startup blink so the user can see the system is initialising.
 * Tries up to 20 × 500 ms = 10 seconds before giving up.
 */
static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        startupBlink();
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected — IP: %s\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Failed — running offline.");
    }
}

/**
 * subscribeControlTopics()
 * Subscribes to all Node-RED dashboard control topics.
 * Called after every successful MQTT connect / reconnect.
 */
static void subscribeControlTopics() {
    mqttClient.subscribe(TOPIC_CTRL_SERVO);
    mqttClient.subscribe(TOPIC_CTRL_DHT11);
    mqttClient.subscribe(TOPIC_CTRL_LED);
    mqttClient.subscribe(TOPIC_CTRL_MOISTURE);
    mqttClient.subscribe(TOPIC_SYNC_RESPONSE);
    Serial.println("[MQTT] Subscribed to control + sync topics.");
}

/**
 * connectMQTT()
 * Attempts MQTT connection while continuing startup blink.
 * Tries up to 5 times.
 */
static void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;

    uint8_t tries = 0;
    while (!mqttClient.connected() && tries < 5) { // try 5 times
        Serial.print("[MQTT] Connecting...");
        startupBlink();
        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
            Serial.println(" connected.");
            subscribeControlTopics();
        } else {
            Serial.printf(" failed (rc=%d). Retrying.\n", mqttClient.state());
            tries++;
        }
    }
}

/**
 * reconnectMQTT()
 * Silent reconnect called every loop() iteration.
 * Re-subscribes to control topics on each successful reconnect.
 */
static void reconnectMQTT() {
    if (mqttClient.connected()) return;
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
        Serial.println("[MQTT] Reconnected.");
        subscribeControlTopics();
    }
}

static void publishJSON(const char* topic, JsonDocument& doc) {
    if (!mqttClient.connected()) return;
    char buf[256];
    serializeJson(doc, buf);
    mqttClient.publish(topic, buf);
    Serial.printf("[MQTT] %s -> %s\n", topic, buf);
}

/**
 * checkDailyReset()
 * Compares today's Malaysia date (UTC+8) against the last stored date.
 * Resets totalSorted / totalWet / totalDry at midnight and publishes
 * a "daily_reset" MQTT event so Node-RED can archive yesterday's count.
 * Safe to call even before NTP syncs — returns immediately if clock is invalid.
 */
static void checkDailyReset() {
    time_t now = time(nullptr);
    if (now < 1000000000UL) return;  // NTP not synced yet

    struct tm t;
    localtime_r(&now, &t);   // localtime = Malaysia time (configTime UTC+8)
    char today[11];
    strftime(today, sizeof(today), "%Y-%m-%d", &t);

    // On very first call: just record the date without resetting
    if (lastDateStr[0] == '\0') {
        strncpy(lastDateStr, today, 10);
        lastDateStr[10] = '\0';
        return;
    }

    if (strcmp(today, lastDateStr) != 0) {
        Serial.printf("[DailyReset] %s → %s — counters reset to 0.\n",
                      lastDateStr, today);

        StaticJsonDocument<128> resetDoc;
        resetDoc["prev_date"]     = lastDateStr;
        resetDoc["new_date"]      = today;
        resetDoc["prev_sorted"]   = totalSorted;
        resetDoc["prev_wet"]      = totalWet;
        resetDoc["prev_dry"]      = totalDry;
        publishJSON(TOPIC_DAILY_RESET, resetDoc);

        totalSorted = 0;
        totalWet    = 0;
        totalDry    = 0;

        strncpy(lastDateStr, today, 10);
        lastDateStr[10] = '\0';
    }
}

// ================================================================
//  SECTION 10 — MODULE 3: BIN LEVEL MONITORING
// ================================================================

static void updateBinLevels() {
    float wetDist = measureDistance(PIN_TRIG2, PIN_ECHO2);
    float dryDist = measureDistance(PIN_TRIG3, PIN_ECHO3);

    // Skip if either sensor is disconnected
    if (wetDist < 0.0f || dryDist < 0.0f) {
        Serial.println("[Module3] Sensor error — check Ultrasonic 2/3 wiring.");
        return;
    }

    lastWetPct = binFillPercent(wetDist);
    lastDryPct = binFillPercent(dryDist);
    binFullWet = (lastWetPct >= BIN_FULL_PCT);
    binFullDry = (lastDryPct >= BIN_FULL_PCT);

    Serial.printf("[Module3] Wet: %.1f cm (%.1f%%)  Dry: %.1f cm (%.1f%%)\n",
              wetDist, lastWetPct,
              dryDist, lastDryPct);

    StaticJsonDocument<192> doc;
    doc["wet_pct"]    = lastWetPct;
    doc["dry_pct"]    = lastDryPct;
    doc["wet_dist"]   = round(wetDist * 10) / 10.0f;  // 1 decimal place, cm
    doc["dry_dist"]   = round(dryDist * 10) / 10.0f;
    doc["wet_status"] = (lastWetPct >= BIN_FULL_PCT)      ? "FULL"
                      : (lastWetPct >= BIN_NEAR_FULL_PCT)  ? "NEAR_FULL" : "OK";
    doc["dry_status"] = (lastDryPct >= BIN_FULL_PCT)      ? "FULL"
                      : (lastDryPct >= BIN_NEAR_FULL_PCT)  ? "NEAR_FULL" : "OK";
    publishJSON(TOPIC_BIN_LEVELS, doc);

    // Bin full → Red ON steady, no buzzer
    if (binFullWet || binFullDry) {
        Serial.println("[Module3] BIN FULL — Red LED on, input blocked.");

        StaticJsonDocument<96> alert;
        alert["type"]     = "BIN_FULL";
        alert["wet_full"] = binFullWet;
        alert["dry_full"] = binFullDry;
        publishJSON(TOPIC_ALERT, alert);

        // Only switch to BIN_FULL if no higher-priority alert is active
        if (currentState == SystemState::MONITORING ||
            currentState == SystemState::VALIDATING) {
            currentState = SystemState::BIN_FULL;
        }
    }
}

// ================================================================
//  SECTION 11 — MODULE 4: ENVIRONMENTAL MONITORING
// ================================================================

/**
 * runEnvCheck()
 * Reads DHT11 (if cfgDht11En) and float switch. Updates globals.
 * Sets currentState directly for each alert type so the
 * correct LED pattern runs in loop().
 * Returns true if any alert condition is active.
 */
static bool runEnvCheck() {
    lastLeak = (digitalRead(PIN_FLOAT) == HIGH);

    bool alertActive = false;

    // ── DHT11 (only if enabled from dashboard) ───────────────────
    if (cfgDht11En) {
        lastTempC    = dht.readTemperature();
        lastHumidPct = dht.readHumidity();

        if (isnan(lastTempC) || isnan(lastHumidPct)) {
            Serial.println("[Module4] DHT11 read failed.");
            lastTempC = 0.0f; lastHumidPct = 0.0f;
        } else {
            Serial.printf("[Module4] Temp: %.1f C  Humidity: %.1f%%\n",
                          lastTempC, lastHumidPct);

            // Fire risk — ALL LEDs blink (highest priority)
            if (lastTempC > TEMP_FIRE_C) {
                if (currentState != SystemState::FIRE_RISK)
                    Serial.println("[Module4] FIRE RISK — All LEDs blinking.");
                alertActive = true;
                currentState = SystemState::FIRE_RISK;

                StaticJsonDocument<96> alert;
                alert["type"]   = "FIRE_RISK";
                alert["temp_c"] = lastTempC;
                publishJSON(TOPIC_ALERT, alert);
            }
            // Temp warning — Yellow medium blink
            else if (lastTempC > TEMP_WARN_C) {
                if (currentState != SystemState::TEMP_WARNING)
                    Serial.println("[Module4] TEMP WARNING — Yellow medium blink.");
                alertActive = true;
                if (currentState == SystemState::MONITORING)
                    currentState = SystemState::TEMP_WARNING;

                StaticJsonDocument<96> alert;
                alert["type"]   = "TEMP_WARNING";
                alert["temp_c"] = lastTempC;
                publishJSON(TOPIC_ALERT, alert);
            }

            // High humidity — Yellow medium blink
            if (lastHumidPct > HUMID_ALERT_PCT) {
                if (currentState != SystemState::HIGH_HUMIDITY)
                    Serial.println("[Module4] HIGH HUMIDITY — Yellow medium blink.");
                alertActive = true;
                if (currentState == SystemState::MONITORING)
                    currentState = SystemState::HIGH_HUMIDITY;

                StaticJsonDocument<96> alert;
                alert["type"]      = "HIGH_HUMIDITY";
                alert["humid_pct"] = lastHumidPct;
                publishJSON(TOPIC_ALERT, alert);
            }
        }
    } else {
        Serial.println("[Module4] DHT11 disabled.");
    }

    // Float switch — liquid leak → Red FAST steady, no buzzer
    if (lastLeak) {
        Serial.println("[Module4] LEAK — Red Steady, bin blocked.");
        alertActive = true;
        currentState = SystemState::LEAK;

        StaticJsonDocument<64> alert;
        alert["type"] = "LIQUID_LEAK";
        publishJSON(TOPIC_ALERT, alert);
    }

    // Publish env data for dashboard graph every check
    StaticJsonDocument<96> envDoc;
    envDoc["temp_c"]    = lastTempC;
    envDoc["humid_pct"] = lastHumidPct;
    envDoc["leak"]      = lastLeak;
    publishJSON(TOPIC_ENV, envDoc);

    return alertActive;
}

// ================================================================
//  SECTION 12 — MODULE 2: CLASSIFICATION & SORTING
// ================================================================

static void classifyAndSort() {
    bool isWet   = false;
    int  moisture = 0;

    if (!cfgMoistureEn) {
        // Moisture sensor disabled — all waste defaults to DRY bin
        Serial.println("[Module2] Moisture sensor disabled — classifying as DRY.");
    } else {
        moisture = analogRead(PIN_MOISTURE);
        isWet    = (moisture > MOISTURE_THRESHOLD);
        Serial.printf("[Module2] Moisture ADC: %d — %s waste\n",
                      moisture, isWet ? "WET" : "DRY");
    }

    // Use dashboard-configurable servo angles
    gateServo.write(isWet ? cfgServoWetDeg : cfgServoDryDeg);
    Serial.printf("[Module2] Servo → %d° (%s)\n",
                  isWet ? cfgServoWetDeg : cfgServoDryDeg,
                  isWet ? "wet" : "dry");
    delay(SERVO_OPEN_MS);            // Hold gate open 3 s
    gateServo.write(SERVO_CENTER_DEG);
    delay(SERVO_RESET_MS);

    totalSorted++;
    if (isWet) totalWet++; else totalDry++;

    StaticJsonDocument<128> doc;
    doc["type"]         = isWet ? "WET" : "DRY";
    doc["moisture_adc"] = moisture;
    doc["total_sorted"] = totalSorted;
    doc["total_wet"]    = totalWet;
    doc["total_dry"]    = totalDry;
    publishJSON(TOPIC_SORT_EVENT, doc);
}

// ================================================================
//  SECTION 13 — STATE HANDLERS
// ================================================================

// ── Module 1: MONITORING ─────────────────────────────────────────
// Green ON steady. Entry sensor polls every 300 ms.
static SystemState handleMonitoring() {
    setLED(PIN_LED_GREEN);

    float d = measureDistance(PIN_TRIG1, PIN_ECHO1);
    if (d > 0.0f) Serial.printf("[Module1] Entry: %.1f cm\n", d);

    if (d > 0.0f && d < ENTRY_THRESHOLD_CM) {
        entryDetectedAt = millis();
        Serial.println("[Module1] Object detected — validating 0.6 s...");
        return SystemState::VALIDATING;
    }

    //delay(READY_POLL_MS);
    return SystemState::MONITORING;
}

// ── Module 1: VALIDATING ─────────────────────────────────────────
// Green ON steady. Confirms object stable for 1 second.
static SystemState handleValidating() {
    setLED(PIN_LED_GREEN);

    float d = measureDistance(PIN_TRIG1, PIN_ECHO1);
    if (d < 0.0f || d >= ENTRY_THRESHOLD_CM) {
        Serial.println("[Module1] Object left — false trigger.");
        return SystemState::MONITORING;
    }
    if ((millis() - entryDetectedAt) >= ENTRY_STABLE_MS) {
        Serial.println("[Module1] Confirmed > 0.6 s — activating sort.");
        return SystemState::SORTING;
    }

    delay(50);
    return SystemState::VALIDATING;
}

// ── Module 2: SORTING ─────────────────────────────────────────────
// Yellow ON steady. Input locked.
// After sort completes → Green ON (back to MONITORING).
static SystemState handleSorting() {
    setLED(PIN_LED_YELLOW);
    Serial.println("[Module2] Sort cycle START — input locked.");

    classifyAndSort();

    Serial.println("[Module2] Sort cycle END — Green LED on.");
    return SystemState::MONITORING;
}

// ── Module 3: BIN_FULL ────────────────────────────────────────────
// Red ON steady. No buzzer. Input blocked.
// Clears automatically when updateBinLevels() sees bin drop below
// BIN_FULL_PCT — no delay(), loop stays alive for MQTT + env checks.
static SystemState handleBinFull() {
    setLED(PIN_LED_RED);

    // binFullWet and binFullDry are updated by updateBinLevels()
    // which now always runs every 10 s even while in this state.
    if (!binFullWet && !binFullDry) {
        Serial.println("[Module3] Bin cleared — back to MONITORING.");
        return SystemState::MONITORING;
    }

    return SystemState::BIN_FULL;
}

// ── Module 4: LEAK ───────────────────────────────────────────────
// Red steady. No buzzer. Bin blocked.
// Clears when float switch returns LOW.
static SystemState handleLeak() {
    setLED(PIN_LED_RED);   // Steady red
    if (digitalRead(PIN_FLOAT) == LOW) {
        lastLeak = false;
        Serial.println("[Module4] Leak cleared — Green LED on.");
        return SystemState::MONITORING;
    }

    return SystemState::LEAK;
}

// ── Module 4: FIRE_RISK ──────────────────────────────────────────
// Red + Yellow + Green ALL blink (200 ms). No buzzer.
// Clears when temperature drops back to <= 50 C.
static SystemState handleFireRisk() {
    blinkAllLEDs(BLINK_FIRE);

    if (!isnan(lastTempC) && lastTempC <= TEMP_FIRE_C) {
        Serial.println("[Module4] Fire risk cleared — resuming.");
        return SystemState::MONITORING;
    }

    return SystemState::FIRE_RISK;
}

// ── Module 4: TEMP_WARNING ───────────────────────────────────────
// Yellow MEDIUM blink (400 ms). No buzzer.
// Clears when temperature drops below 45 C.
static SystemState handleTempWarning() {
    blinkOneLED(PIN_LED_YELLOW, BLINK_MEDIUM);

    if (!isnan(lastTempC) && lastTempC <= TEMP_WARN_C) {
        Serial.println("[Module4] Temp warning cleared — resuming.");
        return SystemState::MONITORING;
    }

    return SystemState::TEMP_WARNING;
}

// ── Module 4: HIGH_HUMIDITY ──────────────────────────────────────
// Yellow MEDIUM blink (400 ms). No buzzer.
// Clears when humidity drops below 80%.
static SystemState handleHighHumidity() {
    blinkOneLED(PIN_LED_YELLOW, BLINK_MEDIUM);

    if (!isnan(lastHumidPct) && lastHumidPct <= HUMID_ALERT_PCT) {
        Serial.println("[Module4] Humidity cleared — resuming.");
        return SystemState::MONITORING;
    }

    return SystemState::HIGH_HUMIDITY;
}

// ================================================================
//  SECTION 14 — SETUP
// ================================================================

void setup() {
    Serial.begin(115200);
    delay(200);

    // LED pins — set up first so startup blink works immediately
    pinMode(PIN_LED_GREEN,  OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED,    OUTPUT);
    ledsOff();

    // Ultrasonic pins
    for (auto& [t, e] : { std::pair{PIN_TRIG1, PIN_ECHO1},
                           std::pair{PIN_TRIG2, PIN_ECHO2},
                           std::pair{PIN_TRIG3, PIN_ECHO3} }) {
        pinMode(t, OUTPUT); digitalWrite(t, LOW);
        pinMode(e, INPUT);
    }

    // Module 4 sensors
    dht.begin();
    pinMode(PIN_FLOAT, INPUT_PULLUP);

    // Module 2 servo — park at center
    gateServo.attach(PIN_SERVO);
    gateServo.write(SERVO_CENTER_DEG);
    delay(600);

    // ── Startup sequence — Red→Yellow→Green blink during WiFi+MQTT ──
    currentState = SystemState::STARTUP;
    Serial.println("[Setup] Starting up — LED cycling while connecting...");

    wifiClient.setCACert(EMQX_CA_CERT);     // Verify EMQX TLS certificate
    mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
    mqttClient.setBufferSize(512);          // Larger buffer needed for TLS payloads
    mqttClient.setCallback(mqttCallback);   // Handle incoming control commands

    connectWiFi();   // Blinks during WiFi connection

    // ── NTP time sync (Malaysia time UTC+8) ──────────────────────
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("[NTP] Syncing time");
    {
        uint32_t t0 = millis();
        while (time(nullptr) < 1000000000UL && millis() - t0 < 8000) {
            delay(400);
            Serial.print(".");
        }
    }
    Serial.println(time(nullptr) > 1000000000UL ? " OK" : " timeout (proceeding)");

    connectMQTT();   // Blinks during MQTT connection + subscribes to control topics

    // ── Request today's counter from Firebase via Node-RED ───────
    {
        StaticJsonDocument<64> syncDoc;
        time_t now = time(nullptr);
        if (now > 1000000000UL) {
            struct tm t;
            localtime_r(&now, &t);
            char todayStr[11];
            strftime(todayStr, sizeof(todayStr), "%Y-%m-%d", &t);
            syncDoc["date"] = todayStr;
            // Seed lastDateStr so checkDailyReset() has a starting point
            strncpy(lastDateStr, todayStr, 10);
            lastDateStr[10] = '\0';
        } else {
            syncDoc["date"] = "unknown";
        }
        publishJSON(TOPIC_SYNC_REQUEST, syncDoc);
        // Give Node-RED ~2 s to reply before entering the state machine
        uint32_t t0 = millis();
        while (millis() - t0 < 2000) { mqttClient.loop(); delay(100); }
    }

    // Connection done — show solid green for 1 second, then go live
    setLED(PIN_LED_GREEN);
    delay(1000);

    currentState = SystemState::MONITORING;

    Serial.println("================================================");
    Serial.println("  Smart Wet & Dry Segregator Dustbin — READY");
    Serial.println("  BMIT2123 IoT — RDS1S3G2");
    Serial.println("  Entry    : < 20 cm for > 0.6 s");
    Serial.println("  Wet      : moisture > 100 -> servo wet deg");
    Serial.println("  Dry      : moisture < 100 -> servo dry deg");
    Serial.println("  Bin full : >= 90% -> Red ON");
    Serial.println("  Leak     : float HIGH -> Red steady");
    Serial.println("  Fire     : temp > 50 C -> ALL blink");
    Serial.println("  Temp warn: 45-50 C -> Yellow medium blink");
    Serial.println("  Humidity : > 80% -> Yellow medium blink");
    Serial.println("  Control  : MQTT smartbin/control/* topics");
    Serial.println("  Default  : ALL components ON (disable via dashboard)");
    Serial.printf ("  Servo    : dry=%d deg  wet=%d deg\n",
                   cfgServoDryDeg, cfgServoWetDeg);
    Serial.println("================================================");
}

// ================================================================
//  SECTION 15 — MAIN LOOP
// ================================================================

void loop() {
    uint32_t now = millis();

    // ── Module 5: Keep MQTT alive ─────────────────────────────────
    reconnectMQTT();
    mqttClient.loop();

    // ── Module 4: Background safety check every 5 s ──────────────
    // Always runs regardless of state — handlers need fresh values
    // to detect when a condition has cleared (e.g. temp drops back
    // below 50C). Blocking it was the reason alerts never cleared.
    if (now - lastEnvCheck >= ENV_CHECK_MS) {
        lastEnvCheck = now;
        runEnvCheck();
    }


    // ── Daily reset check every 60 s (midnight Malaysia time) ────
    if (now - lastDayCheck >= 60000UL) {
        lastDayCheck = now;
        checkDailyReset();
    }

    // ── Module 3 + Heartbeat: MQTT publish every 10 s ────────────
    // Always runs regardless of state — BIN_FULL handler needs
    // binFullWet/binFullDry to update so it can clear automatically
    // when the bin is emptied. Blocking it kept the bin stuck forever.
    if (now - lastMqttPub >= MQTT_PUBLISH_MS) {
        lastMqttPub = now;
        updateBinLevels();

        StaticJsonDocument<64> hb;
        hb["uptime_s"]     = now / 1000;
        hb["total_sorted"] = totalSorted;
        publishJSON(TOPIC_HEARTBEAT, hb);
    }

    // ── State machine dispatch ────────────────────────────────────
    switch (currentState) {
        case SystemState::STARTUP:
            // Should not reach here — handled in setup()
            currentState = SystemState::MONITORING;
            break;

        case SystemState::MONITORING:
            currentState = handleMonitoring();
            break;

        case SystemState::VALIDATING:
            currentState = handleValidating();
            break;

        case SystemState::SORTING:
            currentState = handleSorting();
            break;

        case SystemState::BIN_FULL:
            currentState = handleBinFull();
            break;

        case SystemState::LEAK:
            currentState = handleLeak();
            break;

        case SystemState::FIRE_RISK:
            currentState = handleFireRisk();
            break;

        case SystemState::TEMP_WARNING:
            currentState = handleTempWarning();
            break;

        case SystemState::HIGH_HUMIDITY:
            currentState = handleHighHumidity();
            break;

        default:
            currentState = SystemState::MONITORING;
            break;
    }
}