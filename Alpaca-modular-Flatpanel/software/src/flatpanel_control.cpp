#include "flatpanel_control.h"

#include "debug_monitor.h"

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <esp32-hal-adc.h>
#include <math.h>

namespace FlatPanel {

static Servo flapServo;
static Adafruit_BME280 bme;
static Preferences prefs;

static int g_brightness = 0;
static int g_rawPosition = 0;
static int g_mappedPosition = 0;
static int g_filteredPosition = 0;

static int g_sensorRawOpen = SENSOR_RAW_OPEN;
static int g_sensorRawClosed = SENSOR_RAW_CLOSED;
static int g_servoOpenUs = SERVO_OPEN_US;
static int g_servoClosedUs = SERVO_CLOSE_US;
static int g_servoPulseUs = SERVO_OPEN_US;
static bool g_servoSmoothEnabled = true;
static int g_servoMaxSpeedUsPerSec = SERVO_SPEED_DEFAULT_US_PER_SEC;

static constexpr uint8_t POSITION_FILTER_SAMPLES = 8;
static constexpr int POSITION_CHANGE_DEADBAND = 4;

static CoverStatus g_coverState = COVER_UNKNOWN;
static CalibratorStatus g_calState = CAL_OFF;
static bool g_calibratorEnabled = false;

static bool g_bmeAvailable = false;
static bool g_heaterEnabled = true;
static bool g_heaterManualMode = false;
static bool g_heaterOn = false;

static float g_temperatureC = NAN;
static float g_humidityPct = NAN;
static float g_pressureHpa = NAN;
static float g_dewPointC = NAN;

static unsigned long g_lastSensorReadMs = 0;
static constexpr unsigned long SENSOR_READ_INTERVAL_MS = 2000;
static unsigned long g_lastStatusLogMs = 0;
static constexpr unsigned long STATUS_LOG_INTERVAL_MS = 5000;
static bool g_panelPwmAttached = false;
static unsigned long g_lastPanelGateToggleMs = 0;
static bool g_panelGateTestLevel = false;

static const char* coverStatusName(CoverStatus state) {
    switch (state) {
        case COVER_NOT_PRESENT: return "not_present";
        case COVER_CLOSED: return "closed";
        case COVER_MOVING: return "moving";
        case COVER_OPEN: return "open";
        case COVER_ERROR: return "error";
        default: return "unknown";
    }
}

static const char* calibratorStatusName(CalibratorStatus state) {
    switch (state) {
        case CAL_NOT_PRESENT: return "not_present";
        case CAL_OFF: return "off";
        case CAL_NOT_READY: return "not_ready";
        case CAL_READY: return "ready";
        case CAL_ERROR: return "error";
        default: return "unknown";
    }
}

static float calcDewPointC(float tempC, float humidityPct) {
    if (isnan(tempC) || isnan(humidityPct) || humidityPct <= 0.0f) return NAN;
    const float a = 17.62f;
    const float b = 243.12f;
    const float gamma = log(humidityPct / 100.0f) + (a * tempC) / (b + tempC);
    return (b * gamma) / (a - gamma);
}

static int readFilteredPositionRaw() {
    if (!USE_POSITION_SENSOR) return 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < POSITION_FILTER_SAMPLES; ++i) {
        sum += analogRead(POS_PIN);
        delayMicroseconds(250);
    }
    return static_cast<int>(sum / POSITION_FILTER_SAMPLES);
}

static int applyDeadband(int currentValue, int previousValue) {
    return (abs(currentValue - previousValue) <= POSITION_CHANGE_DEADBAND) ? previousValue : currentValue;
}

static void loadCalibration() {
    prefs.begin("flatpanel", false);
    g_sensorRawOpen = prefs.getInt("raw_open", SENSOR_RAW_OPEN);
    g_sensorRawClosed = prefs.getInt("raw_closed", SENSOR_RAW_CLOSED);
    g_servoOpenUs = constrain(prefs.getInt("servo_open", SERVO_OPEN_US), SERVO_MIN_US, SERVO_MAX_US);
    g_servoClosedUs = constrain(prefs.getInt("servo_closed", SERVO_CLOSE_US), SERVO_MIN_US, SERVO_MAX_US);
    g_servoPulseUs = g_servoOpenUs;
    g_servoSmoothEnabled = prefs.getBool("servo_smooth", true);
    g_servoMaxSpeedUsPerSec = constrain(prefs.getInt("servo_speed", SERVO_SPEED_DEFAULT_US_PER_SEC), SERVO_SPEED_MIN_US_PER_SEC, SERVO_SPEED_MAX_US_PER_SEC);
    Monitor::print("[CAL] geladen openRaw=");
    Monitor::print(g_sensorRawOpen);
    Monitor::print(" closedRaw=");
    Monitor::print(g_sensorRawClosed);
    Monitor::print(" openServo=");
    Monitor::print(g_servoOpenUs);
    Monitor::print(" closedServo=");
    Monitor::print(g_servoClosedUs);
    Monitor::print(" servoSmooth=");
    Monitor::print(g_servoSmoothEnabled ? "on" : "off");
    Monitor::print(" servoSpeed=");
    Monitor::println(g_servoMaxSpeedUsPerSec);
}

static void saveCalibration() {
    prefs.putInt("raw_open", g_sensorRawOpen);
    prefs.putInt("raw_closed", g_sensorRawClosed);
    prefs.putInt("servo_open", g_servoOpenUs);
    prefs.putInt("servo_closed", g_servoClosedUs);
    Monitor::print("[CAL] gespeichert openRaw=");
    Monitor::print(g_sensorRawOpen);
    Monitor::print(" closedRaw=");
    Monitor::print(g_sensorRawClosed);
    Monitor::print(" openServo=");
    Monitor::print(g_servoOpenUs);
    Monitor::print(" closedServo=");
    Monitor::println(g_servoClosedUs);
}

static void refreshPosition() {
    if (!USE_POSITION_SENSOR) return;

    const int raw = readFilteredPositionRaw();
    g_rawPosition = applyDeadband(raw, g_rawPosition);
    g_filteredPosition = g_rawPosition;

    if (g_sensorRawOpen == g_sensorRawClosed) {
        g_mappedPosition = 0;
    } else {
        g_mappedPosition = map(g_filteredPosition, g_sensorRawOpen, g_sensorRawClosed, 0, 100);
        g_mappedPosition = constrain(g_mappedPosition, 0, 100);
    }

    if (COVER_STATE_FOLLOWS_COMMAND) return;

    if (g_mappedPosition >= SENSOR_CLOSED_THRESHOLD) {
        g_coverState = COVER_CLOSED;
    } else if (g_mappedPosition <= SENSOR_OPEN_THRESHOLD) {
        g_coverState = COVER_OPEN;
    } else {
        g_coverState = COVER_UNKNOWN;
    }
}

static void logStatus() {
    Monitor::print("[STATUS] cover=");
    Monitor::print(coverStatusName(g_coverState));
    Monitor::print(" cal=");
    Monitor::print(calibratorStatusName(g_calState));
    Monitor::print(" brightness=");
    Monitor::print(g_brightness);
    Monitor::print(" raw=");
    Monitor::print(g_rawPosition);
    Monitor::print(" mapped=");
    Monitor::print(g_mappedPosition);
    Monitor::print("% heater=");
    Monitor::print(g_heaterOn ? "on" : "off");

    if (g_bmeAvailable) {
        Monitor::print(" tempC=");
        Monitor::print(g_temperatureC);
        Monitor::print(" humPct=");
        Monitor::print(g_humidityPct);
        Monitor::print(" dewC=");
        Monitor::print(g_dewPointC);
    } else {
        Monitor::print(" bme=missing");
    }

    Monitor::println();
}

static void updateCalibratorState() {
    g_calState = g_calibratorEnabled ? CAL_READY : CAL_OFF;
}

static int moveServoToPulseUs(int targetPulseUs, bool useSmoothRamp) {
    const int target = constrain(targetPulseUs, SERVO_MIN_US, SERVO_MAX_US);
    const int start = g_servoPulseUs;
    const int distance = abs(target - start);

    if (!useSmoothRamp || !g_servoSmoothEnabled || distance < 4) {
        g_servoPulseUs = target;
        flapServo.writeMicroseconds(g_servoPulseUs);
        delay(60);
        return g_servoPulseUs;
    }

    const float rampDistanceUs = distance * constrain(SERVO_SMOOTH_RAMP_FRACTION, 0.05f, 0.45f);
    const float cruiseDistanceUs = max(0.0f, distance - (2.0f * rampDistanceUs));
    const float maxSpeedUsPerMs = max(0.1f, g_servoMaxSpeedUsPerSec / 1000.0f);
    const float accelerationUsPerMs2 = (maxSpeedUsPerMs * maxSpeedUsPerMs) / (2.0f * max(1.0f, rampDistanceUs));
    const unsigned long rampMs = max(1UL, static_cast<unsigned long>(ceilf(maxSpeedUsPerMs / accelerationUsPerMs2)));
    const unsigned long cruiseMs = static_cast<unsigned long>(ceilf(cruiseDistanceUs / maxSpeedUsPerMs));
    const unsigned long travelMs = (2UL * rampMs) + cruiseMs;
    const int steps = max(1, static_cast<int>(travelMs / SERVO_SMOOTH_STEP_MS));

    Monitor::print("[SERVO] Sanft ");
    Monitor::print(start);
    Monitor::print(" -> ");
    Monitor::print(target);
    Monitor::print(" us in ");
    Monitor::print(travelMs);
    Monitor::print(" ms ramp=");
    Monitor::print(static_cast<int>(SERVO_SMOOTH_RAMP_FRACTION * 100.0f));
    Monitor::print("% maxSpeed=");
    Monitor::println(g_servoMaxSpeedUsPerSec);

    for (int i = 1; i <= steps; ++i) {
        float elapsedMs = static_cast<float>(i) * SERVO_SMOOTH_STEP_MS;
        if (elapsedMs > static_cast<float>(travelMs)) elapsedMs = static_cast<float>(travelMs);
        float movedUs = 0.0f;

        if (elapsedMs <= rampMs) {
            movedUs = 0.5f * accelerationUsPerMs2 * elapsedMs * elapsedMs;
        } else if (elapsedMs <= (rampMs + cruiseMs)) {
            movedUs = rampDistanceUs + (maxSpeedUsPerMs * (elapsedMs - rampMs));
        } else {
            const float decelElapsedMs = elapsedMs - rampMs - cruiseMs;
            movedUs = rampDistanceUs + cruiseDistanceUs + (maxSpeedUsPerMs * decelElapsedMs) - (0.5f * accelerationUsPerMs2 * decelElapsedMs * decelElapsedMs);
        }

        const float progress = constrain(movedUs / distance, 0.0f, 1.0f);
        g_servoPulseUs = start + static_cast<int>(roundf((target - start) * progress));
        flapServo.writeMicroseconds(g_servoPulseUs);
        delay(SERVO_SMOOTH_STEP_MS);
    }

    g_servoPulseUs = target;
    flapServo.writeMicroseconds(g_servoPulseUs);
    delay(60);
    return g_servoPulseUs;
}

static void updateHeaterControl() {
    if (g_heaterManualMode) {
        digitalWrite(HEATER_PIN, g_heaterOn ? HIGH : LOW);
        return;
    }

    if (!g_bmeAvailable || isnan(g_temperatureC) || isnan(g_dewPointC) || !g_heaterEnabled) {
        g_heaterOn = false;
        digitalWrite(HEATER_PIN, LOW);
        return;
    }
    if (!g_heaterOn && g_temperatureC <= (g_dewPointC + DEWPOINT_ON_MARGIN_C)) {
        g_heaterOn = true;
    } else if (g_heaterOn && g_temperatureC > (g_dewPointC + DEWPOINT_OFF_MARGIN_C)) {
        g_heaterOn = false;
    }
    digitalWrite(HEATER_PIN, g_heaterOn ? HIGH : LOW);
}

static void readBme() {
    if (!USE_BME280 || !g_bmeAvailable) return;
    g_temperatureC = bme.readTemperature();
    g_humidityPct = bme.readHumidity();
    g_pressureHpa = bme.readPressure() / 100.0f;
    g_dewPointC = calcDewPointC(g_temperatureC, g_humidityPct);
    updateHeaterControl();
}

static void logI2CScan() {
    Monitor::print("[I2C] Scan SDA=IO");
    Monitor::print(I2C_SDA_PIN);
    Monitor::print(" SCL=IO");
    Monitor::println(I2C_SCL_PIN);

    uint8_t found = 0;
    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Monitor::print("[I2C] gefunden 0x");
            if (address < 16) Monitor::print("0");
            Monitor::println(address, HEX);
            ++found;
        }
        delay(2);
    }

    if (found == 0) {
        Monitor::println("[I2C] keine Geraete gefunden");
    }
}

static uint32_t panelDutyForBrightness(int value) {
    const int brightness = constrain(value, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    return PANEL_PWM_INVERTED ? (BRIGHTNESS_MAX - brightness) : brightness;
}

static void forcePanelOutputOff() {
    if (g_panelPwmAttached) {
        ledcDetachPin(LED_PIN);
        g_panelPwmAttached = false;
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

static void writePanelBrightness(int value) {
    if (PANEL_FORCE_OFF_TEST) {
        forcePanelOutputOff();
        return;
    }

    const int brightness = constrain(value, BRIGHTNESS_MIN, BRIGHTNESS_MAX);

    if (brightness == 0) {
        if (g_panelPwmAttached) {
            ledcDetachPin(LED_PIN);
            g_panelPwmAttached = false;
        }
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, PANEL_PWM_INVERTED ? HIGH : LOW);
        return;
    }

    if (!g_panelPwmAttached) {
        ledcAttachPin(LED_PIN, PANEL_PWM_CHANNEL);
        g_panelPwmAttached = true;
    }
    ledcWrite(PANEL_PWM_CHANNEL, panelDutyForBrightness(brightness));
}

void init() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(POS_PIN, INPUT);

    analogReadResolution(12);
    analogSetPinAttenuation(POS_PIN, ADC_11db);

    digitalWrite(HEATER_PIN, LOW);
    ledcSetup(PANEL_PWM_CHANNEL, PANEL_PWM_FREQ_HZ, PANEL_PWM_RESOLUTION_BITS);
    writePanelBrightness(0);

    flapServo.setPeriodHertz(SERVO_FREQ_HZ);
    flapServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    loadCalibration();

    if (USE_BME280) {
        g_bmeAvailable = bme.begin(0x76, &Wire);
        if (!g_bmeAvailable) g_bmeAvailable = bme.begin(0x77, &Wire);
    }
    Monitor::print("[BME280] ");
    Monitor::println(g_bmeAvailable ? "gefunden" : "nicht gefunden");
    if (USE_BME280 && !g_bmeAvailable) {
        logI2CScan();
    }

    g_rawPosition = readFilteredPositionRaw();
    g_filteredPosition = g_rawPosition;
    refreshPosition();
    if (COVER_STATE_FOLLOWS_COMMAND) g_coverState = COVER_OPEN;
    updateCalibratorState();
    readBme();
    logStatus();
}

void update() {
    if (PANEL_FORCE_OFF_TEST) {
        forcePanelOutputOff();
        return;
    }

    if (PANEL_GATE_TOGGLE_TEST) {
        const unsigned long now = millis();
        if (now - g_lastPanelGateToggleMs >= PANEL_GATE_TOGGLE_INTERVAL_MS) {
            g_lastPanelGateToggleMs = now;
            if (g_panelPwmAttached) {
                ledcDetachPin(LED_PIN);
                g_panelPwmAttached = false;
            }
            g_panelGateTestLevel = !g_panelGateTestLevel;
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, g_panelGateTestLevel ? HIGH : LOW);
            Monitor::print("[PANEL-TEST] IO");
            Monitor::print(LED_PIN);
            Monitor::print("=");
            Monitor::println(g_panelGateTestLevel ? "HIGH" : "LOW");
        }
        return;
    }

    refreshPosition();
    updateCalibratorState();
    const unsigned long now = millis();
    if (now - g_lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
        g_lastSensorReadMs = now;
        readBme();
    }
    if (now - g_lastStatusLogMs >= STATUS_LOG_INTERVAL_MS) {
        g_lastStatusLogMs = now;
        logStatus();
    }
}

void openCover() {
    Monitor::println("[COVER] Oeffnen");
    g_coverState = COVER_MOVING;
    moveServoToPulseUs(g_servoOpenUs, true);
    if (!g_servoSmoothEnabled) delay(SERVO_SETTLE_MS);
    refreshPosition();
    if (COVER_STATE_FOLLOWS_COMMAND || !USE_POSITION_SENSOR) g_coverState = COVER_OPEN;
    Monitor::print("[COVER] Status nach Oeffnen: ");
    Monitor::println(coverStatusName(g_coverState));
}

void closeCover() {
    Monitor::println("[COVER] Schliessen");
    g_coverState = COVER_MOVING;
    moveServoToPulseUs(g_servoClosedUs, true);
    if (!g_servoSmoothEnabled) delay(SERVO_SETTLE_MS);
    refreshPosition();
    if (COVER_STATE_FOLLOWS_COMMAND || !USE_POSITION_SENSOR) g_coverState = COVER_CLOSED;
    Monitor::print("[COVER] Status nach Schliessen: ");
    Monitor::println(coverStatusName(g_coverState));
}

void haltCover() {
    Monitor::println("[COVER] Stopp");
    g_servoPulseUs = 1500;
    flapServo.writeMicroseconds(g_servoPulseUs);
    refreshPosition();
    Monitor::print("[COVER] Status nach Stopp: ");
    Monitor::println(coverStatusName(g_coverState));
}

int setServoPulseUs(int pulseUs) {
    moveServoToPulseUs(pulseUs, true);
    refreshPosition();
    g_coverState = COVER_UNKNOWN;
    Monitor::print("[SERVO] Manuell pulseUs=");
    Monitor::println(g_servoPulseUs);
    return g_servoPulseUs;
}

int adjustServoPulseUs(int deltaUs) {
    return setServoPulseUs(g_servoPulseUs + deltaUs);
}

void setBrightness(int value) {
    g_brightness = constrain(value, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    writePanelBrightness(g_brightness);
    g_calibratorEnabled = (g_brightness > 0);
    updateCalibratorState();
    Monitor::print("[CAL] Helligkeit=");
    Monitor::print(g_brightness);
    Monitor::print(" state=");
    Monitor::println(calibratorStatusName(g_calState));
}

bool turnCalibratorOn(int value) {
    if (value < BRIGHTNESS_MIN || value > BRIGHTNESS_MAX) return false;
    g_brightness = value;
    writePanelBrightness(g_brightness);
    g_calibratorEnabled = true;
    updateCalibratorState();
    Monitor::print("[CAL] Ein Helligkeit=");
    Monitor::print(g_brightness);
    Monitor::print(" state=");
    Monitor::println(calibratorStatusName(g_calState));
    return true;
}

void turnCalibratorOff() {
    g_brightness = 0;
    writePanelBrightness(0);
    g_calibratorEnabled = false;
    updateCalibratorState();
    Monitor::println("[CAL] Aus");
}

int getBrightness() { return g_brightness; }
int getMaxBrightness() { return BRIGHTNESS_MAX; }
int getServoPulseUs() { return g_servoPulseUs; }
void setServoSmoothEnabled(bool enabled) {
    g_servoSmoothEnabled = enabled;
    prefs.putBool("servo_smooth", g_servoSmoothEnabled);
    Monitor::print("[SERVO] Sanftlauf=");
    Monitor::println(g_servoSmoothEnabled ? "on" : "off");
}
bool isServoSmoothEnabled() { return g_servoSmoothEnabled; }

void setServoMaxSpeedUsPerSec(int speedUsPerSec) {
    g_servoMaxSpeedUsPerSec = constrain(speedUsPerSec, SERVO_SPEED_MIN_US_PER_SEC, SERVO_SPEED_MAX_US_PER_SEC);
    prefs.putInt("servo_speed", g_servoMaxSpeedUsPerSec);
    Monitor::print("[SERVO] MaxSpeed=");
    Monitor::print(g_servoMaxSpeedUsPerSec);
    Monitor::println(" us/s");
}

int getServoMaxSpeedUsPerSec() { return g_servoMaxSpeedUsPerSec; }

CoverStatus getCoverState() { return g_coverState; }
CalibratorStatus getCalibratorState() { return g_calState; }

String coverStateString() {
    switch (g_coverState) {
        case COVER_CLOSED: return "geschlossen";
        case COVER_OPEN: return "offen";
        case COVER_MOVING: return "bewegt";
        case COVER_NOT_PRESENT: return "nicht vorhanden";
        case COVER_ERROR: return "fehler";
        default: return "unbekannt";
    }
}

int getRawPosition() { return g_rawPosition; }
int getFilteredPosition() { return g_filteredPosition; }
int getMappedPosition() { return g_mappedPosition; }
int getCalibrationOpenRaw() { return g_sensorRawOpen; }
int getCalibrationClosedRaw() { return g_sensorRawClosed; }
int getCalibrationOpenServoUs() { return g_servoOpenUs; }
int getCalibrationClosedServoUs() { return g_servoClosedUs; }

bool calibrateCurrentPositionAsOpen() {
    if (!USE_POSITION_SENSOR) return false;
    refreshPosition();
    g_sensorRawOpen = g_filteredPosition;
    g_servoOpenUs = g_servoPulseUs;
    if (g_sensorRawOpen == g_sensorRawClosed) g_sensorRawClosed = g_sensorRawOpen + 1;
    saveCalibration();
    refreshPosition();
    Monitor::print("[CAL] aktuelle Position als offen gesetzt raw=");
    Monitor::print(g_sensorRawOpen);
    Monitor::print(" servo=");
    Monitor::println(g_servoOpenUs);
    return true;
}

bool calibrateCurrentPositionAsClosed() {
    if (!USE_POSITION_SENSOR) return false;
    refreshPosition();
    g_sensorRawClosed = g_filteredPosition;
    g_servoClosedUs = g_servoPulseUs;
    if (g_sensorRawOpen == g_sensorRawClosed) g_sensorRawOpen = g_sensorRawClosed - 1;
    saveCalibration();
    refreshPosition();
    Monitor::print("[CAL] aktuelle Position als geschlossen gesetzt raw=");
    Monitor::print(g_sensorRawClosed);
    Monitor::print(" servo=");
    Monitor::println(g_servoClosedUs);
    return true;
}

void resetPositionCalibration() {
    g_sensorRawOpen = SENSOR_RAW_OPEN;
    g_sensorRawClosed = SENSOR_RAW_CLOSED;
    g_servoOpenUs = SERVO_OPEN_US;
    g_servoClosedUs = SERVO_CLOSE_US;
    saveCalibration();
    refreshPosition();
    Monitor::println("[CAL] Kalibrierung zurueckgesetzt");
}

bool bmeAvailable() { return g_bmeAvailable; }
float getTemperatureC() { return g_temperatureC; }
float getHumidityPercent() { return g_humidityPct; }
float getPressureHpa() { return g_pressureHpa; }
float getDewPointC() { return g_dewPointC; }
bool isHeaterOn() { return g_heaterOn; }

void setHeaterEnabled(bool enabled) {
    g_heaterEnabled = enabled;
    g_heaterManualMode = false;
    if (!enabled) {
        g_heaterOn = false;
        digitalWrite(HEATER_PIN, LOW);
    } else {
        updateHeaterControl();
    }
    Monitor::print("[HEATER] Automatik=");
    Monitor::print(g_heaterEnabled ? "on" : "off");
    Monitor::print(" output=");
    Monitor::println(g_heaterOn ? "on" : "off");
}

bool isHeaterEnabled() { return g_heaterEnabled; }

void setHeaterManual(bool on) {
    g_heaterManualMode = true;
    g_heaterOn = on;
    digitalWrite(HEATER_PIN, g_heaterOn ? HIGH : LOW);
    Monitor::print("[HEATER] Manuell output=");
    Monitor::println(g_heaterOn ? "on" : "off");
}

void setHeaterAutomatic() {
    g_heaterManualMode = false;
    g_heaterEnabled = true;
    updateHeaterControl();
    Monitor::print("[HEATER] Automatik wieder aktiv output=");
    Monitor::println(g_heaterOn ? "on" : "off");
}

bool isHeaterManualMode() { return g_heaterManualMode; }

} // namespace FlatPanel
