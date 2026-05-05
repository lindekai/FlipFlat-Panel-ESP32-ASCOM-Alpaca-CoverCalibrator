// ============================================================
// FlipFlat Panel ESP32 Firmware
// ASCOM Alpaca CoverCalibrator + Serial Interface
// ============================================================
//
// Author:     Kai Linde
// Version:    v1.0.0
// Build:      2026-05-05
// License:    MIT
//
// Repository: https://github.com/lindekai/FlipFlatPanel-ESP32
//
// ============================================================
// Project Overview:
//
// ESP32-basierte Steuerung für ein motorisiertes FlipFlat Panel
// mit ASCOM Alpaca Support, serieller Schnittstelle sowie
// erweiterten Sensor- und Heizfunktionen.
//
// ============================================================
// Based on / Inspired by:
//
// Original concept and initial rudimentary prototype:
//   Moritz Mayer
//   Dark Matters Community
//   https://discord.gg/darkmatters
//
// ============================================================
// Contributions (Kai Linde):
//
// - Complete firmware architecture and implementation
// - ESP32 / ESP32-S3 integration
// - ASCOM Alpaca CoverCalibrator API
// - Alpaca discovery (UDP)
// - Serial protocol compatibility (v3.0)
// - BME280 integration (temperature, humidity, pressure)
// - Heating control (MOSFET, 12V)
// - Hall sensor end-stop logic
// - Servo control and PWM management
//
// ============================================================
// Maintainer:
//
//   Kai Linde
//   https://github.com/lindekai/
//
// ============================================================
// Key Features:
//
// - ASCOM Alpaca CoverCalibrator (WiFi, Port 11111)
// - Alpaca Discovery (UDP, Port 32227)
// - Serial Protocol (USB, 57600 Baud)
// - PWM control for EL panel (flat field)
// - Heating control system
// - Environmental monitoring (BME280)
// - Optional Hall sensor end-stop detection
//
// ============================================================
// Hardware:
//
// - ESP32-WROOM-32 (Prototype) / ESP32-S3 (Production)
// - BME280 (I2C)
// - 2x IRLZ44N MOSFET (EL panel + heating)
// - Servo (cover mechanism)
// - 2x Hall sensors A3144 (optional)
// - AMS1117-3.3V voltage regulator
//
// ============================================================
// Pin Mapping:
//
// GPIO 18  -> Servo PWM (LEDC Channel 0)
// GPIO 19  -> EL panel PWM (LEDC Channel 1)
// GPIO 23  -> Heating (Digital / Low-Freq PWM)
// GPIO 21  -> I2C SDA (BME280)
// GPIO 22  -> I2C SCL (BME280)
// GPIO 34  -> Hall OPEN (Input, external pull-up)
// GPIO 35  -> Hall CLOSE (Input, external pull-up)
//
// ============================================================
// Changelog:
//
// v1.0.0 (2026-05-05)
// - Initial full release
// - Complete rewrite based on early prototype idea
// - Alpaca CoverCalibrator implemented
// - Serial protocol compatibility added
// - Sensor and heating system integrated
//
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <math.h>

// ============================================================
// Firmware-Version & Geräte-ID
// ============================================================
#define FIRMWARE_VERSION    "1.1"
#define FIRMWARE_NAME       "FlipFlatPanel ESP32"
#define DEVICE_GUID         "16c5e400-a3b1-11ed-87cd-0800200c9a66"
#define DEVICE_TYPE         "CoverCalibrator"
#define MANUFACTURER        "FlipFlatPanel Project"
#define ALPACA_PORT         11111
#define DISCOVERY_PORT      32227
#define SERIAL_BAUD         57600

// ============================================================
// Pin-Belegung
// ============================================================
#define PIN_SERVO           18
#define PIN_EL_FOIL         19
#define PIN_HEATER          23
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define PIN_HALL_OPEN       34   // Input-Only, externer Pull-Up
#define PIN_HALL_CLOSE      35   // Input-Only, externer Pull-Up

// LEDC PWM Kanäle
#define LEDC_SERVO_CH       0
#define LEDC_EL_CH           1
#define LEDC_HEATER_CH       2

// LEDC Einstellungen
#define LEDC_EL_FREQ        5000
#define LEDC_EL_BITS        8
#define LEDC_HEATER_FREQ    1000
#define LEDC_HEATER_BITS    8

// ============================================================
// Standard-Einstellungen
// ============================================================
#define DEFAULT_SERVO_OPEN    2500
#define DEFAULT_SERVO_CLOSE   540
#define DEFAULT_SERVO_SPEED   5
#define DEFAULT_SOFTCLOSE     false
#define DEFAULT_MINPWM        128
#define DEFAULT_HEATER_TARGET 20.0f
#define DEFAULT_HEATER_HYST   2.0f
#define DEFAULT_HEATER_ENABLE false
#define DEFAULT_HALL_ENABLE   false
#define DEFAULT_AP_SSID       "FlipFlatPanel"
#define DEFAULT_AP_PASS       "flatfield"

// Servo-Grenzen
#define SERVO_PULSE_MIN     500
#define SERVO_PULSE_MAX     2500

// ============================================================
// Cover & Calibrator States (ASCOM Enums)
// ============================================================
enum CoverStatus : int {
    COVER_NOT_PRESENT = 0,
    COVER_CLOSED      = 1,
    COVER_MOVING      = 2,
    COVER_OPEN        = 3,
    COVER_UNKNOWN     = 4,
    COVER_ERROR       = 5
};

enum CalibratorStatus : int {
    CAL_NOT_PRESENT = 0,
    CAL_OFF         = 1,
    CAL_NOT_READY   = 2,
    CAL_READY       = 3,
    CAL_UNKNOWN     = 4,
    CAL_ERROR       = 5
};

// ============================================================
// Globale Objekte
// ============================================================
WebServer server(ALPACA_PORT);
WiFiUDP udp;
Adafruit_BME280 bme;
Servo coverServo;
Preferences prefs;

// ============================================================
// Zustandsvariablen
// ============================================================
CoverStatus coverState       = COVER_UNKNOWN;
CalibratorStatus calState    = CAL_OFF;
int currentBrightness        = 0;
bool isConnected             = false;
uint32_t serverTransactionID = 0;

// BME280
bool bmeAvailable            = false;
float temperature            = 0.0f;
float humidity               = 0.0f;
float pressure               = 0.0f;
unsigned long lastBmeRead    = 0;
#define BME_READ_INTERVAL    2000   // ms

// Heizung
bool heaterEnabled           = false;
float heaterTarget           = DEFAULT_HEATER_TARGET;
float heaterHysteresis       = DEFAULT_HEATER_HYST;
bool heaterActive            = false;

// Servo-Einstellungen
int servoOpenPulse           = DEFAULT_SERVO_OPEN;
int servoClosePulse          = DEFAULT_SERVO_CLOSE;
int servoSpeed               = DEFAULT_SERVO_SPEED;
bool softCloseEnabled        = DEFAULT_SOFTCLOSE;
int minPWM                   = DEFAULT_MINPWM;
int currentPulsePos          = 0;

// Hall-Sensoren
bool hallEnabled             = DEFAULT_HALL_ENABLE;

// WiFi
String wifiSSID              = "";
String wifiPass              = "";
String apSSID                = DEFAULT_AP_SSID;
String apPass                = DEFAULT_AP_PASS;
bool wifiConnected           = false;
bool apActive                = false;

// Serieller Eingabepuffer
#define INPUT_BUFFER_SIZE    128
char inputBuffer[INPUT_BUFFER_SIZE];
int bufferPos                = 0;

// ============================================================
// Einstellungen: Laden / Speichern (NVS)
// ============================================================
void loadSettings() {
    prefs.begin("flipflat", true); // read-only
    servoOpenPulse    = prefs.getInt("servoOpen", DEFAULT_SERVO_OPEN);
    servoClosePulse   = prefs.getInt("servoClose", DEFAULT_SERVO_CLOSE);
    servoSpeed        = prefs.getInt("servoSpeed", DEFAULT_SERVO_SPEED);
    softCloseEnabled  = prefs.getBool("softClose", DEFAULT_SOFTCLOSE);
    minPWM            = prefs.getInt("minPWM", DEFAULT_MINPWM);
    heaterTarget      = prefs.getFloat("heatTarget", DEFAULT_HEATER_TARGET);
    heaterHysteresis  = prefs.getFloat("heatHyst", DEFAULT_HEATER_HYST);
    heaterEnabled     = prefs.getBool("heatEnable", DEFAULT_HEATER_ENABLE);
    hallEnabled       = prefs.getBool("hallEnable", DEFAULT_HALL_ENABLE);
    wifiSSID          = prefs.getString("wifiSSID", "");
    wifiPass          = prefs.getString("wifiPass", "");
    apSSID            = prefs.getString("apSSID", DEFAULT_AP_SSID);
    apPass            = prefs.getString("apPass", DEFAULT_AP_PASS);
    prefs.end();

    // Plausibilitäts-Check
    servoOpenPulse  = constrain(servoOpenPulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    servoClosePulse = constrain(servoClosePulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    servoSpeed      = constrain(servoSpeed, 1, 10);
    minPWM          = constrain(minPWM, 0, 254);
}

void saveSettings() {
    prefs.begin("flipflat", false); // read-write
    prefs.putInt("servoOpen", servoOpenPulse);
    prefs.putInt("servoClose", servoClosePulse);
    prefs.putInt("servoSpeed", servoSpeed);
    prefs.putBool("softClose", softCloseEnabled);
    prefs.putInt("minPWM", minPWM);
    prefs.putFloat("heatTarget", heaterTarget);
    prefs.putFloat("heatHyst", heaterHysteresis);
    prefs.putBool("heatEnable", heaterEnabled);
    prefs.putBool("hallEnable", hallEnabled);
    prefs.putString("wifiSSID", wifiSSID);
    prefs.putString("wifiPass", wifiPass);
    prefs.putString("apSSID", apSSID);
    prefs.putString("apPass", apPass);
    prefs.end();
}

// ============================================================
// WiFi Setup: AP + STA
// ============================================================
void setupWiFi() {
    // AP-Modus immer starten (für Konfiguration)
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSSID.c_str(), apPass.c_str());
    apActive = true;
    Serial.print("[WiFi] AP gestartet: ");
    Serial.print(apSSID);
    Serial.print(" / IP: ");
    Serial.println(WiFi.softAPIP());

    // STA-Modus: Ins Heim-WLAN verbinden (falls konfiguriert)
    if (wifiSSID.length() > 0) {
        Serial.print("[WiFi] Verbinde mit: ");
        Serial.println(wifiSSID);
        WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            Serial.print("\n[WiFi] Verbunden! IP: ");
            Serial.println(WiFi.localIP());
        } else {
            Serial.println("\n[WiFi] Verbindung fehlgeschlagen. Nur AP-Modus aktiv.");
        }
    } else {
        Serial.println("[WiFi] Kein WLAN konfiguriert. Nur AP-Modus.");
    }
}

// ============================================================
// BME280 Initialisierung
// ============================================================
void setupBME280() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (bme.begin(0x76, &Wire)) {
        bmeAvailable = true;
        Serial.println("[BME280] Sensor gefunden (0x76).");
    } else if (bme.begin(0x77, &Wire)) {
        bmeAvailable = true;
        Serial.println("[BME280] Sensor gefunden (0x77).");
    } else {
        Serial.println("[BME280] Sensor NICHT gefunden! Heizung deaktiviert.");
        heaterEnabled = false;
    }
}

void readBME280() {
    if (!bmeAvailable) return;
    if (millis() - lastBmeRead < BME_READ_INTERVAL) return;
    lastBmeRead = millis();
    temperature = bme.readTemperature();
    humidity    = bme.readHumidity();
    pressure    = bme.readPressure() / 100.0f; // hPa
}

// ============================================================
// Heizungsregelung (Schwellwert mit Hysterese)
// ============================================================
void updateHeater() {
    if (!heaterEnabled || !bmeAvailable) {
        if (heaterActive) {
            ledcWrite(LEDC_HEATER_CH, 0);
            heaterActive = false;
        }
        return;
    }

    if (temperature < (heaterTarget - heaterHysteresis)) {
        // Unter Schwelle: Heizung EIN
        if (!heaterActive) {
            ledcWrite(LEDC_HEATER_CH, 255);
            heaterActive = true;
            Serial.printf("[Heater] EIN (%.1f°C < %.1f°C)\n",
                temperature, heaterTarget - heaterHysteresis);
        }
    } else if (temperature >= heaterTarget) {
        // Zieltemperatur erreicht: Heizung AUS
        if (heaterActive) {
            ledcWrite(LEDC_HEATER_CH, 0);
            heaterActive = false;
            Serial.printf("[Heater] AUS (%.1f°C >= %.1f°C)\n",
                temperature, heaterTarget);
        }
    }
}

// ============================================================
// Hall-Sensoren: Endlagen lesen
// ============================================================
bool isHallOpen() {
    if (!hallEnabled) return false;
    return digitalRead(PIN_HALL_OPEN) == LOW; // A3144: LOW = Magnet erkannt
}

bool isHallClosed() {
    if (!hallEnabled) return false;
    return digitalRead(PIN_HALL_CLOSE) == LOW;
}

void updateCoverStateFromHall() {
    if (!hallEnabled) return;
    if (coverState == COVER_MOVING) return; // Während Bewegung nicht ändern

    if (isHallOpen())       coverState = COVER_OPEN;
    else if (isHallClosed()) coverState = COVER_CLOSED;
}

// ============================================================
// Servo-Bewegung
// ============================================================
void sendServoPulse(int pulseWidth_us) {
    coverServo.writeMicroseconds(pulseWidth_us);
    delay(20);
}

void moveServoDirect(int targetPulse) {
    int stepDelayMs = 44 - (servoSpeed * 4);
    int stepSize = 10;
    int current = (currentPulsePos > 0) ? currentPulsePos : targetPulse;
    int direction = (targetPulse > current) ? 1 : -1;
    int steps = abs(targetPulse - current) / stepSize;

    coverServo.attach(PIN_SERVO);

    if (steps == 0) {
        for (int i = 0; i < 25; i++) sendServoPulse(targetPulse);
    } else {
        for (int i = 0; i <= steps; i++) {
            int pos = current + (direction * i * stepSize);
            pos = constrain(pos, min(current, targetPulse), max(current, targetPulse));
            sendServoPulse(pos);
            if (i < steps) delay(stepDelayMs);
        }
        for (int i = 0; i < 10; i++) sendServoPulse(targetPulse);
    }

    currentPulsePos = targetPulse;
    coverServo.detach();
}

void moveServoSoft(int targetPulse) {
    int current = (currentPulsePos > 0) ? currentPulsePos : targetPulse;
    int totalDelta = targetPulse - current;

    if (abs(totalDelta) < 10) {
        coverServo.attach(PIN_SERVO);
        for (int i = 0; i < 25; i++) sendServoPulse(targetPulse);
        currentPulsePos = targetPulse;
        coverServo.detach();
        return;
    }

    int numSteps = 120 - (servoSpeed * 10);
    if (numSteps < 15) numSteps = 15;
    int baseStepDelay = 35 - (servoSpeed * 3);
    if (baseStepDelay < 3) baseStepDelay = 3;

    coverServo.attach(PIN_SERVO);

    for (int i = 0; i <= numSteps; i++) {
        float t = (float)i / (float)numSteps;
        float eased = (1.0f - cosf(t * M_PI)) / 2.0f;
        int pos = current + (int)(totalDelta * eased);
        pos = constrain(pos, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        sendServoPulse(pos);

        if (i < numSteps) {
            float speed_factor = sinf(t * M_PI);
            if (speed_factor < 0.15f) speed_factor = 0.15f;
            int stepDelay = (int)(baseStepDelay / speed_factor);
            stepDelay = constrain(stepDelay, 2, baseStepDelay * 4);
            delay(stepDelay);
        }
    }

    for (int i = 0; i < 10; i++) sendServoPulse(targetPulse);
    currentPulsePos = targetPulse;
    coverServo.detach();
}

void moveServo(int targetPulse) {
    if (softCloseEnabled) moveServoSoft(targetPulse);
    else moveServoDirect(targetPulse);
}

// ============================================================
// Cover-Funktionen
// ============================================================
void openCover() {
    coverState = COVER_MOVING;
    moveServo(servoOpenPulse);

    if (hallEnabled && isHallOpen()) {
        coverState = COVER_OPEN;
    } else if (!hallEnabled) {
        coverState = COVER_OPEN;
    } else {
        coverState = COVER_ERROR;
    }
}

void closeCover() {
    coverState = COVER_MOVING;
    moveServo(servoClosePulse);

    if (hallEnabled && isHallClosed()) {
        coverState = COVER_CLOSED;
    } else if (!hallEnabled) {
        coverState = COVER_CLOSED;
    } else {
        coverState = COVER_ERROR;
    }
}

// ============================================================
// Helligkeit (mit PWM-Mapping)
// ============================================================
void setBrightness(int value) {
    value = constrain(value, 0, 255);
    currentBrightness = value;

    int pwmValue;
    if (value == 0) {
        pwmValue = 0;
    } else {
        pwmValue = minPWM + ((long)(value - 1) * (255 - minPWM)) / 254;
    }

    ledcWrite(LEDC_EL_CH, pwmValue);
    calState = (value > 0) ? CAL_READY : CAL_OFF;
}

// ============================================================
// Alpaca JSON Helpers
// ============================================================
String alpacaResponse(uint32_t clientTxID, int errorNum, String errorMsg, String value) {
    serverTransactionID++;
    String json = "{";
    json += "\"ClientTransactionID\":" + String(clientTxID) + ",";
    json += "\"ServerTransactionID\":" + String(serverTransactionID) + ",";
    json += "\"ErrorNumber\":" + String(errorNum) + ",";
    json += "\"ErrorMessage\":\"" + errorMsg + "\"";
    if (value.length() > 0) {
        json += ",\"Value\":" + value;
    }
    json += "}";
    return json;
}

String alpacaOK(uint32_t clientTxID, String value) {
    return alpacaResponse(clientTxID, 0, "", value);
}

String alpacaError(uint32_t clientTxID, int errNum, String errMsg) {
    return alpacaResponse(clientTxID, errNum, errMsg, "");
}

uint32_t getClientTxID() {
    if (server.hasArg("ClientTransactionID"))
        return server.arg("ClientTransactionID").toInt();
    return 0;
}

uint32_t getClientID() {
    if (server.hasArg("ClientID"))
        return server.arg("ClientID").toInt();
    return 0;
}

void sendJSON(int code, String json) {
    server.send(code, "application/json", json);
}

// ============================================================
// Alpaca: Management API
// ============================================================
void handleApiVersions() {
    sendJSON(200, "{\"Value\":[1]}");
}

void handleDescription() {
    String json = "{\"Value\":{";
    json += "\"ServerName\":\"" + String(FIRMWARE_NAME) + "\",";
    json += "\"Manufacturer\":\"" + String(MANUFACTURER) + "\",";
    json += "\"ManufacturerVersion\":\"" + String(FIRMWARE_VERSION) + "\",";
    json += "\"Location\":\"\"";
    json += "}}";
    sendJSON(200, json);
}

void handleConfiguredDevices() {
    String json = "{\"Value\":[{";
    json += "\"DeviceName\":\"FlipFlat Panel\",";
    json += "\"DeviceType\":\"CoverCalibrator\",";
    json += "\"DeviceNumber\":0,";
    json += "\"UniqueID\":\"" + String(DEVICE_GUID) + "\"";
    json += "}]}";
    sendJSON(200, json);
}

// ============================================================
// Alpaca: Common ASCOM Device Methods
// ============================================================
void handleConnected() {
    if (server.method() == HTTP_GET) {
        sendJSON(200, alpacaOK(getClientTxID(), isConnected ? "true" : "false"));
    } else { // PUT
        if (server.hasArg("Connected")) {
            isConnected = (server.arg("Connected") == "true" ||
                           server.arg("Connected") == "True");
            if (isConnected) {
                // Zustand abfragen
                if (hallEnabled) updateCoverStateFromHall();
            }
        }
        sendJSON(200, alpacaOK(getClientTxID(), ""));
    }
}

void handleName() {
    sendJSON(200, alpacaOK(getClientTxID(), "\"FlipFlat Panel\""));
}

void handleDriverDescription() {
    sendJSON(200, alpacaOK(getClientTxID(),
        "\"FlipFlat Panel ESP32 - CoverCalibrator with EL-Foil, Heater and Hall Sensors\""));
}

void handleDriverInfo() {
    sendJSON(200, alpacaOK(getClientTxID(),
        "\"FlipFlat Panel ESP32 Alpaca Driver v" + String(FIRMWARE_VERSION) + "\""));
}

void handleDriverVersion() {
    sendJSON(200, alpacaOK(getClientTxID(), "\"" + String(FIRMWARE_VERSION) + "\""));
}

void handleInterfaceVersion() {
    sendJSON(200, alpacaOK(getClientTxID(), "2"));
}

void handleSupportedActions() {
    sendJSON(200, alpacaOK(getClientTxID(), "[]"));
}

void handleAction() {
    sendJSON(400, alpacaError(getClientTxID(), 0x400, "Action not implemented"));
}

void handleCommandBlind() {
    // Unterstütze direktes Senden von Seriellen Befehlen via Alpaca
    if (server.hasArg("Command")) {
        // Hier könnten wir den Befehl an processSerialCommand weiterleiten
    }
    sendJSON(200, alpacaOK(getClientTxID(), ""));
}

void handleCommandBool() {
    sendJSON(400, alpacaError(getClientTxID(), 0x400, "CommandBool not implemented"));
}

void handleCommandString() {
    sendJSON(400, alpacaError(getClientTxID(), 0x400, "CommandString not implemented"));
}

// ============================================================
// Alpaca: CoverCalibrator-Specific Methods
// ============================================================
void handleCoverState() {
    if (hallEnabled) updateCoverStateFromHall();
    sendJSON(200, alpacaOK(getClientTxID(), String((int)coverState)));
}

void handleCoverMoving() {
    sendJSON(200, alpacaOK(getClientTxID(),
        (coverState == COVER_MOVING) ? "true" : "false"));
}

void handleOpenCover() {
    if (!isConnected) {
        sendJSON(400, alpacaError(getClientTxID(), 0x400, "Not connected"));
        return;
    }
    openCover();
    sendJSON(200, alpacaOK(getClientTxID(), ""));
}

void handleCloseCover() {
    if (!isConnected) {
        sendJSON(400, alpacaError(getClientTxID(), 0x400, "Not connected"));
        return;
    }
    closeCover();
    sendJSON(200, alpacaOK(getClientTxID(), ""));
}

void handleHaltCover() {
    // Servo stoppt automatisch nach Detach
    coverServo.detach();
    if (coverState == COVER_MOVING) coverState = COVER_UNKNOWN;
    sendJSON(200, alpacaOK(getClientTxID(), ""));
}

void handleCalibratorState() {
    sendJSON(200, alpacaOK(getClientTxID(), String((int)calState)));
}

void handleCalibratorChanging() {
    sendJSON(200, alpacaOK(getClientTxID(), "false"));
}

void handleBrightness() {
    sendJSON(200, alpacaOK(getClientTxID(), String(currentBrightness)));
}

void handleMaxBrightness() {
    sendJSON(200, alpacaOK(getClientTxID(), "255"));
}

void handleCalibratorOn() {
    if (!isConnected) {
        sendJSON(400, alpacaError(getClientTxID(), 0x400, "Not connected"));
        return;
    }
    if (server.hasArg("Brightness")) {
        int b = server.arg("Brightness").toInt();
        setBrightness(b);
        sendJSON(200, alpacaOK(getClientTxID(), ""));
    } else {
        sendJSON(400, alpacaError(getClientTxID(), 0x401, "Brightness parameter required"));
    }
}

void handleCalibratorOff() {
    if (!isConnected) {
        sendJSON(400, alpacaError(getClientTxID(), 0x400, "Not connected"));
        return;
    }
    setBrightness(0);
    sendJSON(200, alpacaOK(getClientTxID(), ""));
}

// ============================================================
// Alpaca: Erweiterte Endpunkte (Temperatur, Heizung, Settings)
// ============================================================
void handleStatus() {
    // Custom endpoint: /api/v1/covercalibrator/0/status
    String json = "{";
    json += "\"coverState\":" + String((int)coverState) + ",";
    json += "\"brightness\":" + String(currentBrightness) + ",";
    json += "\"temperature\":" + String(temperature, 1) + ",";
    json += "\"humidity\":" + String(humidity, 1) + ",";
    json += "\"pressure\":" + String(pressure, 1) + ",";
    json += "\"heaterActive\":" + String(heaterActive ? "true" : "false") + ",";
    json += "\"heaterTarget\":" + String(heaterTarget, 1) + ",";
    json += "\"heaterEnabled\":" + String(heaterEnabled ? "true" : "false") + ",";
    json += "\"hallEnabled\":" + String(hallEnabled ? "true" : "false") + ",";
    json += "\"hallOpen\":" + String(isHallOpen() ? "true" : "false") + ",";
    json += "\"hallClosed\":" + String(isHallClosed() ? "true" : "false") + ",";
    json += "\"servoOpen\":" + String(servoOpenPulse) + ",";
    json += "\"servoClose\":" + String(servoClosePulse) + ",";
    json += "\"servoSpeed\":" + String(servoSpeed) + ",";
    json += "\"softClose\":" + String(softCloseEnabled ? "true" : "false") + ",";
    json += "\"minPWM\":" + String(minPWM) + ",";
    json += "\"firmware\":\"" + String(FIRMWARE_VERSION) + "\",";
    json += "\"wifiRSSI\":" + String(WiFi.RSSI()) + ",";
    json += "\"wifiIP\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\"";
    json += "}";
    sendJSON(200, json);
}

void handleSettings() {
    // PUT: Einstellungen ändern
    if (server.method() == HTTP_PUT || server.method() == HTTP_POST) {
        if (server.hasArg("servoOpen"))
            servoOpenPulse = constrain(server.arg("servoOpen").toInt(), SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        if (server.hasArg("servoClose"))
            servoClosePulse = constrain(server.arg("servoClose").toInt(), SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        if (server.hasArg("servoSpeed"))
            servoSpeed = constrain(server.arg("servoSpeed").toInt(), 1, 10);
        if (server.hasArg("softClose"))
            softCloseEnabled = (server.arg("softClose") == "true" || server.arg("softClose") == "1");
        if (server.hasArg("minPWM"))
            minPWM = constrain(server.arg("minPWM").toInt(), 0, 254);
        if (server.hasArg("heaterTarget"))
            heaterTarget = server.arg("heaterTarget").toFloat();
        if (server.hasArg("heaterHysteresis"))
            heaterHysteresis = server.arg("heaterHysteresis").toFloat();
        if (server.hasArg("heaterEnabled"))
            heaterEnabled = (server.arg("heaterEnabled") == "true" || server.arg("heaterEnabled") == "1");
        if (server.hasArg("hallEnabled"))
            hallEnabled = (server.arg("hallEnabled") == "true" || server.arg("hallEnabled") == "1");
        if (server.hasArg("save") && server.arg("save") == "true")
            saveSettings();

        sendJSON(200, "{\"result\":\"ok\"}");
    } else {
        // GET: Einstellungen lesen
        handleStatus();
    }
}

void handleWiFiConfig() {
    if (server.method() == HTTP_PUT || server.method() == HTTP_POST) {
        if (server.hasArg("ssid")) wifiSSID = server.arg("ssid");
        if (server.hasArg("pass")) wifiPass = server.arg("pass");
        if (server.hasArg("apSSID")) apSSID = server.arg("apSSID");
        if (server.hasArg("apPass")) apPass = server.arg("apPass");
        saveSettings();
        sendJSON(200, "{\"result\":\"ok\",\"message\":\"WiFi gespeichert. Neustart nötig.\"}");
    } else {
        String json = "{";
        json += "\"ssid\":\"" + wifiSSID + "\",";
        json += "\"apSSID\":\"" + apSSID + "\",";
        json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
        json += "\"wifiIP\":\"" + WiFi.localIP().toString() + "\",";
        json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\"";
        json += "}";
        sendJSON(200, json);
    }
}

// ============================================================
// Alpaca: Konfigurations-Webseite (HTML)
// ============================================================
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlipFlat Panel - Setup</title>
<style>
body{font-family:Arial;background:#1a1a2e;color:#e0e0e0;max-width:600px;margin:0 auto;padding:16px}
h1{color:#5B9BD5;font-size:1.4em}h2{color:#6BCB77;font-size:1.1em;margin-top:24px}
.card{background:#2a2a3c;border-radius:8px;padding:14px;margin:8px 0}
label{display:block;margin:6px 0 2px;font-size:.9em;color:#aaa}
input,select{width:100%;padding:6px;border:1px solid #444;border-radius:4px;background:#1a1a2e;color:#e0e0e0;box-sizing:border-box}
input[type=checkbox]{width:auto}
button{background:#5B9BD5;color:white;border:none;padding:10px 20px;border-radius:6px;cursor:pointer;margin:4px;font-size:.95em}
button:hover{background:#4a8bc4}
.btn-green{background:#6BCB77}.btn-red{background:#FF6B6B}
.status{font-size:.85em;color:#888}
#statusBox{font-family:monospace;font-size:.8em;white-space:pre;overflow-x:auto}
</style></head><body>
<h1>&#128301; FlipFlat Panel ESP32</h1>
<div class="card" id="statusBox">Lade Status...</div>

<h2>WiFi-Einstellungen</h2>
<div class="card">
<label>WLAN SSID:</label><input id="ssid" placeholder="Dein WLAN-Name">
<label>WLAN Passwort:</label><input id="pass" type="password">
<label>AP SSID:</label><input id="apSSID" value="FlipFlatPanel">
<label>AP Passwort:</label><input id="apPass" value="flatfield">
<button onclick="saveWifi()">WiFi speichern & Neustart</button>
</div>

<h2>Servo-Einstellungen</h2>
<div class="card">
<label>Open-Position (500-2500 &micro;s):</label><input id="sOpen" type="number" min="500" max="2500">
<label>Close-Position (500-2500 &micro;s):</label><input id="sClose" type="number" min="500" max="2500">
<label>Geschwindigkeit (1-10):</label><input id="sSpeed" type="number" min="1" max="10">
<label><input type="checkbox" id="sSoft"> SoftClose</label>
<label>Min PWM EL-Folie (0-254):</label><input id="sMinPWM" type="number" min="0" max="254">
<button onclick="applySettings()">&#x2713; &Uuml;bernehmen</button>
<button class="btn-green" onclick="saveAll()">Speichern (NVS)</button>
<button onclick="testOpen()">Test Open</button>
<button onclick="testClose()">Test Close</button>
</div>

<h2>Heizung</h2>
<div class="card">
<label><input type="checkbox" id="hEnable"> Heizung aktiv</label>
<label>Zieltemperatur (&deg;C):</label><input id="hTarget" type="number" step="0.5">
<label>Hysterese (&deg;C):</label><input id="hHyst" type="number" step="0.5">
<button onclick="applySettings()">&#x2713; &Uuml;bernehmen</button>
</div>

<h2>Hall-Sensoren</h2>
<div class="card">
<label><input type="checkbox" id="hallEn"> Hall-Sensoren aktiv</label>
<button onclick="applySettings()">&#x2713; &Uuml;bernehmen</button>
</div>

<div class="status" style="margin-top:16px;text-align:center">
<a href="/update" style="color:#FFB347;text-decoration:none;font-size:.95em">&#128640; Firmware Update (OTA)</a><br><br>
FlipFlat Panel ESP32 Firmware v)rawliteral"
    + String(FIRMWARE_VERSION) + R"rawliteral(<br>
Credits: Moritz Mayer / Dark Matters Discord
</div>

<script>
const BASE = '';
function fetchStatus() {
  fetch(BASE+'/api/v1/covercalibrator/0/status')
    .then(r=>r.json()).then(d=>{
      document.getElementById('statusBox').textContent =
        `Cover: ${['NotPresent','Closed','Moving','Open','Unknown','Error'][d.coverState]}\n`+
        `Helligkeit: ${d.brightness}/255\n`+
        `Temperatur: ${d.temperature}°C  Feuchte: ${d.humidity}%  Druck: ${d.pressure} hPa\n`+
        `Heizung: ${d.heaterActive?'EIN':'AUS'} (Ziel: ${d.heaterTarget}°C)\n`+
        `Hall: Open=${d.hallOpen} Close=${d.hallClosed}\n`+
        `WiFi RSSI: ${d.wifiRSSI} dBm`;
      document.getElementById('sOpen').value=d.servoOpen;
      document.getElementById('sClose').value=d.servoClose;
      document.getElementById('sSpeed').value=d.servoSpeed;
      document.getElementById('sSoft').checked=d.softClose;
      document.getElementById('sMinPWM').value=d.minPWM;
      document.getElementById('hEnable').checked=d.heaterEnabled;
      document.getElementById('hTarget').value=d.heaterTarget;
      document.getElementById('hHyst').value=2.0;
      document.getElementById('hallEn').checked=d.hallEnabled;
    }).catch(e=>{document.getElementById('statusBox').textContent='Fehler: '+e});
}

function applySettings(){
  const params=new URLSearchParams();
  params.set('servoOpen',document.getElementById('sOpen').value);
  params.set('servoClose',document.getElementById('sClose').value);
  params.set('servoSpeed',document.getElementById('sSpeed').value);
  params.set('softClose',document.getElementById('sSoft').checked?'true':'false');
  params.set('minPWM',document.getElementById('sMinPWM').value);
  params.set('heaterEnabled',document.getElementById('hEnable').checked?'true':'false');
  params.set('heaterTarget',document.getElementById('hTarget').value);
  params.set('hallEnabled',document.getElementById('hallEn').checked?'true':'false');
  fetch(BASE+'/api/v1/covercalibrator/0/settings',{method:'PUT',body:params})
    .then(()=>fetchStatus());
}

function saveAll(){
  const params=new URLSearchParams();
  params.set('servoOpen',document.getElementById('sOpen').value);
  params.set('servoClose',document.getElementById('sClose').value);
  params.set('servoSpeed',document.getElementById('sSpeed').value);
  params.set('softClose',document.getElementById('sSoft').checked?'true':'false');
  params.set('minPWM',document.getElementById('sMinPWM').value);
  params.set('heaterEnabled',document.getElementById('hEnable').checked?'true':'false');
  params.set('heaterTarget',document.getElementById('hTarget').value);
  params.set('hallEnabled',document.getElementById('hallEn').checked?'true':'false');
  params.set('save','true');
  fetch(BASE+'/api/v1/covercalibrator/0/settings',{method:'PUT',body:params})
    .then(()=>{alert('Gespeichert!');fetchStatus()});
}

function saveWifi(){
  const params=new URLSearchParams();
  params.set('ssid',document.getElementById('ssid').value);
  params.set('pass',document.getElementById('pass').value);
  params.set('apSSID',document.getElementById('apSSID').value);
  params.set('apPass',document.getElementById('apPass').value);
  fetch(BASE+'/api/v1/covercalibrator/0/wifi',{method:'PUT',body:params})
    .then(()=>{alert('WiFi gespeichert! ESP32 startet neu...');setTimeout(()=>ESP.restart(),2000)});
}

function testOpen(){
  fetch(BASE+'/api/v1/covercalibrator/0/opencover',{method:'PUT',
    body:new URLSearchParams({ClientID:1,ClientTransactionID:1})})
    .then(()=>setTimeout(fetchStatus,3000));
}

function testClose(){
  fetch(BASE+'/api/v1/covercalibrator/0/closecover',{method:'PUT',
    body:new URLSearchParams({ClientID:1,ClientTransactionID:1})})
    .then(()=>setTimeout(fetchStatus,3000));
}

fetchStatus();
setInterval(fetchStatus, 5000);
</script></body></html>)rawliteral";
    server.send(200, "text/html", html);
}

// ============================================================
// OTA: Web-Upload Seite
// ============================================================
void handleOTAPage() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FlipFlat Panel - Firmware Update</title>
<style>
body{font-family:Arial;background:#1a1a2e;color:#e0e0e0;max-width:500px;margin:0 auto;padding:16px}
h1{color:#5B9BD5;font-size:1.3em}
.card{background:#2a2a3c;border-radius:8px;padding:16px;margin:12px 0}
.drop{border:2px dashed #5B9BD5;border-radius:8px;padding:40px 20px;text-align:center;cursor:pointer;transition:all .3s}
.drop:hover,.drop.dragover{border-color:#6BCB77;background:#1e3a1e}
.drop input{display:none}
button{background:#5B9BD5;color:white;border:none;padding:12px 24px;border-radius:6px;cursor:pointer;font-size:1em;width:100%;margin-top:12px}
button:hover{background:#4a8bc4}
button:disabled{background:#555;cursor:not-allowed}
.progress{width:100%;height:24px;background:#1a1a2e;border-radius:12px;overflow:hidden;margin-top:12px;display:none}
.progress-bar{height:100%;background:linear-gradient(90deg,#5B9BD5,#6BCB77);border-radius:12px;transition:width .3s;text-align:center;line-height:24px;font-size:12px;font-weight:bold}
.status{margin-top:12px;padding:10px;border-radius:6px;display:none;text-align:center}
.success{background:#1e3a1e;color:#6BCB77;display:block}
.error{background:#3a1e1e;color:#FF6B6B;display:block}
.warn{color:#FFB347;font-size:.85em;margin-top:12px}
</style></head><body>
<h1>&#128640; Firmware Update (OTA)</h1>
<div class="card">
<p style="color:#aaa;font-size:.9em">Aktuelle Version: <strong style="color:#6BCB77">)rawliteral"
    + String(FIRMWARE_VERSION) + R"rawliteral(</strong></p>

<div class="drop" id="dropZone" onclick="document.getElementById('fileInput').click()">
<div style="font-size:2em;margin-bottom:8px">&#128196;</div>
<div>Firmware-Datei (.bin) hierher ziehen<br>oder klicken zum Auswählen</div>
<input type="file" id="fileInput" accept=".bin">
</div>
<div id="fileName" style="color:#aaa;font-size:.85em;margin-top:6px"></div>

<button id="uploadBtn" onclick="uploadFirmware()" disabled>Firmware hochladen</button>

<div class="progress" id="progressWrap">
<div class="progress-bar" id="progressBar" style="width:0%">0%</div>
</div>

<div class="status" id="statusMsg"></div>

<div class="warn">
&#9888; Während des Updates nicht die Verbindung trennen!<br>
Der ESP32 startet nach dem Update automatisch neu.
</div>
</div>

<p style="text-align:center"><a href="/" style="color:#5B9BD5">&#8592; Zurück zur Hauptseite</a></p>

<script>
const dropZone=document.getElementById('dropZone');
const fileInput=document.getElementById('fileInput');
const uploadBtn=document.getElementById('uploadBtn');
let selectedFile=null;

dropZone.addEventListener('dragover',e=>{e.preventDefault();dropZone.classList.add('dragover')});
dropZone.addEventListener('dragleave',()=>dropZone.classList.remove('dragover'));
dropZone.addEventListener('drop',e=>{e.preventDefault();dropZone.classList.remove('dragover');
  if(e.dataTransfer.files.length)selectFile(e.dataTransfer.files[0])});
fileInput.addEventListener('change',()=>{if(fileInput.files.length)selectFile(fileInput.files[0])});

function selectFile(f){
  if(!f.name.endsWith('.bin')){alert('Nur .bin Dateien!');return}
  selectedFile=f;
  document.getElementById('fileName').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
  uploadBtn.disabled=false;
}

function uploadFirmware(){
  if(!selectedFile)return;
  if(!confirm('Firmware-Update starten mit: '+selectedFile.name+'?'))return;
  uploadBtn.disabled=true;
  const progress=document.getElementById('progressWrap');
  const bar=document.getElementById('progressBar');
  const status=document.getElementById('statusMsg');
  progress.style.display='block';
  status.style.display='none';

  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  xhr.upload.onprogress=e=>{if(e.lengthComputable){
    const pct=Math.round(e.loaded/e.total*100);
    bar.style.width=pct+'%';bar.textContent=pct+'%'}};
  xhr.onload=()=>{
    if(xhr.status===200){
      status.className='status success';
      status.textContent='\\u2713 Update erfolgreich! ESP32 startet neu...';
      status.style.display='block';
      setTimeout(()=>location.href='/',10000);
    }else{
      status.className='status error';
      status.textContent='\\u2717 Update fehlgeschlagen: '+xhr.responseText;
      status.style.display='block';
      uploadBtn.disabled=false;
    }
  };
  xhr.onerror=()=>{
    status.className='status error';
    status.textContent='\\u2717 Verbindungsfehler';
    status.style.display='block';
    uploadBtn.disabled=false;
  };

  const formData=new FormData();
  formData.append('firmware',selectedFile);
  xhr.send(formData);
}
</script></body></html>)rawliteral";
    server.send(200, "text/html", html);
}

void handleOTAUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Web-Update gestartet: %s\n", upload.filename.c_str());
        // Genug Platz? Prüfe Partition
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        // Daten schreiben
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("[OTA] Update erfolgreich! %u Bytes. Neustart...\n", upload.totalSize);
        } else {
            Update.printError(Serial);
        }
    }
}

void handleOTAResult() {
    if (Update.hasError()) {
        server.send(500, "text/plain", "Update fehlgeschlagen: " + String(Update.errorString()));
    } else {
        server.send(200, "text/plain", "OK");
        delay(1000);
        ESP.restart();
    }
}

// ============================================================
// OTA: ArduinoOTA Setup (IDE-Upload über WiFi)
// ============================================================
void setupArduinoOTA() {
    ArduinoOTA.setHostname("flipflatpanel");

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Firmware" : "Filesystem";
        Serial.println("[OTA] Arduino IDE Update gestartet: " + type);
        // Servo und Heizung sicherheitshalber abschalten
        coverServo.detach();
        ledcWrite(LEDC_HEATER_CH, 0);
        ledcWrite(LEDC_EL_CH, 0);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Update abgeschlossen. Neustart...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Fortschritt: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Fehler [%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth fehlgeschlagen");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin fehlgeschlagen");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Verbindung fehlgeschlagen");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Empfang fehlgeschlagen");
        else if (error == OTA_END_ERROR) Serial.println("End fehlgeschlagen");
    });

    ArduinoOTA.begin();
    Serial.println("[OTA] ArduinoOTA bereit (Hostname: flipflatpanel)");
}

// ============================================================
// Alpaca: Route-Setup
// ============================================================
void setupAlpacaRoutes() {
    // Root / Konfigurations-Seite
    server.on("/", handleRoot);
    server.on("/setup", handleRoot);

    // Management API
    server.on("/management/apiversions", HTTP_GET, handleApiVersions);
    server.on("/management/v1/description", HTTP_GET, handleDescription);
    server.on("/management/v1/configureddevices", HTTP_GET, handleConfiguredDevices);

    // Common ASCOM Methods
    String base = "/api/v1/covercalibrator/0";
    server.on(base + "/connected", handleConnected);
    server.on(base + "/name", HTTP_GET, handleName);
    server.on(base + "/description", HTTP_GET, handleDriverDescription);
    server.on(base + "/driverinfo", HTTP_GET, handleDriverInfo);
    server.on(base + "/driverversion", HTTP_GET, handleDriverVersion);
    server.on(base + "/interfaceversion", HTTP_GET, handleInterfaceVersion);
    server.on(base + "/supportedactions", HTTP_GET, handleSupportedActions);
    server.on(base + "/action", HTTP_PUT, handleAction);
    server.on(base + "/commandblind", HTTP_PUT, handleCommandBlind);
    server.on(base + "/commandbool", HTTP_PUT, handleCommandBool);
    server.on(base + "/commandstring", HTTP_PUT, handleCommandString);

    // CoverCalibrator Methods
    server.on(base + "/coverstate", HTTP_GET, handleCoverState);
    server.on(base + "/covermoving", HTTP_GET, handleCoverMoving);
    server.on(base + "/opencover", HTTP_PUT, handleOpenCover);
    server.on(base + "/closecover", HTTP_PUT, handleCloseCover);
    server.on(base + "/haltcover", HTTP_PUT, handleHaltCover);
    server.on(base + "/calibratorstate", HTTP_GET, handleCalibratorState);
    server.on(base + "/calibratorchanging", HTTP_GET, handleCalibratorChanging);
    server.on(base + "/brightness", HTTP_GET, handleBrightness);
    server.on(base + "/maxbrightness", HTTP_GET, handleMaxBrightness);
    server.on(base + "/calibratoron", HTTP_PUT, handleCalibratorOn);
    server.on(base + "/calibratoroff", HTTP_PUT, handleCalibratorOff);

    // Erweiterte Endpunkte
    server.on(base + "/status", HTTP_GET, handleStatus);
    server.on(base + "/settings", handleSettings);
    server.on(base + "/wifi", handleWiFiConfig);

    // OTA Web-Update
    server.on("/update", HTTP_GET, handleOTAPage);
    server.on("/update", HTTP_POST, handleOTAResult, handleOTAUpload);

    server.begin();
    Serial.printf("[Alpaca] HTTP-Server gestartet auf Port %d\n", ALPACA_PORT);
}

// ============================================================
// Alpaca Discovery (UDP Broadcast)
// ============================================================
void setupDiscovery() {
    udp.begin(DISCOVERY_PORT);
    Serial.printf("[Alpaca] Discovery lauscht auf UDP Port %d\n", DISCOVERY_PORT);
}

void handleDiscovery() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        char buffer[64];
        int len = udp.read(buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';

        // Alpaca Discovery Request: "alpacadiscovery1"
        if (strstr(buffer, "alpacadiscovery") != nullptr) {
            String response = "{\"AlpacaPort\":" + String(ALPACA_PORT) + "}";
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.print(response);
            udp.endPacket();
            Serial.printf("[Alpaca] Discovery von %s -> Port %d\n",
                udp.remoteIP().toString().c_str(), ALPACA_PORT);
        }
    }
}

// ============================================================
// Serielles Protokoll (kompatibel zu Arduino v3.0)
// ============================================================
void processSerialCommand(const char* cmd) {
    // v2.0 Befehle
    if (strcmp(cmd, "COMMAND:PING") == 0) {
        Serial.printf("RESULT:PING:OK:%s\n", DEVICE_GUID);
    }
    else if (strcmp(cmd, "COMMAND:INFO") == 0) {
        Serial.printf("RESULT:INFO:%s Firmware v%s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
    }
    else if (strcmp(cmd, "COMMAND:OPEN") == 0) {
        openCover();
        Serial.println(coverState == COVER_OPEN ? "RESULT:OPEN:offen" : "RESULT:OPEN:error");
    }
    else if (strcmp(cmd, "COMMAND:CLOSE") == 0) {
        closeCover();
        Serial.println(coverState == COVER_CLOSED ? "RESULT:CLOSE:geschlossen" : "RESULT:CLOSE:error");
    }
    else if (strcmp(cmd, "COMMAND:POSITION") == 0) {
        const char* states[] = {"notpresent","geschlossen","moving","offen","unknown","error"};
        Serial.printf("RESULT:POSITION:%s\n", states[(int)coverState]);
    }
    else if (strcmp(cmd, "COMMAND:BRIGHTNESS") == 0) {
        Serial.printf("RESULT:BRIGHTNESS:%d\n", currentBrightness);
    }
    else if (strcmp(cmd, "COMMAND:MAXBRIGHTNESS") == 0) {
        Serial.println("RESULT:MAXBRIGHTNESS:255");
    }
    else if (strncmp(cmd, "COMMAND:SETBRIGHTNESS:", 22) == 0) {
        setBrightness(atoi(cmd + 22));
        Serial.printf("RESULT:SETBRIGHTNESS:%d\n", currentBrightness);
    }
    // v3.0 Servo-Befehle
    else if (strncmp(cmd, "COMMAND:SETOPEN:", 16) == 0) {
        servoOpenPulse = constrain(atoi(cmd + 16), SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        Serial.printf("RESULT:SETOPEN:%d\n", servoOpenPulse);
    }
    else if (strcmp(cmd, "COMMAND:GETOPEN") == 0) {
        Serial.printf("RESULT:GETOPEN:%d\n", servoOpenPulse);
    }
    else if (strncmp(cmd, "COMMAND:SETCLOSE:", 17) == 0) {
        servoClosePulse = constrain(atoi(cmd + 17), SERVO_PULSE_MIN, SERVO_PULSE_MAX);
        Serial.printf("RESULT:SETCLOSE:%d\n", servoClosePulse);
    }
    else if (strcmp(cmd, "COMMAND:GETCLOSE") == 0) {
        Serial.printf("RESULT:GETCLOSE:%d\n", servoClosePulse);
    }
    else if (strncmp(cmd, "COMMAND:SETSPEED:", 17) == 0) {
        servoSpeed = constrain(atoi(cmd + 17), 1, 10);
        Serial.printf("RESULT:SETSPEED:%d\n", servoSpeed);
    }
    else if (strcmp(cmd, "COMMAND:GETSPEED") == 0) {
        Serial.printf("RESULT:GETSPEED:%d\n", servoSpeed);
    }
    else if (strncmp(cmd, "COMMAND:SETSOFTCLOSE:", 21) == 0) {
        softCloseEnabled = atoi(cmd + 21) != 0;
        Serial.printf("RESULT:SETSOFTCLOSE:%d\n", softCloseEnabled ? 1 : 0);
    }
    else if (strcmp(cmd, "COMMAND:GETSOFTCLOSE") == 0) {
        Serial.printf("RESULT:GETSOFTCLOSE:%d\n", softCloseEnabled ? 1 : 0);
    }
    else if (strncmp(cmd, "COMMAND:SETMINPWM:", 18) == 0) {
        minPWM = constrain(atoi(cmd + 18), 0, 254);
        Serial.printf("RESULT:SETMINPWM:%d\n", minPWM);
    }
    else if (strcmp(cmd, "COMMAND:GETMINPWM") == 0) {
        Serial.printf("RESULT:GETMINPWM:%d\n", minPWM);
    }
    // Neue Befehle: Temperatur & Heizung
    else if (strcmp(cmd, "COMMAND:TEMPERATURE") == 0) {
        Serial.printf("RESULT:TEMPERATURE:%.1f\n", temperature);
    }
    else if (strcmp(cmd, "COMMAND:HUMIDITY") == 0) {
        Serial.printf("RESULT:HUMIDITY:%.1f\n", humidity);
    }
    else if (strcmp(cmd, "COMMAND:PRESSURE") == 0) {
        Serial.printf("RESULT:PRESSURE:%.1f\n", pressure);
    }
    else if (strncmp(cmd, "COMMAND:SETHEATER:", 18) == 0) {
        heaterEnabled = atoi(cmd + 18) != 0;
        Serial.printf("RESULT:SETHEATER:%d\n", heaterEnabled ? 1 : 0);
    }
    else if (strcmp(cmd, "COMMAND:GETHEATER") == 0) {
        Serial.printf("RESULT:GETHEATER:%d:%.1f:%.1f:%d\n",
            heaterEnabled ? 1 : 0, heaterTarget, temperature, heaterActive ? 1 : 0);
    }
    else if (strncmp(cmd, "COMMAND:SETHEATTARGET:", 21) == 0) {
        heaterTarget = atof(cmd + 21);
        Serial.printf("RESULT:SETHEATTARGET:%.1f\n", heaterTarget);
    }
    // WiFi-Befehle
    else if (strcmp(cmd, "COMMAND:WIFIIP") == 0) {
        Serial.printf("RESULT:WIFIIP:%s\n", WiFi.localIP().toString().c_str());
    }
    else if (strncmp(cmd, "COMMAND:SETWIFI:", 16) == 0) {
        // Format: COMMAND:SETWIFI:ssid:password
        String s = String(cmd + 16);
        int sep = s.indexOf(':');
        if (sep > 0) {
            wifiSSID = s.substring(0, sep);
            wifiPass = s.substring(sep + 1);
            saveSettings();
            Serial.printf("RESULT:SETWIFI:%s\n", wifiSSID.c_str());
        } else {
            Serial.println("ERROR:INVALID_WIFI_FORMAT");
        }
    }
    else if (strcmp(cmd, "COMMAND:SAVE") == 0) {
        saveSettings();
        Serial.println("RESULT:SAVE:OK");
    }
    else if (strcmp(cmd, "COMMAND:RESTART") == 0) {
        Serial.println("RESULT:RESTART:OK");
        delay(500);
        ESP.restart();
    }
    else {
        Serial.println("ERROR:INVALID_COMMAND");
    }
}

void processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (bufferPos > 0) {
                inputBuffer[bufferPos] = '\0';
                processSerialCommand(inputBuffer);
                bufferPos = 0;
            }
        } else if (bufferPos < INPUT_BUFFER_SIZE - 1) {
            inputBuffer[bufferPos++] = c;
        } else {
            bufferPos = 0;
        }
    }
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println("\n============================================");
    Serial.printf("  %s v%s\n", FIRMWARE_NAME, FIRMWARE_VERSION);
    Serial.println("  Credits: Moritz Mayer / Dark Matters Discord");
    Serial.println("============================================\n");

    // Einstellungen laden
    loadSettings();

    // Pins initialisieren
    ledcSetup(LEDC_EL_CH, LEDC_EL_FREQ, LEDC_EL_BITS);
    ledcAttachPin(PIN_EL_FOIL, LEDC_EL_CH);
    ledcWrite(LEDC_EL_CH, 0);

    ledcSetup(LEDC_HEATER_CH, LEDC_HEATER_FREQ, LEDC_HEATER_BITS);
    ledcAttachPin(PIN_HEATER, LEDC_HEATER_CH);
    ledcWrite(LEDC_HEATER_CH, 0);

    // Hall-Sensoren (Input-Only Pins, externer Pull-Up nötig)
    pinMode(PIN_HALL_OPEN, INPUT);
    pinMode(PIN_HALL_CLOSE, INPUT);

    // BME280
    setupBME280();

    // WiFi
    setupWiFi();

    // Alpaca
    setupAlpacaRoutes();
    setupDiscovery();

    // OTA
    setupArduinoOTA();

    // Initialer Status
    if (hallEnabled) updateCoverStateFromHall();

    Serial.println("\n[Ready] FlipFlat Panel bereit.");
    if (wifiConnected)
        Serial.printf("[Ready] Alpaca: http://%s:%d\n", WiFi.localIP().toString().c_str(), ALPACA_PORT);
    Serial.printf("[Ready] AP: http://%s:%d\n", WiFi.softAPIP().toString().c_str(), ALPACA_PORT);
    Serial.printf("[Ready] Web-OTA: http://<IP>:%d/update\n", ALPACA_PORT);
    Serial.println("[Ready] Arduino IDE OTA: flipflatpanel");
    Serial.println("[Ready] Serial: 57600 Baud\n");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
    server.handleClient();
    ArduinoOTA.handle();
    handleDiscovery();
    processSerial();
    readBME280();
    updateHeater();

    if (hallEnabled) updateCoverStateFromHall();

    yield(); // WDT
}
