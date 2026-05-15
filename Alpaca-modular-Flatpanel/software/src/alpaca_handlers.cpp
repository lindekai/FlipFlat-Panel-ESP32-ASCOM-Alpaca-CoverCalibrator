#include "alpaca_handlers.h"
#include "flatpanel_control.h"
#include "debug_monitor.h"

#include <ArduinoJson.h>
#include <WiFi.h>

using namespace FlatPanel;

static bool g_connected = true;
static uint32_t g_serverTransactionID = 1;
static uint32_t nextTransactionId() { return g_serverTransactionID++; }
static constexpr int ALPACA_ERR_NOT_IMPLEMENTED = 1024;
static constexpr int ALPACA_ERR_INVALID_VALUE = 1025;

static void logRequest(AsyncWebServerRequest* request) {
    Monitor::print("[ALPACA] ");
    Monitor::print(request->methodToString());
    Monitor::print(" ");
    Monitor::println(request->url());
}

static String getParamValue(AsyncWebServerRequest* request, const char* name, const String& fallback = "") {
    if (request->hasParam(name, true)) return request->getParam(name, true)->value();
    if (request->hasParam(name)) return request->getParam(name)->value();
    return fallback;
}

static bool parseBoolParam(const String& value, bool fallback = false) {
    String normalized = value;
    normalized.trim();
    normalized.toLowerCase();
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") return true;
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") return false;
    return fallback;
}

static void sendAlpacaResponseRaw(AsyncWebServerRequest* request, DynamicJsonDocument& doc) {
    doc["ClientTransactionID"] = 0;
    doc["ServerTransactionID"] = nextTransactionId();
    doc["ErrorNumber"] = 0;
    doc["ErrorMessage"] = "";
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

static void sendAlpacaError(AsyncWebServerRequest* request, int errorNumber, const String& errorMessage) {
    DynamicJsonDocument doc(256);
    doc["ClientTransactionID"] = 0;
    doc["ServerTransactionID"] = nextTransactionId();
    doc["ErrorNumber"] = errorNumber;
    doc["ErrorMessage"] = errorMessage;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

static void sendEnvironmentJson(AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1536);
    const wifi_mode_t wifiMode = WiFi.getMode();
    const bool networkConnected = WiFi.status() == WL_CONNECTED;
    const bool accessPointActive = wifiMode == WIFI_AP || wifiMode == WIFI_AP_STA;
    doc["networkConnected"] = networkConnected;
    doc["networkSsid"] = networkConnected ? WiFi.SSID() : "";
    doc["networkRssi"] = networkConnected ? WiFi.RSSI() : 0;
    doc["networkIp"] = networkConnected ? WiFi.localIP().toString() : "";
    doc["accessPointActive"] = accessPointActive;
    doc["accessPointIp"] = accessPointActive ? WiFi.softAPIP().toString() : "";
    doc["wifiMode"] = static_cast<int>(wifiMode);
    doc["alpacaHttpOk"] = true;
    doc["otaReady"] = true;
    doc["temperatureC"] = getTemperatureC();
    doc["humidityPct"] = getHumidityPercent();
    doc["pressureHpa"] = getPressureHpa();
    doc["dewPointC"] = getDewPointC();
    doc["heaterOn"] = isHeaterOn();
    doc["heaterEnabled"] = isHeaterEnabled();
    doc["heaterManualMode"] = isHeaterManualMode();
    doc["bmeAvailable"] = bmeAvailable();
    doc["brightness"] = getBrightness();
    doc["servoPulseUs"] = getServoPulseUs();
    doc["servoMinUs"] = SERVO_MIN_US;
    doc["servoMaxUs"] = SERVO_MAX_US;
    doc["servoOpenUs"] = getCalibrationOpenServoUs();
    doc["servoCloseUs"] = getCalibrationClosedServoUs();
    doc["servoSmoothEnabled"] = isServoSmoothEnabled();
    doc["servoMaxSpeedUsPerSec"] = getServoMaxSpeedUsPerSec();
    doc["servoSpeedMinUsPerSec"] = SERVO_SPEED_MIN_US_PER_SEC;
    doc["servoSpeedMaxUsPerSec"] = SERVO_SPEED_MAX_US_PER_SEC;
    doc["coverState"] = static_cast<int>(getCoverState());
    doc["coverStateText"] = coverStateString();
    doc["rawPosition"] = getRawPosition();
    doc["filteredPosition"] = getFilteredPosition();
    doc["mappedPosition"] = getMappedPosition();
    doc["calibrationOpenRaw"] = getCalibrationOpenRaw();
    doc["calibrationClosedRaw"] = getCalibrationClosedRaw();
    doc["calibrationOpenServoUs"] = getCalibrationOpenServoUs();
    doc["calibrationClosedServoUs"] = getCalibrationClosedServoUs();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>FlipFlatpanel</title>
<style>
body{font-family:Arial,sans-serif;margin:20px;background:#111;color:#eee}.card{background:#1d1d1d;padding:16px;border-radius:12px;margin-bottom:16px}
button{padding:10px 16px;margin:4px;border:none;border-radius:8px;cursor:pointer}input[type=range]{width:100%}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}.value{font-size:1.2rem;font-weight:bold}
a{color:#9cf}
</style></head><body>
<h1>FlipFlatpanel / Alpaca</h1>
<p><a href="/setup">Setup öffnen</a></p>
<div class="card"><h2>Deckel</h2><button onclick="openCover()">Öffnen</button><button onclick="closeCover()">Schließen</button><button onclick="haltCover()">Stopp</button><p>Status: <span id="coverState">-</span></p></div>
<div class="card"><h2>Flatpanel</h2><button onclick="panelOff()">Aus</button><button onclick="panelOn()">Ein</button><p>Helligkeit: <span id="brightnessLabel">0</span></p><input id="brightness" type="range" min="0" max="255" value="0" oninput="brightnessChanged(this.value)"></div>
<div class="card"><h2>Heizung</h2><button onclick="heaterEnable(true)">Automatik EIN</button><button onclick="heaterEnable(false)">Automatik AUS</button><p>Heater: <span id="heaterState">-</span></p><p>Automatik: <span id="heaterEnabled">-</span></p></div>
<div class="card"><h2>BME280</h2><div class="grid"><div><div>Temperatur</div><div class="value" id="temp">-</div></div><div><div>Luftfeuchte</div><div class="value" id="hum">-</div></div><div><div>Druck</div><div class="value" id="pres">-</div></div><div><div>Taupunkt</div><div class="value" id="dew">-</div></div></div></div>
<div class="card"><h2>Positionssensor / Kalibrierung</h2><button onclick="calibrateOpen()">Aktuelle Position = Offen</button><button onclick="calibrateClosed()">Aktuelle Position = Geschlossen</button><button onclick="resetCalibration()">Kalibrierung zurücksetzen</button><div class="grid" style="margin-top:12px;"><div><div>RAW ADC</div><div class="value" id="rawPos">-</div></div><div><div>Gefiltert</div><div class="value" id="filteredPos">-</div></div><div><div>Mapped</div><div class="value" id="mappedPos">-</div></div><div><div>Open RAW</div><div class="value" id="openRaw">-</div></div><div><div>Closed RAW</div><div class="value" id="closedRaw">-</div></div></div></div>
<script>
async function post(url, body=''){await fetch(url,{method:'PUT',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});refresh();}
function openCover(){post('/api/v1/covercalibrator/0/opencover')} function closeCover(){post('/api/v1/covercalibrator/0/closecover')} function haltCover(){post('/api/v1/covercalibrator/0/haltcover')}
function panelOff(){post('/api/v1/covercalibrator/0/calibratoroff')} function panelOn(){const v=document.getElementById('brightness').value;post('/api/v1/covercalibrator/0/calibratoron','Brightness='+encodeURIComponent(v))}
function brightnessChanged(v){document.getElementById('brightnessLabel').textContent=v;post('/api/v1/covercalibrator/0/brightness','Brightness='+encodeURIComponent(v))}
function heaterEnable(state){post('/api/v1/flatpanel/heaterenabled','Enabled='+(state?'true':'false'))}
function calibrateOpen(){post('/api/v1/flatpanel/calibrate/open')} function calibrateClosed(){post('/api/v1/flatpanel/calibrate/closed')} function resetCalibration(){post('/api/v1/flatpanel/calibrate/reset')}
async function refresh(){const r=await fetch('/api/v1/flatpanel/environment');const d=await r.json();
 document.getElementById('coverState').textContent=d.coverStateText; document.getElementById('brightnessLabel').textContent=d.brightness; document.getElementById('brightness').value=d.brightness;
 document.getElementById('heaterState').textContent=d.heaterOn?'AN':'AUS'; document.getElementById('heaterEnabled').textContent=d.heaterEnabled?'AN':'AUS';
 document.getElementById('temp').textContent=isFinite(d.temperatureC)?d.temperatureC.toFixed(1)+' °C':'-'; document.getElementById('hum').textContent=isFinite(d.humidityPct)?d.humidityPct.toFixed(1)+' %':'-';
 document.getElementById('pres').textContent=isFinite(d.pressureHpa)?d.pressureHpa.toFixed(1)+' hPa':'-'; document.getElementById('dew').textContent=isFinite(d.dewPointC)?d.dewPointC.toFixed(1)+' °C':'-';
 document.getElementById('rawPos').textContent=d.rawPosition ?? '-'; document.getElementById('filteredPos').textContent=d.filteredPosition ?? '-'; document.getElementById('mappedPos').textContent=isFinite(d.mappedPosition)?d.mappedPosition+' %':'-';
 document.getElementById('openRaw').textContent=d.calibrationOpenRaw ?? '-'; document.getElementById('closedRaw').textContent=d.calibrationClosedRaw ?? '-';}
setInterval(refresh,2000); refresh();
</script></body></html>
)rawliteral";

void setupAlpacaHandlers(AsyncWebServer& server) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/setup");
    });
    server.on("/api/v1/flatpanel/environment", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); sendEnvironmentJson(request); });
    server.on("/api/v1/flatpanel/heaterenabled", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const String enabled = getParamValue(request, "Enabled", "true");
        setHeaterEnabled(parseBoolParam(enabled, true));
        DynamicJsonDocument doc(128); doc["Value"] = isHeaterEnabled(); sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/flatpanel/heatermanual", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const String on = getParamValue(request, "On", "false");
        setHeaterManual(parseBoolParam(on, false));
        DynamicJsonDocument doc(128); doc["Value"] = isHeaterOn(); sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/flatpanel/servo", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const int pulseUs = getParamValue(request, "PulseUs", String(getServoPulseUs())).toInt();
        DynamicJsonDocument doc(128); doc["Value"] = setServoPulseUs(pulseUs); sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/flatpanel/servosmooth", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const String enabled = getParamValue(request, "Enabled", "true");
        setServoSmoothEnabled(parseBoolParam(enabled, true));
        DynamicJsonDocument doc(128); doc["Value"] = isServoSmoothEnabled(); sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/flatpanel/servospeed", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const int speed = getParamValue(request, "MaxSpeedUsPerSec", String(getServoMaxSpeedUsPerSec())).toInt();
        setServoMaxSpeedUsPerSec(speed);
        DynamicJsonDocument doc(128); doc["Value"] = getServoMaxSpeedUsPerSec(); sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/flatpanel/calibrate/open", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = calibrateCurrentPositionAsOpen(); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/flatpanel/calibrate/closed", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = calibrateCurrentPositionAsClosed(); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/flatpanel/calibrate/reset", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); resetPositionCalibration(); DynamicJsonDocument doc(128); doc["Value"] = true; sendAlpacaResponseRaw(request, doc); });

    server.on("/management/apiversions", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); JsonArray arr = doc.createNestedArray("Value"); arr.add(1); sendAlpacaResponseRaw(request, doc); });
    server.on("/management/v1/description", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(256); JsonObject value = doc.createNestedObject("Value"); value["ServerName"] = APP_NAME; value["Manufacturer"] = "Open source ESP32 Alpaca"; value["ManufacturerVersion"] = APP_VERSION; value["Location"] = WiFi.localIP().toString(); sendAlpacaResponseRaw(request, doc); });
    server.on("/management/v1/configureddevices", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(256); JsonArray value = doc.createNestedArray("Value"); JsonObject dev = value.createNestedObject(); dev["DeviceName"] = APP_NAME; dev["DeviceType"] = "CoverCalibrator"; dev["DeviceNumber"] = 0; dev["UniqueID"] = "flipflatpanel-esp32-001"; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/connected", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = g_connected; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/connected", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); const String val = getParamValue(request, "Connected", "true"); g_connected = parseBoolParam(val, true); DynamicJsonDocument doc(128); doc["Value"] = g_connected; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/description", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = "ESP32 Flatpanel with cover, BME280 and dew heater"; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/driverinfo", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = "ASCOM Alpaca CoverCalibrator for ESP32"; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/driverversion", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = APP_VERSION; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/interfaceversion", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = 1; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/name", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = APP_NAME; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/supportedactions", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(256); JsonArray arr = doc.createNestedArray("Value"); arr.add("environment"); arr.add("heaterenabled"); arr.add("calibration"); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/brightness", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = getBrightness(); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/brightness", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); int value = getParamValue(request, "Brightness", "0").toInt(); setBrightness(value); DynamicJsonDocument doc(128); doc["Value"] = getBrightness(); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/maxbrightness", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = getMaxBrightness(); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/calibratorstate", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = static_cast<int>(getCalibratorState()); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/coverstate", HTTP_GET, [](AsyncWebServerRequest* request) { logRequest(request); DynamicJsonDocument doc(128); doc["Value"] = static_cast<int>(getCoverState()); sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/calibratoron", HTTP_PUT, [](AsyncWebServerRequest* request) {
        logRequest(request);
        const int value = getParamValue(request, "Brightness", "255").toInt();
        if (!turnCalibratorOn(value)) {
            sendAlpacaError(request, ALPACA_ERR_INVALID_VALUE, "Brightness must be in the range 0 to 255.");
            return;
        }
        DynamicJsonDocument doc(128); doc["Value"] = true; sendAlpacaResponseRaw(request, doc);
    });
    server.on("/api/v1/covercalibrator/0/calibratoroff", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); turnCalibratorOff(); DynamicJsonDocument doc(128); doc["Value"] = true; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/opencover", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); openCover(); DynamicJsonDocument doc(128); doc["Value"] = true; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/closecover", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); closeCover(); DynamicJsonDocument doc(128); doc["Value"] = true; sendAlpacaResponseRaw(request, doc); });
    server.on("/api/v1/covercalibrator/0/haltcover", HTTP_PUT, [](AsyncWebServerRequest* request) { logRequest(request); sendAlpacaError(request, ALPACA_ERR_NOT_IMPLEMENTED, "HaltCover is not implemented because cover movement is synchronous."); });
}
