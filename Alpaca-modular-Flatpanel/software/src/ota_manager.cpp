#include "ota_manager.h"

#include "debug_monitor.h"
#include "flatpanel_control.h"
#include "wifi_manager.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

static constexpr uint16_t OTA_PORT = 3232;
static const char* OTA_HOSTNAME = APP_HOSTNAME;
static uint32_t g_lastOtaProgressMs = 0;
static bool g_otaStarted = false;

void initOTA() {
    if (g_otaStarted) return;

    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA
        .onStart([]() {
            const String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Monitor::print("[OTA] Start update: ");
            Monitor::println(type);
        })
        .onEnd([]() {
            Monitor::println("[OTA] Update fertig, Neustart");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            const uint32_t now = millis();
            if (now - g_lastOtaProgressMs < 500) return;
            g_lastOtaProgressMs = now;

            Monitor::print("[OTA] Fortschritt ");
            Monitor::print((progress * 100U) / total);
            Monitor::println("%");
        })
        .onError([](ota_error_t error) {
            Monitor::print("[OTA] Fehler ");
            Monitor::print(static_cast<unsigned int>(error));
            Monitor::print(": ");
            if (error == OTA_AUTH_ERROR) {
                Monitor::println("Auth fehlgeschlagen");
            } else if (error == OTA_BEGIN_ERROR) {
                Monitor::println("Begin fehlgeschlagen");
            } else if (error == OTA_CONNECT_ERROR) {
                Monitor::println("Connect fehlgeschlagen");
            } else if (error == OTA_RECEIVE_ERROR) {
                Monitor::println("Receive fehlgeschlagen");
            } else if (error == OTA_END_ERROR) {
                Monitor::println("End fehlgeschlagen");
            } else {
                Monitor::println("unbekannt");
            }
        });

    ArduinoOTA.begin();
    g_otaStarted = true;

    Monitor::print("[OTA] Bereit: ");
    Monitor::print(OTA_HOSTNAME);
    Monitor::print(".local:");
    Monitor::print(OTA_PORT);
    Monitor::print(" IP=");
    Monitor::println(getIPAddress());
}

void handleOTA() {
    if (!g_otaStarted) return;
    ArduinoOTA.handle();
}
