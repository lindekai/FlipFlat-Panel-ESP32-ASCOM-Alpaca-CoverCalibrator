#include <Arduino.h>

#ifndef SERIAL_ALIVE_ONLY
#define SERIAL_ALIVE_ONLY 0
#endif

#if SERIAL_ALIVE_ONLY

#ifndef MONITOR_BAUD
#define MONITOR_BAUD 115200
#endif

constexpr int SERIAL_RX_PIN = 44;
constexpr int SERIAL_TX_PIN = 43;

void setup() {
    Serial.begin(MONITOR_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
    Serial.println();
    Serial.println("[ALIVE-TEST] start");
    Serial.print("[ALIVE-TEST] tx=");
    Serial.print(SERIAL_TX_PIN);
    Serial.print(" rx=");
    Serial.print(SERIAL_RX_PIN);
    Serial.print(" baud=");
    Serial.println(MONITOR_BAUD);
}

void loop() {
    Serial.print("[ALIVE] ms=");
    Serial.println(millis());
    delay(1000);
}

#else

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "alpaca_discovery.h"
#include "flatpanel_control.h"
#include "alpaca_handlers.h"
#include "debug_monitor.h"
#include "ota_manager.h"
#include "wifi_manager.h"

AsyncWebServer server(80);

void setup() {
    Monitor::begin();
    delay(500);

    Monitor::println();
    Monitor::println("[BOOT] FlipFlatpanel startet");
    Monitor::print("[BOOT] UART0 Monitor TX GPIO43 RX GPIO44 @ ");
    Monitor::print(MONITOR_BAUD);
    Monitor::println(" baud");

    FlatPanel::init();
    initWiFi();
    initOTA();
    initAlpacaDiscovery();
    setupWiFiEndpoints(server);
    setupAlpacaHandlers(server);
    server.begin();

    Monitor::println("[BOOT] Flatpanel server gestartet");
    Monitor::print("[BOOT] IP: ");
    Monitor::println(getIPAddress());
}

void loop() {
    handleOTA();
    handleAlpacaDiscovery();
    FlatPanel::update();
    processDNS();
    delay(1);
}

#endif
