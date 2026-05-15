#include "alpaca_discovery.h"

#include "debug_monitor.h"

#include <WiFiUdp.h>

static constexpr uint16_t ALPACA_DISCOVERY_PORT = 32227;
static constexpr uint16_t ALPACA_HTTP_PORT = 80;
static const char* ALPACA_DISCOVERY_MESSAGE = "alpacadiscovery1";
static const char* ALPACA_DISCOVERY_RESPONSE = "{\"AlpacaPort\":80}";

static WiFiUDP discoveryUdp;
static bool discoveryStarted = false;

void initAlpacaDiscovery() {
    if (discoveryStarted) return;

    discoveryStarted = discoveryUdp.begin(ALPACA_DISCOVERY_PORT);
    Monitor::print("[ALPACA] Discovery UDP ");
    Monitor::print(ALPACA_DISCOVERY_PORT);
    Monitor::println(discoveryStarted ? " bereit" : " Start fehlgeschlagen");
}

void handleAlpacaDiscovery() {
    if (!discoveryStarted) return;

    const int packetSize = discoveryUdp.parsePacket();
    if (packetSize <= 0) return;

    char buffer[32] = {};
    const int bytesRead = discoveryUdp.read(buffer, sizeof(buffer) - 1);
    if (bytesRead <= 0) return;

    if (String(buffer) != ALPACA_DISCOVERY_MESSAGE) {
        return;
    }

    discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
    discoveryUdp.write(reinterpret_cast<const uint8_t*>(ALPACA_DISCOVERY_RESPONSE), strlen(ALPACA_DISCOVERY_RESPONSE));
    discoveryUdp.endPacket();

    Monitor::print("[ALPACA] Discovery Antwort an ");
    Monitor::print(discoveryUdp.remoteIP());
    Monitor::print(":");
    Monitor::print(discoveryUdp.remotePort());
    Monitor::print(" AlpacaPort=");
    Monitor::println(ALPACA_HTTP_PORT);
}
