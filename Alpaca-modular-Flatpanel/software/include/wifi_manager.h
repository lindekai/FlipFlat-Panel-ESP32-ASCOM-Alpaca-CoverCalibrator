#pragma once

#include <ESPAsyncWebServer.h>

void initWiFi();
void processDNS();
void setupWiFiEndpoints(AsyncWebServer& server);
String getIPAddress();
