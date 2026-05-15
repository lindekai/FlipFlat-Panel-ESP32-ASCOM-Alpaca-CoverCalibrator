#pragma once

#include <Arduino.h>

#ifndef MONITOR_BAUD
#define MONITOR_BAUD 115200
#endif

namespace Monitor {

void begin(uint32_t baud = MONITOR_BAUD);
Print& stream();
String getLog();
void clearLog();

template <typename T>
size_t print(const T& value) {
    return stream().print(value);
}

template <typename T>
size_t print(const T& value, int format) {
    return stream().print(value, format);
}

template <typename T>
size_t println(const T& value) {
    return stream().println(value);
}

template <typename T>
size_t println(const T& value, int format) {
    return stream().println(value, format);
}

size_t println();

} // namespace Monitor
