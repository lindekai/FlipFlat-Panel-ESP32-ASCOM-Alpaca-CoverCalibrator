#include "debug_monitor.h"

namespace Monitor {

static constexpr size_t LOG_BUFFER_MAX = 7000;
static constexpr size_t LOG_LINE_MAX = 280;

static String logBuffer;
static String currentLine;

static void appendCompletedLine() {
    String entry = "[";
    entry += millis();
    entry += "] ";
    entry += currentLine;
    entry += "\n";
    currentLine = "";

    logBuffer += entry;
    while (logBuffer.length() > LOG_BUFFER_MAX) {
        const int newline = logBuffer.indexOf('\n');
        if (newline < 0) {
            logBuffer.remove(0, logBuffer.length() - LOG_BUFFER_MAX);
            break;
        }
        logBuffer.remove(0, newline + 1);
    }
}

class MirrorPrint : public Print {
public:
    size_t write(uint8_t c) override {
        const size_t written = Serial.write(c);
        if (c == '\n') {
            appendCompletedLine();
        } else if (c != '\r') {
            if (currentLine.length() >= LOG_LINE_MAX) {
                currentLine.remove(0, currentLine.length() - LOG_LINE_MAX + 1);
            }
            currentLine += static_cast<char>(c);
        }
        return written;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        size_t written = 0;
        for (size_t i = 0; i < size; ++i) {
            written += write(buffer[i]);
        }
        return written;
    }
};

static MirrorPrint monitorStream;

void begin(uint32_t baud) {
    Serial.begin(baud);
    Serial.setTimeout(50);
}

Print& stream() {
    return monitorStream;
}

String getLog() {
    String snapshot = logBuffer;
    if (!currentLine.isEmpty()) {
        snapshot += "[";
        snapshot += millis();
        snapshot += "] ";
        snapshot += currentLine;
        snapshot += "\n";
    }
    return snapshot;
}

void clearLog() {
    logBuffer = "";
    currentLine = "";
}

size_t println() {
    return stream().println();
}

} // namespace Monitor
