#pragma once

#include <Arduino.h>

namespace FlatPanel {

#ifndef APP_NAME
#define APP_NAME "FlipFlatpanel Alpaca"
#endif

#ifndef APP_VERSION
#define APP_VERSION "1.1.0"
#endif

#ifndef APP_HOSTNAME
#define APP_HOSTNAME "flipflatpanel"
#endif

constexpr int SERVO_PIN = 4;
constexpr int LED_PIN = 13;
constexpr int HEATER_PIN = 16;
constexpr int POS_PIN = 5;
constexpr int I2C_SDA_PIN = 8;
constexpr int I2C_SCL_PIN = 9;

constexpr int PANEL_PWM_CHANNEL = 7;
constexpr int PANEL_PWM_FREQ_HZ = 1000;
constexpr int PANEL_PWM_RESOLUTION_BITS = 8;
constexpr bool PANEL_PWM_INVERTED = false;
constexpr bool PANEL_FORCE_OFF_TEST = false;
constexpr bool PANEL_GATE_TOGGLE_TEST = false;
constexpr uint32_t PANEL_GATE_TOGGLE_INTERVAL_MS = 10000;

constexpr bool USE_POSITION_SENSOR = true;
constexpr bool USE_BME280 = true;
constexpr bool COVER_STATE_FOLLOWS_COMMAND = true;

constexpr int SERVO_FREQ_HZ = 50;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_OPEN_US = 2500;
constexpr int SERVO_CLOSE_US = 500;
constexpr uint32_t SERVO_SETTLE_MS = 900;
constexpr uint32_t SERVO_SMOOTH_STEP_MS = 20;
constexpr float SERVO_SMOOTH_RAMP_FRACTION = 0.20f;
constexpr int SERVO_SPEED_MIN_US_PER_SEC = 100;
constexpr int SERVO_SPEED_MAX_US_PER_SEC = 3000;
constexpr int SERVO_SPEED_DEFAULT_US_PER_SEC = 1000;

constexpr int BRIGHTNESS_MIN = 0;
constexpr int BRIGHTNESS_MAX = 255;

constexpr float DEWPOINT_ON_MARGIN_C  = 0.5f;
constexpr float DEWPOINT_OFF_MARGIN_C = 1.0f;

constexpr int SENSOR_RAW_OPEN = 50;
constexpr int SENSOR_RAW_CLOSED = 500;
constexpr int SENSOR_OPEN_THRESHOLD = 5;
constexpr int SENSOR_CLOSED_THRESHOLD = 95;

enum CoverStatus : uint8_t {
    COVER_NOT_PRESENT = 0,
    COVER_CLOSED = 1,
    COVER_MOVING = 2,
    COVER_OPEN = 3,
    COVER_UNKNOWN = 4,
    COVER_ERROR = 5
};

enum CalibratorStatus : uint8_t {
    CAL_NOT_PRESENT = 0,
    CAL_OFF = 1,
    CAL_NOT_READY = 2,
    CAL_READY = 3,
    CAL_UNKNOWN = 4,
    CAL_ERROR = 5
};

void init();
void update();

void openCover();
void closeCover();
void haltCover();
int setServoPulseUs(int pulseUs);
int adjustServoPulseUs(int deltaUs);
int getServoPulseUs();
void setServoSmoothEnabled(bool enabled);
bool isServoSmoothEnabled();
void setServoMaxSpeedUsPerSec(int speedUsPerSec);
int getServoMaxSpeedUsPerSec();

void setBrightness(int value);
bool turnCalibratorOn(int value);
void turnCalibratorOff();
int getBrightness();
int getMaxBrightness();

CoverStatus getCoverState();
CalibratorStatus getCalibratorState();
String coverStateString();
int getRawPosition();
int getFilteredPosition();
int getMappedPosition();
int getCalibrationOpenRaw();
int getCalibrationClosedRaw();
int getCalibrationOpenServoUs();
int getCalibrationClosedServoUs();
bool calibrateCurrentPositionAsOpen();
bool calibrateCurrentPositionAsClosed();
void resetPositionCalibration();

bool bmeAvailable();
float getTemperatureC();
float getHumidityPercent();
float getPressureHpa();
float getDewPointC();

bool isHeaterOn();
void setHeaterEnabled(bool enabled);
bool isHeaterEnabled();
void setHeaterManual(bool on);
void setHeaterAutomatic();
bool isHeaterManualMode();

} // namespace FlatPanel
