#ifndef DISPLAY_H
#define DISPLAY_H
#include <Arduino.h>
#include <TFT_eSPI.h>
#include "estimator_22states.h"

// Display pins are configured in TFT_eSPI/User_Setup.h
//#define TFT_CS   10
//#define TFT_DC   9
//#define TFT_RST  8
//#define TFT_MOSI 11
//#define TFT_MISO 12
//#define TFT_SCLK 13

class NavDisplay {
private:
    TFT_eSPI tft;
    uint32_t lastDisplayUpdate;
   
public:
    NavDisplay();
    bool begin();
    void setupLayout();
    void update(AttPosEKF* ekf, bool gpsRefSet,
                uint32_t imuTime, uint32_t gpsTime,
                uint32_t magTime, uint32_t baroTime,
                uint32_t totalTime);
    void showStartupMessage(const char* message, bool isError = false);
    void showSensorStatus(const char* sensor, bool status);
};
#endif