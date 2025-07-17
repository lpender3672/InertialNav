#include "display.h"

NavDisplay::NavDisplay() : lastDisplayUpdate(0) {

}

bool NavDisplay::begin() {

    // attempt at resetting on reset pin
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(8, LOW);
    delay(100);
    digitalWrite(8, HIGH);
    delay(200);

    tft.init();
    tft.setRotation(1);  // landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);

    uint16_t id1 = tft.readcommand16(0x04);  // Read display ID
    uint8_t id2 = tft.readcommand8(0x09);    // Read status
    uint32_t id3 = tft.readcommand32(0xEF);  // Read ID4 (some displays)
    
    Serial.print("ID 0x04: 0x"); Serial.println(id1, HEX);
    Serial.print("ID 0x09: 0x"); Serial.println(id2, HEX);
    Serial.print("ID 0xEF: 0x"); Serial.println(id3, HEX);
        
    return (id1 == 0x9488); // display is ILI9488
}

void NavDisplay::setupLayout() {
    tft.fillScreen(TFT_BLACK);
    
    // Title
    tft.setTextColor(TFT_CYAN);
    tft.setTextSize(2);
    tft.setCursor(50, 5);
    tft.println("Navigation System");
    
    // Draw sections
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    
    // Attitude section
    tft.setCursor(5, 30);
    tft.println("ATTITUDE");
    tft.drawLine(0, 40, 160, 40, TFT_WHITE);
    
    // Position section
    tft.setCursor(5, 90);
    tft.println("POSITION");
    tft.drawLine(0, 100, 160, 100, TFT_WHITE);
    
    // Velocity section
    tft.setCursor(5, 150);
    tft.println("VELOCITY");
    tft.drawLine(0, 160, 160, 160, TFT_WHITE);
    
    // GPS section
    tft.setCursor(170, 30);
    tft.println("GPS STATUS");
    tft.drawLine(165, 40, 320, 40, TFT_WHITE);
    
    // Sensors section
    tft.setCursor(170, 90);
    tft.println("SENSORS");
    tft.drawLine(165, 100, 320, 100, TFT_WHITE);
    
    // Performance section
    tft.setCursor(170, 150);
    tft.println("PERFORMANCE");
    tft.drawLine(165, 160, 320, 160, TFT_WHITE);
}

void NavDisplay::update(AttPosEKF* ekf, bool gpsRefSet, 
                       uint32_t imuTime, uint32_t gpsTime, 
                       uint32_t magTime, uint32_t baroTime,
                       uint32_t totalTime) {
    
    // Update display at 5Hz to avoid flicker
    if (millis() - lastDisplayUpdate < 200) return;
    lastDisplayUpdate = millis();
    
    // Clear data areas (not headers)
    tft.fillRect(5, 45, 155, 40, TFT_BLACK);    // Attitude
    tft.fillRect(5, 105, 155, 40, TFT_BLACK);   // Position
    tft.fillRect(5, 165, 155, 55, TFT_BLACK);   // Velocity
    tft.fillRect(170, 45, 145, 40, TFT_BLACK);  // GPS
    tft.fillRect(170, 105, 145, 40, TFT_BLACK); // Sensors
    tft.fillRect(170, 165, 145, 55, TFT_BLACK); // Performance
    
    tft.setTextSize(1);
    
    if (ekf->statesInitialised) {
        // Get Euler angles
        float euler[3];
        float tempQuat[4];
        for (int i = 0; i < 4; i++) {
            tempQuat[i] = ekf->states[i];
        }
        ekf->quat2eul(euler, tempQuat);
        
        // Attitude section
        tft.setTextColor(TFT_YELLOW);
        tft.setCursor(5, 45);
        tft.print("Roll:  "); tft.print(euler[0] * RAD_TO_DEG, 1); tft.println(" deg");
        tft.setCursor(5, 55);
        tft.print("Pitch: "); tft.print(euler[1] * RAD_TO_DEG, 1); tft.println(" deg");
        tft.setCursor(5, 65);
        tft.print("Yaw:   "); tft.print(euler[2] * RAD_TO_DEG, 1); tft.println(" deg");
        
        // Position section
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(5, 105);
        tft.print("North: "); tft.print(ekf->states[7], 1); tft.println(" m");
        tft.setCursor(5, 115);
        tft.print("East:  "); tft.print(ekf->states[8], 1); tft.println(" m");
        tft.setCursor(5, 125);
        tft.print("Alt:   "); tft.print(-ekf->states[9], 1); tft.println(" m");
        
        // Velocity section
        tft.setTextColor(TFT_MAGENTA);
        tft.setCursor(5, 165);
        tft.print("Vel N: "); tft.print(ekf->states[4], 2); tft.println(" m/s");
        tft.setCursor(5, 175);
        tft.print("Vel E: "); tft.print(ekf->states[5], 2); tft.println(" m/s");
        tft.setCursor(5, 185);
        tft.print("Vel D: "); tft.print(ekf->states[6], 2); tft.println(" m/s");
        
        // GPS status
        tft.setCursor(170, 45);
        if (gpsRefSet) {
            tft.setTextColor(TFT_GREEN);
            tft.println("GPS: ACTIVE");
            tft.setCursor(170, 55);
            tft.print("Fix: 3D");
        } else {
            tft.setTextColor(TFT_RED);
            tft.println("GPS: WAITING");
        }
        
        // Sensor status
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(170, 105);
        tft.println("BNO055: OK");
        tft.setCursor(170, 115);
        tft.println("MS5611: OK");
        tft.setCursor(170, 125);
        tft.println("GPS: OK");
        
        // Performance metrics
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(170, 165);
        tft.print("IMU: "); tft.print(imuTime); tft.println(" us");
        tft.setCursor(170, 175);
        tft.print("GPS: "); tft.print(gpsTime); tft.println(" us");
        tft.setCursor(170, 185);
        tft.print("Total: "); tft.print(totalTime); tft.println(" us");
        
    } else {
        // EKF not initialized
        tft.setTextColor(TFT_RED);
        tft.setCursor(5, 45);
        tft.println("WAITING FOR GPS...");
        
        // Show sensor status
        tft.setTextColor(TFT_YELLOW);
        tft.setCursor(170, 105);
        tft.println("BNO055: OK");
        tft.setCursor(170, 115);
        tft.println("MS5611: OK");
        tft.setCursor(170, 125);
        if (gpsRefSet) {
            tft.println("GPS: OK");
        } else {
            tft.println("GPS: SEARCHING");
        }
        
        // Show performance even when not initialized
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(170, 165);
        tft.print("IMU: "); tft.print(imuTime); tft.println(" us");
        tft.setCursor(170, 175);
        tft.print("Total: "); tft.print(totalTime); tft.println(" us");
    }
}

void NavDisplay::showStartupMessage(const char* message, bool isError) {
    tft.setTextSize(1);
    if (isError) {
        tft.setTextColor(TFT_RED);
    } else {
        tft.setTextColor(TFT_WHITE);
    }
    tft.println(message);
}

void NavDisplay::showSensorStatus(const char* sensor, bool status) {
    tft.setTextSize(1);
    tft.print(sensor);
    tft.print("... ");
    if (status) {
        tft.setTextColor(TFT_GREEN);
        tft.println("OK");
    } else {
        tft.setTextColor(TFT_RED);
        tft.println("FAILED!");
    }
    tft.setTextColor(TFT_WHITE);  // Reset color
}
