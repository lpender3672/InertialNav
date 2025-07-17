#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5611.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>
#include "estimator_22states.h"
#include "display.h"

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// Barometer
MS5611 ms5611(0x77);
// GPS
SFE_UBLOX_GNSS myGNSS;
// Global EKF instance
AttPosEKF* ekf;

// Display
NavDisplay display;

// Timing variables
uint32_t lastIMUTime = 0;
uint32_t lastGPSTime = 0;
uint32_t lastMagTime = 0;
uint32_t lastBaroTime = 0;

// Sensor delays (milliseconds)
const uint32_t GPS_DELAY_MS = 200;
const uint32_t MAG_DELAY_MS = 30;
const uint32_t BARO_DELAY_MS = 100;

// GPS reference position (set on first fix)
float latRef = 0.0f;
float lonRef = 0.0f;
float altRef = 0.0f;
bool gpsRefSet = false;

// Execution time tracking variables
uint32_t imuExecutionTime = 0;
uint32_t gpsExecutionTime = 0;
uint32_t magExecutionTime = 0;
uint32_t baroExecutionTime = 0;
uint32_t totalExecutionTime = 0;
uint32_t maxImuTime = 0;
uint32_t maxGpsTime = 0;
uint32_t maxMagTime = 0;
uint32_t maxBaroTime = 0;

void setup() {
    Serial.begin(115200);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV128);

    pinMode(10, OUTPUT); // CS
    pinMode(9, OUTPUT);  // DC
    pinMode(8, OUTPUT);  // RST
    
    // Reset sequence
    digitalWrite(8, LOW);
    delay(10);
    digitalWrite(8, HIGH);
    delay(10);
    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    
    if (!display.begin()) {
        Serial.println("Display initialization failed?");
    }

    display.showStartupMessage("Inertial Navigation System");
    display.showStartupMessage("Initializing sensors...");
    
    // Create EKF instance
    ekf = new AttPosEKF();
    
    // Initialize your sensors here
    // initIMU();
    // initGPS();
    // initMag();
    // initBaro();
    
    Serial.println("EKF initialized");

    // Initialize IMU
    Wire.begin();
    Wire.setClock(400000);

    if (!bno.begin()) {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        while (1) {
            delay(1000);
            Serial.println("BNO055 init failed - retrying...");
        }
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    Serial.println("BNO055 initialized successfully");

    // Initialize MS5611
    if (ms5611.begin() == true) {
        Serial.println("MS5611 found");
        Serial.print("MS5611 PROM lib version: ");
        Serial.println(MS5611_LIB_VERSION);
    } else {
        Serial.println("MS5611 not found. halt.");
        while (1);
    }

    if (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR
    
    Serial.println("u-blox GNSS initialized successfully");

}


void updateIMU() {
    uint32_t startTime = micros();

    sensors_event_t orientationData, angVelData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // Read IMU data from your sensor
    float gx, gy, gz;  // rad/s
    float ax, ay, az;  // m/s^2

    // For now, using dummy data
    gx = angVelData.gyro.x;      // rad/s
    gy = angVelData.gyro.y;      // rad/s  
    gz = angVelData.gyro.z;      // rad/s
    ax = linearAccelData.acceleration.x;  // m/s^2
    ay = linearAccelData.acceleration.y;  // m/s^2
    az = linearAccelData.acceleration.z;  // m/s^2

    //Serial.print("IMU Data: ");
    //Serial.print("Gyro (rad/s): "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);
    //Serial.print("Accel (m/s^2): "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
    
    // Calculate dt
    static uint32_t lastIMUUpdate = 0;
    float dt = (millis() - lastIMUUpdate) * 0.001f;
    lastIMUUpdate = millis();
    
    if (dt > 0.0f && dt < 0.1f) {  // Sanity check
        // Update EKF with IMU data
        ekf->dtIMU = dt;
        ekf->angRate.x = gx;
        ekf->angRate.y = gy;
        ekf->angRate.z = gz;
        ekf->accel.x = ax;
        ekf->accel.y = ay;
        ekf->accel.z = az;
        
        // Calculate delta angles and velocities
        static Vector3f lastAngRate;
        static Vector3f lastAccel;
        ekf->dAngIMU = 0.5f * (ekf->angRate + lastAngRate) * ekf->dtIMU;
        ekf->dVelIMU = 0.5f * (ekf->accel + lastAccel) * ekf->dtIMU;
        lastAngRate = ekf->angRate;
        lastAccel = ekf->accel;
        
        if (ekf->statesInitialised) {
            // Run strapdown equations
            ekf->UpdateStrapdownEquationsNED();
            
            // Store states for delayed sensor fusion
            ekf->StoreStates(millis());
            
            // Update covariance if needed
            static float covDt = 0.0f;
            ekf->summedDelAng = ekf->summedDelAng + ekf->correctedDelAng;
            ekf->summedDelVel = ekf->summedDelVel + ekf->dVelIMU;
            covDt += ekf->dtIMU;
            
            if (covDt >= (ekf->covTimeStepMax - ekf->dtIMU) || 
                ekf->summedDelAng.length() > ekf->covDelAngMax) {
                ekf->CovariancePrediction(covDt);
                ekf->summedDelAng.zero();
                ekf->summedDelVel.zero();
                covDt = 0.0f;
            }
            
            ekf->globalTimeStamp_ms = millis();
        }
    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    imuExecutionTime = endTime - startTime;
    if (imuExecutionTime > maxImuTime) {
        maxImuTime = imuExecutionTime;
    }
}

void updateGPS() {
    uint32_t startTime = micros();
    
    // Check if GPS data is available
    if (myGNSS.getPVT() == false) {
        // No new data available
        return;
    }
    
    // Read GPS data
    float lat, lon, alt;     // degrees, degrees, meters
    float velN, velE, velD;  // m/s
    uint8_t fixType, numSats;         // 0=no fix, 3=3D fix
    
    lat = myGNSS.getLatitude() * 1e-7;
    lon = myGNSS.getLongitude() * 1e-7;
    alt = myGNSS.getAltitude() * 1e-3;
    velN = myGNSS.getNedNorthVel() * 1e-3;
    velE = myGNSS.getNedEastVel() * 1e-3;
    velD = myGNSS.getNedDownVel() * 1e-3;
    fixType = myGNSS.getFixType();
    numSats = myGNSS.getSIV();

    //Serial.print("GPS Data: ");
    //Serial.print("Lat: "); Serial.print(lat, 6); Serial.print(", ");
    //Serial.print("Lon: "); Serial.print(lon, 6); Serial.print(", ");
    //Serial.print("Alt: "); Serial.print(alt, 2); Serial.print(" m, ");
    //Serial.print("VelN: "); Serial.print(velN, 2); Serial.print(" m/s, ");
    //Serial.print("VelE: "); Serial.print(velE, 2); Serial.print(" m/s, ");
    //Serial.print("VelD: "); Serial.print(velD, 2); Serial.print(" m/s, ");
    //Serial.print("FixType: "); Serial.print(fixType); Serial.print(", ");
    //Serial.print("NumSats: "); Serial.println(numSats);
    
    if (fixType >= 3) {
        // Convert to radians
        ekf->gpsLat = lat * DEG_TO_RAD;
        ekf->gpsLon = lon * DEG_TO_RAD;
        ekf->gpsHgt = alt;
        ekf->GPSstatus = fixType;
        
        // Set reference on first fix
        if (!gpsRefSet && !ekf->statesInitialised) {
            latRef = ekf->gpsLat;
            lonRef = ekf->gpsLon;
            altRef = ekf->gpsHgt;
            gpsRefSet = true;
            
            // Initialize filter
            float initVelNED[3] = {velN, velE, velD};
            ekf->InitialiseFilter(initVelNED, ekf->gpsLat, ekf->gpsLon, ekf->gpsHgt, 0.0f);
        }
        
        if (ekf->statesInitialised) {
            // Update velocity
            ekf->velNED[0] = velN;
            ekf->velNED[1] = velE;
            ekf->velNED[2] = velD;
            
            // Calculate position in NED frame
            float posNED[3];
            calcposNED(posNED, ekf->gpsLat, ekf->gpsLon, ekf->gpsHgt, 
                      latRef, lonRef, altRef);
            ekf->posNE[0] = posNED[0];  // North
            ekf->posNE[1] = posNED[1];  // East
            
            // Set fusion flags
            ekf->fuseVelData = true;
            ekf->fusePosData = true;
            
            // Recall states at measurement time
            ekf->RecallStates(ekf->statesAtVelTime, millis() - GPS_DELAY_MS);
            ekf->RecallStates(ekf->statesAtPosTime, millis() - GPS_DELAY_MS);
            
            // Fuse GPS data
            ekf->FuseVelposNED();
            
            // Clear flags
            ekf->fuseVelData = false;
            ekf->fusePosData = false;
        }
    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    gpsExecutionTime = endTime - startTime;
    if (gpsExecutionTime > maxGpsTime) {
        maxGpsTime = gpsExecutionTime;
    }
}

void updateMag() {
    uint32_t startTime = micros();
    
    // Read magnetometer data
    float mx, my, mz;  // milligauss
    
    // TODO: Read from your magnetometer
    // mx = readMagX();
    // my = readMagY();
    // mz = readMagZ();
    
    // For now, dummy data (typical earth field)
    mx = 250.0f;
    my = 50.0f;
    mz = -400.0f;
    
    if (ekf->statesInitialised && ekf->useCompass) {
        // Convert to gauss
        ekf->magData.x = mx * 0.001f;
        ekf->magData.y = my * 0.001f;
        ekf->magData.z = mz * 0.001f;
        
        // Set fusion flag
        ekf->fuseMagData = true;
        
        // Recall states at measurement time
        ekf->RecallStates(ekf->statesAtMagMeasTime, millis() - MAG_DELAY_MS);
        
        // Fuse magnetometer data
        ekf->magstate.obsIndex = 0;
        ekf->FuseMagnetometer();
        ekf->FuseMagnetometer();
        ekf->FuseMagnetometer();
        
        ekf->fuseMagData = false;
    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    magExecutionTime = endTime - startTime;
    if (magExecutionTime > maxMagTime) {
        maxMagTime = magExecutionTime;
    }
}

void updateBaro() {
    uint32_t startTime = micros();
    
    // Read barometer data
    
    // TODO: Read from your barometer
    // baroAlt = readBaroAltitude();
    int result = ms5611.read();
    
    if (result != MS5611_READ_OK) {
        Serial.print("MS5611 read error: ");
        Serial.println(result);
        return;
    }

    float temperature = ms5611.getTemperature();
    float pressure = ms5611.getPressure();
    float baroAlt = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));

    //Serial.print("Barometer Data: ");
    //Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" C, ");
    //Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa, ");
    //Serial.print("Altitude: "); Serial.print(baroAlt); Serial.println(" m");
    
    if (ekf->statesInitialised) {
        ekf->baroHgt = baroAlt;
        ekf->hgtMea = ekf->baroHgt - altRef - ekf->baroHgtOffset;
        
        // Set fusion flags
        ekf->fuseVelData = false;
        ekf->fusePosData = false;
        ekf->fuseHgtData = true;
        
        // Recall states at measurement time
        ekf->RecallStates(ekf->statesAtHgtTime, millis() - BARO_DELAY_MS);
        
        // Fuse height data
        ekf->FuseVelposNED();
        
        ekf->fuseHgtData = false;
    }
    
    // Calculate execution time
    uint32_t endTime = micros();
    baroExecutionTime = endTime - startTime;
    if (baroExecutionTime > maxBaroTime) {
        maxBaroTime = baroExecutionTime;
    }
}

void outputState() {
    static uint32_t lastOutput = 0;
    
    // Output at 10Hz
    if (millis() - lastOutput < 100) return;
    lastOutput = millis();
    
    if (!ekf->statesInitialised) {
        Serial.println("EKF not initialized");
        return;
    }
    
    // Get Euler angles
    float euler[3];
    float tempQuat[4];
    for (int i = 0; i < 4; i++) {
        tempQuat[i] = ekf->states[i];
    }
    ekf->quat2eul(euler, tempQuat);
    
    // Output state information
    Serial.print("Time: "); Serial.print(millis());
    Serial.print(" Roll: "); Serial.print(euler[0] * RAD_TO_DEG, 2);
    Serial.print(" Pitch: "); Serial.print(euler[1] * RAD_TO_DEG, 2);
    Serial.print(" Yaw: "); Serial.print(euler[2] * RAD_TO_DEG, 2);
    Serial.print(" Alt: "); Serial.print(-ekf->states[9], 2);
    Serial.print(" VelN: "); Serial.print(ekf->states[4], 2);
    Serial.print(" VelE: "); Serial.print(ekf->states[5], 2);
    Serial.print(" VelD: "); Serial.print(ekf->states[6], 2);
    Serial.println();
    
}

void outputTimings() {
    Serial.print("IMU Execution Time: "); Serial.print(imuExecutionTime); Serial.print(" us, Max: "); Serial.println(maxImuTime);
    Serial.print("GPS Execution Time: "); Serial.print(gpsExecutionTime); Serial.print(" us, Max: "); Serial.println(maxGpsTime);
    Serial.print("Magnetometer Execution Time: "); Serial.print(magExecutionTime); Serial.print(" us, Max: "); Serial.println(maxMagTime);
    Serial.print("Barometer Execution Time: "); Serial.print(baroExecutionTime); Serial.print(" us, Max: "); Serial.println(maxBaroTime);
    totalExecutionTime = imuExecutionTime + gpsExecutionTime + magExecutionTime + baroExecutionTime;
    Serial.print("Total Execution Time: "); Serial.print(totalExecutionTime); Serial.println(" us");
}

void loop() {
    uint32_t currentTime = millis();
    
    // Update IMU at highest rate (example: 100Hz)
    if (currentTime - lastIMUTime >= 10) {
        updateIMU();
        lastIMUTime = currentTime;
    }
    
    // Update GPS at lower rate (example: 10Hz)
    if (currentTime - lastGPSTime >= 100) {
        updateGPS();
        lastGPSTime = currentTime;
    }
    
    // Update Magnetometer (example: 50Hz)
    if (currentTime - lastMagTime >= 20) {
        updateMag();
        lastMagTime = currentTime;
    }
    
    // Update Barometer (example: 20Hz)
    if (currentTime - lastBaroTime >= 50) {
        updateBaro();
        lastBaroTime = currentTime;
    }
    
    // Output current state
    outputState();
    outputTimings();

    display.update(ekf, gpsRefSet, imuExecutionTime, gpsExecutionTime, 
                  magExecutionTime, baroExecutionTime, totalExecutionTime);
}


// Helper function - implement these based on estimator_22states.h
void calcposNED(float (&posNED)[3], float lat, float lon, float alt, 
                float latRef, float lonRef, float altRef) {
    // Simple flat earth approximation
    const float EARTH_RADIUS = 6371000.0f;  // meters
    
    posNED[0] = (lat - latRef) * EARTH_RADIUS;
    posNED[1] = (lon - lonRef) * EARTH_RADIUS * cos(latRef);
    posNED[2] = -(alt - altRef);
}

// Teensy-specific: Use these for microsecond timing if needed
uint64_t getMicros() {
    return micros();
}
