#include <Arduino.h>
#include "estimator_22states.h"

// Global EKF instance
AttPosEKF* ekf;

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
    
    // Create EKF instance
    ekf = new AttPosEKF();
    
    // Initialize your sensors here
    // initIMU();
    // initGPS();
    // initMag();
    // initBaro();
    
    Serial.println("EKF initialized");
}


void updateIMU() {
    uint32_t startTime = micros();
    
    // Read IMU data from your sensor
    float gx, gy, gz;  // rad/s
    float ax, ay, az;  // m/s^2
    
    // TODO: Read from your IMU sensor
    // gx = readGyroX();
    // gy = readGyroY();
    // gz = readGyroZ();
    // ax = readAccelX();
    // ay = readAccelY();
    // az = readAccelZ();
    
    // For now, using dummy data
    gx = 0.0f;
    gy = 0.0f;
    gz = 0.0f;
    ax = 0.0f;
    ay = 0.0f;
    az = -9.81f;
    
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
    // if (!gpsAvailable()) return;
    
    // Read GPS data
    float lat, lon, alt;     // degrees, degrees, meters
    float velN, velE, velD;  // m/s
    uint8_t fixType;         // 0=no fix, 3=3D fix
    
    // TODO: Read from your GPS
    // lat = readGPSLat();
    // lon = readGPSLon();
    // alt = readGPSAlt();
    // velN = readGPSVelN();
    // velE = readGPSVelE();
    // velD = readGPSVelD();
    // fixType = readGPSFixType();
    
    // For now, dummy data
    lat = 37.7749f;
    lon = -122.4194f;
    alt = 100.0f;
    velN = 0.0f;
    velE = 0.0f;
    velD = 0.0f;
    fixType = 3;
    
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
    float baroAlt;  // meters
    
    // TODO: Read from your barometer
    // baroAlt = readBaroAltitude();
    
    // For now, dummy data
    baroAlt = 100.0f;
    
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
