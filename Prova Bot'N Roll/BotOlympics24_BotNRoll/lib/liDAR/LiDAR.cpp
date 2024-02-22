#include "LiDAR.h"

LiDAR::LiDAR() {

}

LiDAR::~LiDAR() {

}

void LiDAR::begin() {
    Wire.begin();

    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);

    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    digitalWrite(PIN_XSHUT_FRONT, LOW);
    digitalWrite(PIN_XSHUT_LEFT, LOW);

    delay(150);
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(150);
    deviceLidarRight.setAddress(ADDR_LIDAR_RIGHT);

    delay(150);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(150);
    deviceLidarFront.setAddress(ADDR_LIDAR_FRONT);

    delay(150);
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(150);
    deviceLidarLeft.setAddress(ADDR_LIDAR_LEFT);

    deviceLidarRight.init(true);
    deviceLidarFront.init(true);
    deviceLidarLeft.init(true);

    deviceLidarRight.setTimeout(500);
    deviceLidarFront.setTimeout(500);
    deviceLidarLeft.setTimeout(500);

    deviceLidarRight.startContinuous(0);
    deviceLidarFront.startContinuous(0);
    deviceLidarLeft.startContinuous(0);
}

void LiDAR::scanI2C() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ ) 
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
        Serial.print("I2C device found at address 0x");
        if (address<16) 
            Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");

        nDevices++;
        }
        else if (error==4) 
        {
        Serial.print("Unknown error at address 0x");
        if (address<16) 
            Serial.print("0");
        Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000);
}

uint16_t LiDAR::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

uint16_t LiDAR::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

uint16_t LiDAR::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

LiDAR Lidar = LiDAR();