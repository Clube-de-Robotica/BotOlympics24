#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Wire.h>

class LiDAR {
private:
    static constexpr uint16_t PIN_XSHUT_RIGHT = 7;
    static constexpr uint16_t PIN_XSHUT_FRONT = 6;
    static constexpr uint16_t PIN_XSHUT_LEFT = 5;

    static constexpr uint16_t ADDR_LIDAR_RIGHT = 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT = 0x71;
    static constexpr uint16_t ADDR_LIDAR_LEFT= 0x72;

    static constexpr uint16_t DIST_LIDAR_MIN = 0;
    static constexpr uint16_t DIST_LIDAR_MAX = 2600;

    VL53L0X deviceLidarRight;
    VL53L0X deviceLidarFront;
    VL53L0X deviceLidarLeft;

public:
    LiDAR();
    ~LiDAR();

    void begin();

    void scanI2C();

    uint16_t getLidarRightDistance();
    uint16_t getLidarFrontDistance();
    uint16_t getLidarLeftDistance();
};

extern LiDAR Lidar;

#endif