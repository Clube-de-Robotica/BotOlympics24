// =============================================================
// = Header File BotFCTUC                       BotOlympics 2024
// = JNDVasco - Rev 1.0
// = CodeCritical - Rev 2.0
// = LittleNyanCat - Rev 3.0
// =
// = Hardware Mapping
// = |---------------------|--------|
// = |     Descrição       |  Pino  |
// = |---------------------|--------|
// = |Motor Esquerda - A   |   27   |
// = |---------------------|--------|
// = |Motor Esquerda - B   |   13   |
// = |---------------------|--------|
// = |Motor Direita - A    |   17   |
// = |---------------------|--------|
// = |Motor Direita - B    |   16   |
// = |---------------------|--------|
// = |LED NeoPixel         |   14   |
// = |---------------------|--------|
// = |Buzzer               |   26   |
// = |---------------------|--------|
// = |XSHUT LiDAR Direita  |   25   |
// = |---------------------|--------|
// = |XSHUT LiDAR Frente   |   33   |
// = |---------------------|--------|
// = |XSHUT LiDAR Esquerda |   32   |
// = |---------------------|--------|
// = |Tensão bateria       |   36   |
// = |---------------------|--------|
// = |Botão                |   39   |
// = |---------------------|--------|
// = |RFID Esq. SDA        |    5   |
// = |---------------------|--------|
// = |RFID Dir. SDA        |   12   |
// = |---------------------|--------|
// =
// =============================================================

#ifndef BOT_FCTUC_H
#define BOT_FCTUC_H

// Auxiliary libraries
#include <Wire.h>
#include <WiFi.h>

// Third-party auxiliary libraries
#include <./external/VL53L0X.h>
#include <NeoPixelBus.h>
#include <MFRC522.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>

/**
 * @brief Holds a (X, Y) position and if it is valid or not
 */
struct Position {
    bool isValid;
    uint8_t x;
    uint8_t y;
};

/**
 * @brief Class to interface with robot hardware.
 */
class BotFCTUC {
private:
    // Pin definitions
    static constexpr uint16_t PIN_MOTOR_L_1 = 13;
    static constexpr uint16_t PIN_MOTOR_L_2 = 27;
    static constexpr uint16_t PIN_MOTOR_R_1 = 17;
    static constexpr uint16_t PIN_MOTOR_R_2 = 16;
    static constexpr uint16_t PIN_NEOPIXEL = 14;
    static constexpr uint16_t PIN_BUZZER = 26;
    static constexpr uint16_t PIN_XSHUT_RIGHT = 25;
    static constexpr uint16_t PIN_XSHUT_FRONT = 33;
    static constexpr uint16_t PIN_XSHUT_LEFT = 32;
    static constexpr uint16_t PIN_BUTTON = 39;
    static constexpr uint16_t PIN_BAT_SENSE = 36;
    static constexpr uint16_t PIN_RFID_SDA_L = 5;
    static constexpr uint16_t PIN_RFID_SDA_R = 12;

    //LIDAR constants
    static constexpr uint16_t ADDR_LIDAR_LEFT= 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT = 0x71;
    static constexpr uint16_t ADDR_LIDAR_RIGHT = 0x72;

    static constexpr uint16_t DIST_LIDAR_MIN = 0;
    static constexpr uint16_t DIST_LIDAR_MAX = 2600;

    static constexpr float MIN_BAT_VOLTAGE = 5.8f;

    static constexpr uint8_t PWM_RESOLUTION_BITS = 9;
    static constexpr int16_t DUTY_MOTOR_MAX = 512 - 1;  //Must be a power of 2

    //WiFi functionality constants
    const char *WIFI_SSID = "Testing";
    const char *WIFI_PW = "12345678";
    static constexpr int16_t OTA_BLINK_DELAY = 800;
    static constexpr uint8_t MAX_TCP_CLIENTS = 1;

    //RFID Reader Constants
    static constexpr uint8_t TAG_BLOCK_POS = 61;
    MFRC522::MIFARE_Key TAG_KEY = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // public key, only allows read operations
    static constexpr uint8_t TAG_ANTENNA_SWITCH_MS = 25;

    //Used for printing out hex
    static constexpr char HexToCharMap[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };


    static BotFCTUC* botRef;
    TaskHandle_t botLoopHandle;
    TaskHandle_t mainLoopTask;
    bool startButtonOverride = false;

    uint16_t otaBlinkDelay = OTA_BLINK_DELAY;
    volatile bool otaInProgress = false;

    static AsyncServer *server;
    AsyncClient *tcpClients[MAX_TCP_CLIENTS];

    bool botOperational = false;

    MFRC522 *RFIDPtr;   //Pointer to currently active RFID reader
    uint64_t antennaSwitchMillis = 0;

    //Index 0 is left, Index 1 is right
    int8_t motorOffsets[2] = {};  
    bool invertDir[2] = {};

    void setupButton();
    void setupBuzzer();
    void setupNeopixel();
    void setupMotors(bool, bool);
    void setupLidar();
    void setupRFID();
    void setupWifi();

    static void handleTCPConnect(void*, AsyncClient*);
    static void handleTCPDisconnect(void*, AsyncClient*);
    static void handleTCPData(void*, AsyncClient*, void*, size_t);

    static void BotLoop(void* p) {((BotFCTUC*) p)->BotLoop();}  //cursed workaroud to tasks only working on static methods
    void BotLoop();
    void StopBot();
    bool SelfTest();

    static NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2811Method> deviceNeoPixel;
    VL53L0X deviceLidarRight;
    VL53L0X deviceLidarFront;
    VL53L0X deviceLidarLeft;
    static MFRC522 deviceRFIDL;
    static MFRC522 deviceRFIDR;

public:
    bool begin(bool shouldInvertMotorL = false, bool shouldInvertMotorR = false);

    void fanOn();
    void fanOff();

    void setOTAPassword(const String&);
    void beginOTA(const String&);
    void waitStart();
    bool readButton();

    void buzzer(uint16_t);
    void buzzer(uint16_t, uint16_t);

    void setPixelColor(uint8_t, uint8_t, uint8_t);


    void setMotorOffsets(int8_t, int8_t);  
    void moveMotorL(int16_t);
    void moveMotorR(int16_t);
    void moveMotors(int16_t, int16_t);
    void stopMotors();


    uint16_t getLidarRightDistance();
    uint16_t getLidarFrontDistance();
    uint16_t getLidarLeftDistance();

    float getBatVoltage();

    bool tagDetected();
    Position readPosition();
    void dumpTagInfo();


    void print(const char*);

    void printI2C();
    void printLidarValue();
    void printRfidPcdFw();


    //method overloads for print()
    //I did not want to inline these, but c++ forced my hand
    /**
    @brief Prints a string via serial and WiFi
     */
    template<typename T> void print(T x) { print( String(x).c_str() ); }; 
    /**
    @brief Prints a string via serial and WiFi and appends a \n at the end
     */
    template<typename T> void println(T x) { print( (String(x) + '\n').c_str() ); };
};

#endif