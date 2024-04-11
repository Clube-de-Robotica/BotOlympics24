// =============================================================
// = Header File FCTUC                       BotOlympics 2024
// = JNDVasco - Rev 1.0
// = CodeCritical - Rev 2.0
// = LittleNyanCat - Rev 3.0
// =
// = Hardware Mapping
// = |---------------------|--------|
// = |     Descrição       |  Pino  |
// = |---------------------|--------|
// = |Motor Esquerda - 1   |   17   |
// = |---------------------|--------|
// = |Motor Esquerda - 2   |   16   |
// = |---------------------|--------|
// = |Motor Direita - 1    |   13   |
// = |---------------------|--------|
// = |Motor Direita - 2    |   27   |
// = |---------------------|--------|
// = |LED NeoPixel         |   14   |
// = |---------------------|--------|
// = |Buzzer               |   26   |
// = |---------------------|--------|
// = |XSHUT LiDAR Direita  |   25   |
// = |---------------------|--------|
// = |XSHUT LiDAR Frente   |   32   |
// = |---------------------|--------|
// = |XSHUT LiDAR Esquerda |   33   |
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
#include <WiFiUdp.h>
#include <Vec2.h>

// Third-party auxiliary libraries
#include <./external/VL53L0X.h>
#include <NeoPixelBus.h>
#include <MFRC522.h>


#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
#endif

#ifdef WIFI_MONITORING   
    #include <AsyncTCP.h>
#endif


/**
* @brief Holds the presence of walls on all sides of a cell.
*/
struct Walls{
    bool top : 1;
    bool right : 1;
    bool bottom : 1;
    bool left : 1;
};

/**
 * @brief Class to interface with robot hardware.
 */
class FCTUC {
private:
    // Pin constants
    static constexpr uint16_t PIN_MOTOR_L_1 = 17;
    static constexpr uint16_t PIN_MOTOR_L_2 = 16;
    static constexpr uint16_t PIN_MOTOR_R_1 = 13;
    static constexpr uint16_t PIN_MOTOR_R_2 = 27;
    static constexpr uint16_t PIN_NEOPIXEL = 14;
    static constexpr uint16_t PIN_BUZZER = 26;
    static constexpr uint16_t PIN_XSHUT_RIGHT = 25;
    static constexpr uint16_t PIN_XSHUT_FRONT = 33;
    static constexpr uint16_t PIN_XSHUT_LEFT = 32;
    static constexpr uint16_t PIN_BUTTON = 39;
    static constexpr uint16_t PIN_BAT_SENSE = 36;
    static constexpr uint16_t PIN_RFID_SDA_L = 5;
    static constexpr uint16_t PIN_RFID_SDA_R = 12;

    // LIDAR constants
    static constexpr uint16_t ADDR_LIDAR_LEFT= 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT = 0x71;
    static constexpr uint16_t ADDR_LIDAR_RIGHT = 0x72;

    static constexpr uint16_t DIST_LIDAR_MIN = 0;
    static constexpr uint16_t DIST_LIDAR_MAX = 2600;
    
    // Battery constants
    static constexpr float BAT_CUTOFF_VOLTAGE = 5.8f;
    static constexpr float MIN_BAT_VOLTAGE = 6.1f;
    static constexpr float MAX_BAT_VOLTAGE = 8.5f;

    static constexpr uint8_t PWM_RESOLUTION_BITS = 9;
    static constexpr int16_t DUTY_MOTOR_MAX = 512 - 1;  //Must be a power of 2
    static constexpr int16_t DUTY_MOTOR_CAP = 500;  //Limit the range participants can use so that the offsets can always be properly applied

    // WiFi constants
    static constexpr char WIFI_SSID[] = "Testing";
    static constexpr char WIFI_PWD[] = "12345678";
    static constexpr uint8_t MAX_TCP_CLIENTS = 1;
    static constexpr int16_t UDP_MSG_CHECK_INTERVAL = 500;

    // RFID constants
    MFRC522::MIFARE_Key TAG_KEY = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // public key, only allows read operations
    static constexpr uint8_t TAG_BLOCK_POS = 61;
    static constexpr uint8_t TAG_ANTENNA_READ_MS = 5;
    static constexpr uint8_t TAG_ANTENNA_SWITCH_MS = 25;



    // Internal variables

    static TaskHandle_t batteryTaskHandle;
    static TaskHandle_t rfidTaskHandle;
    static TaskHandle_t udpTaskHandle;
    #ifdef ENABLE_OTA
        static TaskHandle_t OTATaskHandle;
    #endif

    static WiFiUDP udp;

    static bool startButtonOverride;
    static bool botEnabled;

    #ifdef WIFI_MONITORING
        static AsyncServer *server;
        static AsyncClient *tcpClients[MAX_TCP_CLIENTS];
    #endif

    static Vec2 targetPosition;
    static Vec2 currentPosition;
    static bool hasRoundFinished;

    static bool isTagDetected;
    static bool isTagReadSuccess;


    MFRC522 *RFIDPtr;   //Pointer to currently active RFID reader
  

    void setupButton();
    void setupBuzzer();
    void setupNeopixel();
    void setupMotors();
    void setupLidar();
    void setupRFID();
    void setupWifi();

    #ifdef WIFI_MONITORING
        static void handleTCPConnect(void*, AsyncClient*);
        static void handleTCPDisconnect(void*, AsyncClient*);
        static void handleTCPData(void*, AsyncClient*, void*, size_t);
    #endif

    static void taskMonitorBatteryValue(void*);
    static void taskReadActiveRFIDValue(void*);
    static void taskMonitorTargetPosition(void*);
    
    #ifdef ENABLE_OTA
        static void taskHandleOTA(void*);
    #endif

    static void stopBot();
    bool doSelfTest();

    static NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2811Method> deviceNeoPixel;
    
    static MFRC522 deviceRFIDLeft;
    static MFRC522 deviceRFIDRight;

    static VL53L0X deviceLidarRight;
    static VL53L0X deviceLidarFront;
    static VL53L0X deviceLidarLeft;

public:
    bool begin();


    #ifdef ENABLE_OTA
        void beginOTA(const String&);
    #endif
    
    void waitStart();
    bool readButton();

    void buzzer(uint16_t);
    void buzzer(uint16_t, uint16_t);

    static void setPixelColor(uint8_t, uint8_t, uint8_t);


    void setMotorOffsets(int8_t, int8_t);  
    static void moveMotorLeft(int16_t);
    static void moveMotorRight(int16_t);
    static void moveMotors(int16_t, int16_t);
    static void stopMotors();


    uint16_t getLidarRightDistance();
    uint16_t getLidarFrontDistance();
    uint16_t getLidarLeftDistance();

    static float getBatteryVoltage();
    static float getBatteryPercent();

    static Vec2 getRobotPosition();
    static Vec2 getThiefPosition();
    static bool getRoundFinished();

    static bool getTagDetected();
    static bool getTagReadSuccess();


    void printI2C();
    void printLidarValue();
    void printRfidPcdFw();

    static void print(const char*);

    //method overloads for print()
    //I did not want to inline these, but c++ forced my hand
    /**
    @brief Prints a string via serial and WiFi
     */
    template<typename T> static void print(T x) { print( String(x).c_str() ); }; 
    /**
    @brief Prints a string via serial and WiFi and appends a \n at the end
     */
    template<typename T> static void println(T x) { print( (String(x) + '\n').c_str() ); };

};


Walls GetWallsAtPos(const uint8_t, const uint8_t, const uint8_t *);
Walls GetWallsAtPos(const Vec2, const uint8_t *);



#endif