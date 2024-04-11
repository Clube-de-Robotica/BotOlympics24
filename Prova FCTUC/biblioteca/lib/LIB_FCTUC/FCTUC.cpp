#include "FCTUC.h"

TaskHandle_t FCTUC::batteryTaskHandle;
TaskHandle_t FCTUC::rfidTaskHandle;
TaskHandle_t FCTUC::udpTaskHandle;

#ifdef ENABLE_OTA
    TaskHandle_t FCTUC::OTATaskHandle;
#endif

WiFiUDP FCTUC::udp;

#ifdef WIFI_MONITORING    
    AsyncServer *FCTUC::server;
#endif
constexpr char FCTUC::WIFI_SSID[];
constexpr char FCTUC::WIFI_PWD[];

bool FCTUC::startButtonOverride = false;
bool FCTUC::botEnabled = false;
bool FCTUC::hasRoundFinished = false;

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2811Method> FCTUC::deviceNeoPixel(1, PIN_NEOPIXEL);

MFRC522 FCTUC::deviceRFIDLeft(PIN_RFID_SDA_L, MFRC522::UNUSED_PIN);
MFRC522 FCTUC::deviceRFIDRight(PIN_RFID_SDA_R, MFRC522::UNUSED_PIN);

VL53L0X FCTUC::deviceLidarRight;
VL53L0X FCTUC::deviceLidarFront;
VL53L0X FCTUC::deviceLidarLeft;

int8_t motorOffsets[2] = {};

#ifdef WIFI_MONITORING
    AsyncClient* FCTUC::tcpClients[FCTUC::MAX_TCP_CLIENTS];
#endif

static bool startButtonOverride = false;

Vec2 FCTUC::targetPosition = {9, 9};
Vec2 FCTUC::currentPosition = {255, 255};
bool FCTUC::isTagDetected = false;
bool FCTUC::isTagReadSuccess = false;

/**
 * @brief Set offsets for the motors to ensure their speeds are consistent.
 * @param R, L The offsets applied to each of the motors. These values may be negative.
 */
void FCTUC::setMotorOffsets(int8_t R, int8_t L){
    motorOffsets[0] = L;
    motorOffsets[1] = R;
}

void FCTUC::setupButton() {
    pinMode(PIN_BUTTON, INPUT);
}

void FCTUC::setupBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
}

void FCTUC::setupNeopixel() {
    deviceNeoPixel.Begin();
}


void FCTUC::setupMotors() {
    pinMode(PIN_MOTOR_L_1, OUTPUT);
    pinMode(PIN_MOTOR_L_2, OUTPUT);
    pinMode(PIN_MOTOR_R_1, OUTPUT);
    pinMode(PIN_MOTOR_R_2, OUTPUT);

    analogWriteResolution(PWM_RESOLUTION_BITS);

    moveMotors(0,0);
}

void FCTUC::setupLidar() {
    Wire.begin();
    
    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);

    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(200);
    deviceLidarRight.setAddress(ADDR_LIDAR_RIGHT);
    deviceLidarRight.setTimeout(500);
    deviceLidarRight.init(true);

    digitalWrite(PIN_XSHUT_FRONT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(200);
    deviceLidarFront.setAddress(ADDR_LIDAR_FRONT);
    deviceLidarFront.setTimeout(500);
    deviceLidarFront.init(true);

    digitalWrite(PIN_XSHUT_LEFT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(200);
    deviceLidarLeft.setAddress(ADDR_LIDAR_LEFT);
    deviceLidarLeft.setTimeout(500);
    deviceLidarLeft.init(true);

    deviceLidarRight.startContinuous(0);
    deviceLidarFront.startContinuous(0);
    deviceLidarLeft.startContinuous(0);
}

void FCTUC::setupRFID() {
  SPI.begin(); 

  deviceRFIDRight.PCD_Init();
  deviceRFIDLeft.PCD_Init();
}

void FCTUC::setupWifi(){
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    deviceNeoPixel.SetPixelColor(0, RgbColor(0, 0, 100));

    bool on = true;
        while (WiFi.status() != WL_CONNECTED) {
            deviceNeoPixel.SetPixelColor(0, RgbColor(0, 0, on * 100));
            deviceNeoPixel.Show();

            on = !on;
            yield();
            delay(400);
        }
    Serial.print("[INFO] - BotFCTUC's IP is: ");
    Serial.println(WiFi.localIP());

    #ifdef WIFI_MONITORING  
        server = new AsyncServer(21);
        server->onClient(handleTCPConnect, &server);
        server->begin();
    #endif

    udp.begin(1234);
}

#ifdef WIFI_MONITORING
    void FCTUC::handleTCPConnect(void*, AsyncClient *client){
        println("[INFO] - BotFCTUC New TCP connection: " + String(client->remoteIP().toString()));

        static String CONNECT_GREETING = "[INFO] - Connected to FCTUC!\n[INFO] - BotFCTUC's battery level: " + String(getBatteryPercent()) + "%\n";

        delay(5);   //Guarentee that greeting message is only sent after the client had time to be ready to recieve

        for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){
            if(tcpClients[i] == nullptr){
                tcpClients[i] = client;
                tcpClients[i]->onDisconnect(handleTCPDisconnect);
                tcpClients[i]->onData(handleTCPData);

                tcpClients[i]->add(CONNECT_GREETING.c_str(), CONNECT_GREETING.length());
                tcpClients[i]->send();

                break;
            }
        }
    }

    void FCTUC::handleTCPDisconnect(void*, AsyncClient *client){

        for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){

            if(tcpClients[i] == client){
                tcpClients[i] = nullptr;
                break;
            }   
        }
    }

    void FCTUC::handleTCPData(void*, AsyncClient *client, void *dPtr, size_t dLen){
        String msg;

        for(size_t i = 0; i < dLen; i++){

            if( !( i == (dLen - 1) && msg[i] == '\n')){ //prevent trailing '\n' some TCP clients send from appearing in the string

                msg += ((char*) dPtr)[i];
            }
            
        }

        //Robot may be started remotely with a "start"
        if(msg.startsWith("start") ){
            startButtonOverride = true;
        }
    }
#endif

/**
 * @brief Immobilizes the bot.
 */
void FCTUC::stopBot(){

    botEnabled = false;
    moveMotors(0,0);
}

/**
 * @brief Returns true if the RFID readers recently detected a tag.
 * @return True if yes, false if no.
 */
bool FCTUC::getTagDetected() {
    return isTagDetected;
}

/**
 * @brief Get if the last detected tag was successfully read.
 * @return True if successful, false if not successful.
 */
bool FCTUC::getTagReadSuccess() {
    return isTagReadSuccess;
}


/**
 * @brief Get the last successfully read tag position.
 */
Vec2 FCTUC::getRobotPosition() {
    return currentPosition;
}

/**
 * @brief Get the last known thief position.
 */
Vec2 FCTUC::getThiefPosition() {
    return targetPosition;
}

/**
 * @brief Get if the round has finished yet or not.
 */
bool FCTUC::getRoundFinished(){
    return hasRoundFinished;
}

/**
 * @brief Test connection to peripherals.
 */
bool FCTUC::doSelfTest(){
    static constexpr char HexToCharMap[] = "0123456789ABCDEF";

    //Look for I2C devices by scanning witin the expected address range
    static constexpr uint8_t EXPECTED_I2C_DEVICES = 3;

    byte I2CCount = 0;
    byte I2CAddr[EXPECTED_I2C_DEVICES] = {};

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            I2CAddr[I2CCount] = i;
            I2CCount++;
        }
    }

    if(I2CCount < EXPECTED_I2C_DEVICES){
        println("Error during self-test: Only found " + String(I2CCount) +  " I2C devices, expected " + String(EXPECTED_I2C_DEVICES) + " ! :");

        for(uint8_t i = 0; i < I2CCount; i++){
            print(String(i) + ": 0x");
            print(HexToCharMap[I2CAddr[i] & 0xF]);          //print out the first half of the hex (bitwise operator with 1111)
            print(HexToCharMap[(I2CAddr[i] & 0xF0) >> 4]);  //print out the second half of the hex (bitwise operator with 11110000, then shift 4 bits to the right)

            if(I2CAddr[i] == ADDR_LIDAR_LEFT){print(" (Left Lidar)");}
            else if(I2CAddr[i] == ADDR_LIDAR_RIGHT){print(" (Right Lidar)");}
            else if(I2CAddr[i] == ADDR_LIDAR_FRONT){print(" (Front Lidar)");}
            print('\n');
        }

        return false;
    }


    //Test connection to RFID readers by using the MFRC522's built in CRC coprocessor. A timeout most likely means a bad connection.
    //Initially I was just reading the firmware version register, but that proved to be unreliable.

    //
    //
    //  NOT YET WORKING, do CTRL+F in the library header file for "STATUS_CRC_WRONG" and implement checking if the calculated CRC matches what we should recieve
    //
    //

    byte CRCData[2] = {0xFF, 0xFF};

    MFRC522::StatusCode RFIDStat = deviceRFIDLeft.PCD_CalculateCRC(CRCData, 2, CRCData);

    if(RFIDStat != MFRC522::STATUS_OK){
        println("Error during self-test: Failed to communicated with Left RFID Reader!");
        return false;
    }

    RFIDStat = deviceRFIDRight.PCD_CalculateCRC(CRCData, 2, CRCData);

    if(RFIDStat != MFRC522::STATUS_OK){
        println("Error during self-test: Failed to communicated with Right RFID Reader!");
        return false;
    }

    println("[INFO] - BotFCTUC self test pass!");

    return true;
}

/**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 * @return bool - whether the bot was successfuly initialized or not.
 */
bool FCTUC::begin() {

    setupMotors();
    setupNeopixel();

    //If voltage is this low, then the robot likely has it's power switch off and the esp is being powered by USB.
    //Just glow orange and don't allow
    while(getBatteryVoltage() < 0.1f){
        deviceNeoPixel.SetPixelColor(0, RgbColor(70, 50, 0));
        deviceNeoPixel.Show();
        delay(1000);
    }

    setupButton();
    setupBuzzer();
    setupLidar();
    setupRFID();
    setupWifi();

    doSelfTest();


    botEnabled = true;

    xTaskCreatePinnedToCore(taskReadActiveRFIDValue, "TASK_RFID", 1100, NULL, 1, &rfidTaskHandle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskMonitorBatteryValue, "TASK_BATT", 1000, NULL, 1, &batteryTaskHandle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskMonitorTargetPosition, "TASK_POSN", 2000, NULL, 1, &udpTaskHandle, tskNO_AFFINITY);
    #ifdef ENABLE_OTA
        xTaskCreatePinnedToCore(taskHandleOTA, "TASK_OTA", 2000, NULL, 1, &OTATaskHandle, tskNO_AFFINITY);
    #endif

    //Serial.println("[INFO] - BotFCTUC battery voltage: " + String(getBatteryVoltage()) + "v");

    Serial.println("[INFO] - BotFCTUC battery level: " + String(getBatteryPercent()) + "%");

    return true;
}

#ifdef ENABLE_OTA
    /**
     * @brief Enables OTA programming, allowing you to program the robot without needing to plug it in via USB.
     * @param password: the password used for wireless programming. Make sure the upload_flags = --auth=<password> in the platformio.ini file matches this, or it won't work.
     */
    void FCTUC::beginOTA(const String &password){

        ArduinoOTA.setPort(3232);
        ArduinoOTA.setPassword(password.c_str());

        static uint64_t blinkMillis = millis();
        static bool blink = true;

        ArduinoOTA
            .onStart([&]() {
            
                stopBot();
                
                if (ArduinoOTA.getCommand() == U_FLASH)
                    setPixelColor(50, 50, 0);
                else
                    setPixelColor(0, 50, 50);

                    println("[INFO] - BotFCTUC OTA programming started!");
                })
            

            .onEnd([&]() {
                setPixelColor(125, 0, 125);
                delay(200);
                setPixelColor(125, 0, 125);
            })

            //Blink faster and faster until it is done
            .onProgress([](unsigned int progress, unsigned int total) {

                float prog = (float) progress/ (float) total;

                if(millis() - blinkMillis > (700 * (1.0f - prog)) ){
                    blink = !blink;
                    deviceNeoPixel.SetPixelColor(0, RgbColor(25 * blink, 25 * blink, 0));
                    deviceNeoPixel.Show();
                    blinkMillis = millis();
                }
            })

            .onError([&](ota_error_t error) {
                if (error == OTA_AUTH_ERROR) println("OTA Error: Auth Failed"); //Auth Failed
                else if (error == OTA_BEGIN_ERROR) println("OTA Error: Begin Failed"); //Begin failed
                else if (error == OTA_CONNECT_ERROR) println("OTA Error: Connect Error"); //Connect error
                else if (error == OTA_RECEIVE_ERROR) println("OTA Error: Receive Error"); //Receive error
                else if (error == OTA_END_ERROR) println("OTA Error: End Error"); //End Error
            });
    
        ArduinoOTA.begin();
    }
#endif

/**
 * @brief Waits until the button is pressed. This will block execution.
 */
void FCTUC::waitStart() {
    println("[INFO] - BotFCTUC is waiting to start!");

    static uint32_t lastMillis = 0;
    static bool on = false;

    while (!(readButton() || startButtonOverride)) {
        if (millis() - lastMillis >= 500) {
            setPixelColor(0, (on = !on) * 70, 0);
            lastMillis = millis();
        }
        delay(20);
    }

    if(startButtonOverride){
        println("[INFO] - BotFCTUC is starting from remote command!");
        startButtonOverride = false;    //reset in case this method is called again
    }else{
        println("[INFO] - BotFCTUC is starting from button press!");
    }


    setPixelColor(0, 0, 0);
}

/**
  @brief Read the button state.
  @return true if button is pressed, false if not.
  */
// Note that logic is inverted due to PULLUP resistor!
bool FCTUC::readButton() {
    return !digitalRead(PIN_BUTTON);
}

/**
  @brief Play a tone on the buzzer.
  @param frequency Frequency of square wave, in Hz.
 */
void FCTUC::buzzer(uint16_t frequency) {
    tone(PIN_BUZZER, frequency);
}
/**
  @brief Play a tone on the buzzer.
  @param frequency Frequency of square wave, in Hz.
  @param duration Time to play the tone for, in milliseconds.
 */
void FCTUC::buzzer(uint16_t frequency, uint16_t duration) {
    tone(PIN_BUZZER, frequency, duration);
}

/**
  @brief Set the NeoPixel's color.
  @param red value between [0, 255]
  @param green value between [0, 255]
  @param blue value between [0, 255]
 */
void FCTUC::setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    if(!botEnabled)
        return;
        
    deviceNeoPixel.SetPixelColor(0, RgbColor(red, green, blue)); // (pixelID, color)
    deviceNeoPixel.Show();
}


/**
  @brief Control left motor speed.
  @param duty desired duty cycle for the motor, value in the range [-500, 500].
 */

void FCTUC::moveMotorLeft(int16_t duty) {
    if(duty == 0 || !botEnabled){
        digitalWrite(PIN_MOTOR_L_1, HIGH);
        analogWrite(PIN_MOTOR_L_2, DUTY_MOTOR_MAX);
        return;
    }

    duty = constrain(duty + motorOffsets[0], -DUTY_MOTOR_CAP, DUTY_MOTOR_CAP);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_L_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_L_1, HIGH);
    }

    analogWrite(PIN_MOTOR_L_2, abs(duty));
}

/**
  @brief Control right motor speed.
  @param duty desired duty cycle for the motor, value in the range [-500, 500].
 */

void FCTUC::moveMotorRight(int16_t duty) {
    if(duty == 0 || !botEnabled){
        digitalWrite(PIN_MOTOR_R_1, HIGH);
        analogWrite(PIN_MOTOR_R_2, DUTY_MOTOR_MAX);
        return;
    }

    duty = constrain(duty + motorOffsets[1], -DUTY_MOTOR_CAP, DUTY_MOTOR_CAP);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_R_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_R_1, HIGH);
    }

    analogWrite(PIN_MOTOR_R_2, abs(duty));
}

/**
  @brief Control both motors simultaneously.
  @param dutyMotor1Left, dutyMotorRight desired duty cycle for left motor and right motors, values in the range [-500, 500].
 */
void FCTUC::moveMotors(int16_t dutyMotorLeft, int16_t dutyMotorRight) {
    moveMotorLeft(dutyMotorLeft);
    moveMotorRight(dutyMotorRight);
}


/**
  @brief Bring both motors to a stop, this will block execution for 50 ms.
 */
void FCTUC::stopMotors() {   
    moveMotorLeft(0);
    moveMotorRight(0);
    delay(50);
}


/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] (mm)
 */
uint16_t FCTUC::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t FCTUC::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t FCTUC::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Returns the current battery voltage, in volts.
 */
//Voltage divider made from a 1.8k resistor connected to vBat and 1k resistor to GND.
float FCTUC::getBatteryVoltage(){
    constexpr float mult = ((1800.0f + 1000.0f) / 1000.0f) * 3.3f/4095.0f;
    return analogRead(PIN_BAT_SENSE) * mult;
}

/**
 * @brief Returns the current battery level percentage.
 */
float FCTUC::getBatteryPercent(){
    float out = (getBatteryVoltage() - MIN_BAT_VOLTAGE) / (MAX_BAT_VOLTAGE - MIN_BAT_VOLTAGE) * 100.0f;

    out = constrain(out, 0.0f, 100.0f);

    return out;
}

void FCTUC::taskMonitorBatteryValue(void*) {
    while (true) {
        if(getBatteryVoltage() < BAT_CUTOFF_VOLTAGE){
            println("[WARN] - FCTUC's battery has ran out! - Request a new one from a mentor or technical team member!");
            stopBot();

            bool on = true;
            while (true) {
                deviceNeoPixel.SetPixelColor(0, RgbColor(on * 100, 0, 0)); // (pixelID, color)
                deviceNeoPixel.Show();

                on = !on;
                delay(100);
            }

        }
        delay(3000);
    }
}

void FCTUC::taskReadActiveRFIDValue(void*) {
    // We can do this because the objects are preallocated
    static constexpr MFRC522* deviceRFIDSelect[] = {&deviceRFIDRight, &deviceRFIDLeft};
    static MFRC522::MIFARE_Key TAG_KEY = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // public key, only allows read operations

    

    while (true) {
        static bool active = false;
        static unsigned long switchMillis = millis();

        if(millis() - switchMillis > TAG_ANTENNA_SWITCH_MS){
            active = !active;
            switchMillis = millis();
        }
        
        

        // Simple and effective way to toggle active RFID
        deviceRFIDSelect[active]->PCD_AntennaOn();
        deviceRFIDSelect[!active]->PCD_AntennaOff();
        
        // RFID detected a new card, perform read
        isTagDetected = deviceRFIDSelect[active]->PICC_IsNewCardPresent() && deviceRFIDSelect[active]->PICC_ReadCardSerial();

        if (isTagDetected) {
            MFRC522::StatusCode status = MFRC522::STATUS_ERROR;
            byte size = 18; // must be 18 as the 2 extra bytes are used internally by the MFRC522 library for the read operation
            byte buffer[size] = {};

            isTagReadSuccess = false;

            status = deviceRFIDSelect[active]->PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, TAG_BLOCK_POS, &TAG_KEY, &(deviceRFIDSelect[active]->uid)); //authenticate with our public key for reading

            if (status != MFRC522::STATUS_OK) {
                print("[ERROR] - ");
                print(deviceRFIDSelect[active] == &deviceRFIDRight ? "Right" : "Left");
                print(" RFID  Read Fail - PCD_Authenticate() failed: ");
                println(deviceRFIDRight.GetStatusCodeName(status));
                deviceRFIDSelect[active]->PICC_HaltA();
                deviceRFIDSelect[active]->PCD_StopCrypto1();
                continue;
            }

            status = deviceRFIDSelect[active]->MIFARE_Read(TAG_BLOCK_POS, buffer, &size); //try to read block
            
            if (status != MFRC522::STATUS_OK) {
                print("[ERROR] - ");
                print(deviceRFIDSelect[active] == &deviceRFIDRight ? "Right" : "Left");
                print(" RFID  Read Fail - PCD_Authenticate() failed: ");
                println(deviceRFIDRight.GetStatusCodeName(status));                
                deviceRFIDSelect[active]->PICC_HaltA();
                deviceRFIDSelect[active]->PCD_StopCrypto1();
                continue;
            }

            deviceRFIDSelect[active]->PICC_HaltA();
            deviceRFIDSelect[active]->PCD_StopCrypto1();


            currentPosition = Vec2(buffer[0], buffer[1]);

            isTagReadSuccess = true;
        }

        delay(TAG_ANTENNA_READ_MS);
    }
}

void FCTUC::taskMonitorTargetPosition(void*) {
    static char buffer[5] = {0};

    while (true) {
        int packetSize = udp.parsePacket();

        if (packetSize) {
            int length = udp.read(buffer, 5);
            // I'm aware this validation is dodgy, don't worry its just temporary
            //This "temporary" validation has been around for too long, me getting worried
            
            targetPosition = Vec2(buffer[0], buffer[1]);

            if(buffer[2] == 80){
                hasRoundFinished = true;

            }
            //println("X: " + String(targetPosition.x) + " Y: " + String(targetPosition.y));
            //if (buffer[0] + buffer[1] + buffer[2] + buffer[3] == buffer[4]) {  
            //}
        }

        delay(UDP_MSG_CHECK_INTERVAL);
    }
}

#ifdef ENABLE_OTA
    void FCTUC::taskHandleOTA(void*){
        while(true){
            ArduinoOTA.handle();
            delay(1500);
        }
    }
#endif

/**
  @brief Prints a string via serial and via WiFi
 */
void FCTUC::print(const char* str){
    Serial.print(str);

    #ifdef WIFI_MONITORING 
        for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){
            if(tcpClients[i] != nullptr){
                tcpClients[i]->add(str, strlen(str));
                tcpClients[i]->send();
            }
        }
    #endif
}

/**
  @brief Print all detected I2C devices.
 */
void FCTUC::printI2C() {
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1); // maybe unneeded?
        }
    }

    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
}

/**
  @brief Print all LiDAR distance values.
 */
void FCTUC::printLidarValue() {
    uint16_t left = getLidarLeftDistance();
    uint16_t front = getLidarFrontDistance();
    uint16_t right = getLidarRightDistance();

    println(
        "Left : Front : Right - " + String(left) + " : " +  String(front)  + " : " + String(right) + " (mm)"
    );
}

/**
  @brief Print (to serial ONLY) the detected RFID reader firmware versions. Useful for detecting connection issues.
 */
void FCTUC::printRfidPcdFw() {
    Serial.print("Left: ");
    deviceRFIDLeft.PCD_DumpVersionToSerial();
    Serial.print("Right: ");
    deviceRFIDRight.PCD_DumpVersionToSerial();
}


/**
 * @brief Get which walls exist around this cell.
 * @returns A Walls struct, containing data about the existance of a wall on each side of the cell. This struct will return {1,1,1,1} if an invalid position is requested.
 */
Walls GetWallsAtPos(const uint8_t x, const uint8_t y, const uint8_t *lab){

    if(x >= lab[0] || y >= lab[0])  //invalid position
        return {1,1,1,1};


    uint8_t linIndx = x + y * lab[0]; //lab[0] is the side size of the labyrinth. //linear index within the labyrinth. begins at 0 on the top left, going from left to right
                                      //and from top to bottom

    uint8_t byteIndx = (linIndx / 2) + 1; //position of the byte within the array that contains the data for the cell we want to read.

    uint8_t cellByte = (lab[byteIndx] >> (linIndx % 2 ? 0 : 4) ) & 0b1111 ; //get the 4 bits relevant to this cell


    return {(bool) ( (cellByte >> 3) & 1), (bool) ( (cellByte >> 1) & 2), (bool) ( (cellByte >> 1) & 1), (bool) ( (cellByte >> 0) & 1)};
}

/**
 * @brief Get which walls exist around this cell.
 * @returns A Walls struct, containing data about the existance of a wall on each side of the cell.
 */
Walls GetWallsAtPos(const Vec2 pos, const uint8_t *lab){
    return GetWallsAtPos(pos.x, pos.y, lab);
}