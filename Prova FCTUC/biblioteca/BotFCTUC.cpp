#include "BotFCTUC.h"

NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2811Method> BotFCTUC::deviceNeoPixel(1, PIN_NEOPIXEL);
BotFCTUC *BotFCTUC::botRef;
AsyncServer *BotFCTUC::server;
MFRC522 BotFCTUC::deviceRFIDL(PIN_RFID_SDA_L, MFRC522::UNUSED_PIN);
MFRC522 BotFCTUC::deviceRFIDR(PIN_RFID_SDA_R, MFRC522::UNUSED_PIN);
constexpr char BotFCTUC::HexToCharMap[16] ;

/**
 * @brief Set offsets for the motors to ensure their speeds are consistent.
 * @param R, L The offsets applied to each of the motors. These values may be negative.
 */
void BotFCTUC::setMotorOffsets(int8_t R, int8_t L){
    motorOffsets[0] = R;
    motorOffsets[1] = L;
}

void BotFCTUC::setupButton() {
    pinMode(PIN_BUTTON, INPUT);
}

void BotFCTUC::setupBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
}

void BotFCTUC::setupNeopixel() {
    deviceNeoPixel.Begin();
}


void BotFCTUC::setupMotors(bool shouldInvertMotorL, bool shouldInvertMotorR) {
    invertDir[0] = shouldInvertMotorL;
    invertDir[1] = shouldInvertMotorR;

    pinMode(PIN_MOTOR_L_1, OUTPUT);
    pinMode(PIN_MOTOR_L_2, OUTPUT);
    pinMode(PIN_MOTOR_R_1, OUTPUT);
    pinMode(PIN_MOTOR_R_2, OUTPUT);

    analogWriteResolution(PWM_RESOLUTION_BITS);

}

void BotFCTUC::setupLidar() {
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

void BotFCTUC::setupRFID() {
  SPI.begin(); 

  RFIDPtr = &deviceRFIDL;

  deviceRFIDR.PCD_Init();
  deviceRFIDL.PCD_Init();
 
}

void BotFCTUC::setupWifi(){
    WiFi.begin(WIFI_SSID, WIFI_PW);
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

    server = new AsyncServer(21);
    server->onClient(handleTCPConnect, &server);
    server->begin();
}

void BotFCTUC::handleTCPConnect(void*, AsyncClient *client){
    botRef->println("[INFO] - BotFCTUC New TCP connection: " + String(client->remoteIP().toString()));

    static String CONNECT_GREETING = "[INFO] - Connected to BotFCTUC!\n";

    for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){
        if(botRef->tcpClients[i] == nullptr){
            botRef->tcpClients[i] = client;
            client->onDisconnect(handleTCPDisconnect);
            client->onData(handleTCPData);

            client->add(CONNECT_GREETING.c_str(), CONNECT_GREETING.length());
            client->send();

            break;
        }
    }
}

void BotFCTUC::handleTCPDisconnect(void*, AsyncClient *client){

    for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){

        if(botRef->tcpClients[i] == client){
            botRef->tcpClients[i] = nullptr;
            break;
        }   
    }
}

void BotFCTUC::handleTCPData(void*, AsyncClient *client, void *dPtr, size_t dLen){
    String msg;

    for(size_t i = 0; i < dLen; i++){

        if( !( i == (dLen - 1) && msg[i] == '\n')){ //prevent trailing '\n' some TCP clients send from appearing in the string

            msg += ((char*) dPtr)[i];
        }
        
    }

    //Robot may be started remotely with a "start" or "Start"
    if(msg.startsWith("start") ){
        botRef->startButtonOverride = true;
    }

    //botRef->println(msg.length());
}

//Loop for us to use for our needs, independent of the loop the participants will use.
//If you get crashes from hitting the stack canary, increase the task's stack size in BotFCTUC::begin(bool,bool)
void BotFCTUC::BotLoop(){
    while(true){
        
        ArduinoOTA.handle();


        if(millis() - antennaSwitchMillis > TAG_ANTENNA_SWITCH_MS && botOperational){

            if(RFIDPtr != &deviceRFIDR){    //enable right antenna
                RFIDPtr = &deviceRFIDR;
                deviceRFIDR.PCD_AntennaOn();
                deviceRFIDL.PCD_AntennaOff();

            }else{  //enable left antenna
            RFIDPtr = &deviceRFIDL;
                deviceRFIDL.PCD_AntennaOn();
                deviceRFIDR.PCD_AntennaOff();
            }
            antennaSwitchMillis = millis();
        }

        //Battery has ran too low, lock out robot.
        if(getBatVoltage() < MIN_BAT_VOLTAGE){
            StopBot();
            println("[ERROR] - BotFCTUC's battery has ran out! - Request a new one from a mentor or technical team member!");
            

            bool on = true;  

            while (true) {
                deviceNeoPixel.SetPixelColor(0, RgbColor(on * 50, 0, 0));
                deviceNeoPixel.Show();

                on = !on;
                delay(100);
            }
        }

       delay(10);
    }
}

/**
 * @brief Immobilizes the bot and stops the default Loop() task (which the participants will be using).
 */
void BotFCTUC::StopBot(){
    botOperational = false;
    moveMotors(0,0);
    vTaskDelete(mainLoopTask);
}

/**
 * @brief Test connection to peripherals
 */
bool BotFCTUC::SelfTest(){

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

    MFRC522::StatusCode RFIDStat = deviceRFIDL.PCD_CalculateCRC(CRCData, 2, CRCData);

    if(RFIDStat != MFRC522::STATUS_OK){
        println("Error during self-test: Failed to communicated with Left RFID Reader!");
        return false;
    }

    RFIDStat = deviceRFIDR.PCD_CalculateCRC(CRCData, 2, CRCData);

    if(RFIDStat != MFRC522::STATUS_OK){
        println("Error during self-test: Failed to communicated with Right RFID Reader!");
        return false;
    }

    println("Self test pass!");
    return true;
}

/**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 * @param shouldInvertMotorL invert left motor if it spins the wrong way
 * @param shouldInvertMotorR invert right motor if it spins the wrong way
 * @return bool - whether the motor was successfuly initialized or not
 */
bool BotFCTUC::begin(bool shouldInvertMotorL, bool shouldInvertMotorR) {

    //Only one instance can be active due to static tasks
    if(botRef != nullptr)
        return false;
    
    botRef = this;

    //used to terminate the default arduino loop task when the battery runs out
    mainLoopTask = xTaskGetCurrentTaskHandle();

    setupMotors(shouldInvertMotorL, shouldInvertMotorR);
    setupNeopixel();

    //if bat voltage is too low, block robot and blink red to indicate a new battery is required
    if(getBatVoltage() < MIN_BAT_VOLTAGE){
        Serial.println("[ERROR] - BotFCTUC Battery voltage too low! Request a new battery from a mentor or technical team member!");
        StopBot();

        bool on = true;
        while (true) {
            deviceNeoPixel.SetPixelColor(0, RgbColor(on * 50, 0, 0));
            deviceNeoPixel.Show();

            on = !on;
            yield();
            delay(300);
        }
    }

    setupButton();
    setupBuzzer();
    setupLidar();
    setupRFID();
    setupWifi();

    SelfTest();
                                            //    \/ - Stack size allocated to task, increase here if you hit the stack canary in the BotLoop() task
    xTaskCreatePinnedToCore(BotLoop, "Bot Loop", 2000, this, 1, &botLoopHandle, tskNO_AFFINITY);

    botOperational = true;
    
    Serial.println("[INFO] - BotFCTUC battery voltage: " + String(getBatVoltage()) + "v");

    return true;
}

/**
 * @brief Enables OTA programming, allowing you to program the robot without needing to plug it in via USB.
 * @param password: the password used for wireless programming. Make sure the upload_flags = --auth=<password> in the platformio.ini file matches this, or it won't work.
 */
void BotFCTUC::beginOTA(const String &password){

    ArduinoOTA.setPort(3232);
    ArduinoOTA.setPassword(password.c_str());

    ArduinoOTA
        .onStart([&]() {
        
            StopBot();
            
            if (ArduinoOTA.getCommand() == U_FLASH)
                setPixelColor(50, 50, 0);
            else
                setPixelColor(0, 50, 50);

                botOperational = false;
                otaInProgress = true;

                println("[INFO] - BotFCTUC OTA programming started!");
            })
          

        .onEnd([&]() {
            otaInProgress = false;
            setPixelColor(125, 0, 125);
            delay(200);
            setPixelColor(125, 0, 125);
        })

        //Blink faster and faster until it is done
        .onProgress([](unsigned int progress, unsigned int total) {

            static uint64_t blinkMillis = millis();
            static bool blink = true;

            float prog = (float) progress/total;

            if(millis() - blinkMillis > 800 * (1.0f - prog)){
                blink = !blink;
                deviceNeoPixel.SetPixelColor(0, RgbColor(25 * blink, 25 * blink, 0));
                deviceNeoPixel.Show();
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

/**
 * @brief Waits until the button is pressed. This will block execution.
 */
void BotFCTUC::waitStart() {
    println("[INFO] - BotFCTUC is waiting to start!");

    static uint32_t lastMillis = 0;
    static bool enabled = false;

    while (!digitalRead(PIN_BUTTON) && !startButtonOverride ) {
        if (millis() - lastMillis >= 500) {
            enabled = !enabled;
            lastMillis = millis();

            setPixelColor(0, (enabled) * 70, 0);
        }
    }

    if(startButtonOverride){
        println("[INFO] - BotFCTUC is starting from remote command!");
    }else{
        println("[INFO] - BotFCTUC is starting from button press!");
    }

    startButtonOverride = false;    //reset in case this method is called again

    setPixelColor(0, 0, 0);
}

/**
  @brief Read the button state.
  @return true if button is pressed, false if not
  */
bool BotFCTUC::readButton() {
    return digitalRead(PIN_BUTTON);
}

/**
  @brief Play a tone on the buzzer.
  @param tone Frequency of square wave, in Hz.
 */
void BotFCTUC::buzzer(uint16_t freq) {
    tone(PIN_BUZZER, freq);
}
/**
  @brief Play a tone on the buzzer.
  @param tone Frequency of square wave, in Hz
  @param dur Time to play the tone for, in milliseconds 
 */
void BotFCTUC::buzzer(uint16_t freq, uint16_t dur) {
    tone(PIN_BUZZER, freq, dur);
}

/**
  @brief Set the NeoPixel's color.
  @param red value between [0, 255]
  @param green value between [0, 255]
  @param blue value between [0, 255]
 */
void BotFCTUC::setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    //don't allow other things to mess with the neopixel when it should be showing upload progress
    if(!botOperational)
        return;

    deviceNeoPixel.SetPixelColor(0, RgbColor(red, green, blue)); // (pixelID, color)
    deviceNeoPixel.Show();
}



/**
  @brief Control left motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void BotFCTUC::moveMotorL(int16_t duty) {

    if(duty != 0 && botOperational){
        duty += motorOffsets[0];
    }else{
        digitalWrite(PIN_MOTOR_L_1, HIGH);
        digitalWrite(PIN_MOTOR_L_2, HIGH);
   }

    duty = constrain(duty, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (invertDir[0]) {
        if (duty < 0) {
            digitalWrite(PIN_MOTOR_L_1, LOW);
            analogWrite(PIN_MOTOR_L_2, abs(duty));
        } else {
            duty = DUTY_MOTOR_MAX - duty;
            digitalWrite(PIN_MOTOR_L_1, HIGH);
            analogWrite(PIN_MOTOR_L_2, duty);
        }
    } else {
        if (duty > 0) {
            digitalWrite(PIN_MOTOR_L_1, LOW);
            analogWrite(PIN_MOTOR_L_2, duty);
        }  else {
            duty = DUTY_MOTOR_MAX + duty;
            digitalWrite(PIN_MOTOR_L_1, HIGH);
            analogWrite(PIN_MOTOR_L_2, duty);
        }
    }
}

/**
  @brief Control right motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void BotFCTUC::moveMotorR(int16_t duty) {

    if(duty != 0 && botOperational){
        duty += motorOffsets[1];
    }else{
        digitalWrite(PIN_MOTOR_R_1, HIGH);
        digitalWrite(PIN_MOTOR_R_2, HIGH);
   }

    duty = constrain(duty, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (invertDir[1]) {
        if (duty < 0) {
            digitalWrite(PIN_MOTOR_R_1, LOW);
            analogWrite(PIN_MOTOR_R_2, abs(duty));
        } else {
            duty = DUTY_MOTOR_MAX - duty;
            digitalWrite(PIN_MOTOR_R_1, HIGH);
            analogWrite(PIN_MOTOR_R_2, duty);
        }
    } else {
        if (duty > 0) {
            digitalWrite(PIN_MOTOR_R_1, LOW);
            analogWrite(PIN_MOTOR_R_2, duty);
        }  else {
            duty = DUTY_MOTOR_MAX + duty;
            digitalWrite(PIN_MOTOR_R_1, HIGH);
            analogWrite(PIN_MOTOR_R_2, duty);
        }
    }
}


/**
  @brief Control both motors simultaneously.
  @param dutyMotor1 desired duty cycle for left motor, value between [-511, 511]
  @param dutyMotor2 desired duty cycle for right motor, value between [-511, 511]
 */
void BotFCTUC::moveMotors(int16_t dutyMotorL, int16_t dutyMotorR) {
    moveMotorL(dutyMotorL);
    moveMotorR(dutyMotorR);
}


/**
  @brief Bring both motors to a stop, this will block execution for 50 ms.
 */
void BotFCTUC::stopMotors() {   
    moveMotorL(0);
    moveMotorR(0);
    delay(50);
}


/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] (mm)
 */
uint16_t BotFCTUC::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotFCTUC::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotFCTUC::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Returns the current battery voltage, in volts.
 */
//Voltage divider made from a 1.8k resistor connected to vBat and 1k resistor to GND.
float BotFCTUC::getBatVoltage(){
    constexpr float mult = ((1800.0f + 1000.0f) / 1000.0f) * 3.3f/4095.0f;
    return analogRead(PIN_BAT_SENSE) * mult;
}

/**
 * @brief Get if a new position tag has been detected. **Note: due to our double reader setup this may return true more than once in the same position**
 * @return True if yes, false if no
 */
bool BotFCTUC::tagDetected(){
    if(!botOperational)
        return false;

    if(RFIDPtr->PICC_IsNewCardPresent() && RFIDPtr->PICC_ReadCardSerial()){
        //println(RFIDPtr == &deviceRFIDR ? "Detect on right" : "Detect on left");
        antennaSwitchMillis = millis(); //Reset antenna switch timer so we don't turn off antenna mid read
        return true;
    }

    return false;
}

/**
 * @brief Attept to read the current position from the tag. Returns an invalid position if failed.
 * Make sure the robot is over a tag with tagDetected(), or else it will fail.
 */
Position BotFCTUC::readPosition(){
    byte size = 18;    //must be 18 as the 2 extra bytes are used internally by the MFRC522 library for the read operation
    byte readBuffer[size] = {};
    Position output = {false, 0, 0};
    MFRC522::StatusCode stat = MFRC522::STATUS_ERROR;

    if(!botOperational)
        return output;


    stat = RFIDPtr->PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, TAG_BLOCK_POS, &TAG_KEY, &(RFIDPtr->uid)); //authenticate with our public key for reading

    if(stat != MFRC522::STATUS_OK){ //failed to auth
        print("[ERROR] - ");
        print(RFIDPtr == &deviceRFIDR ? "Right" : "Left");
        print(" RFID  Read Fail - PCD_Authenticate() failed: ");
        println(deviceRFIDR.GetStatusCodeName(stat));

        RFIDPtr->PICC_HaltA();
        RFIDPtr->PCD_StopCrypto1();

        return output;
    }

    stat = RFIDPtr->MIFARE_Read(TAG_BLOCK_POS, readBuffer, &size); //try to read block

    if(stat != MFRC522::STATUS_OK){ //failed to auth
        print("[ERROR] - ");
        print(RFIDPtr == &deviceRFIDR ? "Right" : "Left");
        print(" RFID Read Fail - MIFARE_Read() failed: ");
        println(deviceRFIDR.GetStatusCodeName(stat));

        RFIDPtr->PICC_HaltA();
        RFIDPtr->PCD_StopCrypto1();

        return output;
    }

    RFIDPtr->PICC_HaltA();
    RFIDPtr->PCD_StopCrypto1();

    output.isValid = true;
    output.x = readBuffer[0];
    output.y = readBuffer[1];

    return output;
}


/**
  @brief Prints a string via serial and via WiFi
 */
void BotFCTUC::print(const char* str){
    Serial.print(str);

    for(uint8_t i = 0; i < MAX_TCP_CLIENTS; i++){
        if(tcpClients[i] != nullptr){
            tcpClients[i]->add(str, strlen(str));
            tcpClients[i]->send();
        }
    }
}


/**
  @brief Print all detected I2C devices.
 */
void BotFCTUC::printI2C() {
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
void BotFCTUC::printLidarValue() {
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
void BotFCTUC::printRfidPcdFw() {
    Serial.print("Left: ");
    deviceRFIDL.PCD_DumpVersionToSerial();
    Serial.print("Right: ");
    deviceRFIDR.PCD_DumpVersionToSerial();
}