#include <Arduino.h>
#include "BotFCTUC.h"

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    Serial.println("BOT FCTUC");
    
    jeff.begin(false, false);
    jeff.waitStart();
}

void loop() {
    //----------------------------------------------------------
    Serial.println("LIDARES");

    jeff.printI2C();

    for (int i = 0; i < 64; i++) {
        jeff.printLidarValue();
        delay(100);
    }
    delay(250);
    //----------------------------------------------------------
    Serial.println("BUZZER");

    jeff.buzzerPlay(255);
    delay(1000);
    jeff.buzzerPlay(0);
    //----------------------------------------------------------
    Serial.println("SENSOR COR");

    ColorRGBC color;

    for (int i = 0; i < 64; i++) {
        color = jeff.getColorValue();
        Serial.println(
            "R: " + String(color.r) + 
            ", G: " + String(color.g) + 
            ", B: " + String(color.b) + 
            ", C: " + String(color.c)
        );
        delay(50);
    }
    //----------------------------------------------------------
    Serial.println("BOTAO");

    for (int i = 0; i < 255; i++) {
        Serial.println("Botão: " + String(jeff.readButton() ? "Pressionado!" : "OFF"));
        delay(50);
    }
    //----------------------------------------------------------
    Serial.println("MOTORES");

    for (int i = 0; i < 512; i++) {
        jeff.moveMotors(i - 255, i - 255);
        delay(50);
    }
    //----------------------------------------------------------
}