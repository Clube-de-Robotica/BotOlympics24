#include <Arduino.h>
#include "FCTUC.h"

FCTUC bot;

void setup() {
    Serial.begin(9600);
    bot.setMotorOffsets(-5,5); // L, R offset aplicado a cada motor 
    bot.begin();

    Serial.println("Teste motores");

    bot.waitStart();
}

void loop() {

    bot.moveMotorLeft(127); // Mover só o motor esquerdo
    delay(1000);
    bot.moveMotorLeft(0);
    delay(1000);
    
    bot.moveMotorRight(127); // Mover só o motor direito
    delay(1000);
    bot.moveMotorRight(0);
    delay(1000);

    

    bot.moveMotors(127, 127); // Mover ambos os motores
    delay(1000);
    bot.stopMotors();
    delay(1000);
}



