#include <Arduino.h>
#include "FCTUC.h"

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    jeff.begin(false, false);

    Serial.println("Teste motores");

    jeff.waitStart();
}

void loop() {

    
    jeff.setMotorOffsets(-5,5); // L, R offset aplicado a cada motor 
    jeff.moveMotorLeft(127); // Mover só o motor esquerdo
    delay(1000);
    jeff.moveMotorLeft(0);
    delay(1000);
    
    jeff.moveMotorRight(127); // Mover só o motor direito
    delay(1000);
    jeff.moveMotorRight(0);
    delay(1000);

    

    jeff.moveMotors(127, 127); // Mover ambos os motores
    delay(1000);
    jeff.stopMotors();
    delay(1000);
}



