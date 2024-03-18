#include <Arduino.h>
#include "BotFCTUC.h"

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    jeff.begin(false, false);

    Serial.println("Teste motores");

    jeff.waitStart();
}

void loop() {
    jeff.moveMotor1(127); // Mover só o motor 1
    delay(1000);
    jeff.moveMotor1(0);
    delay(1000);

    jeff.moveMotor2(127); // Mover só o motor 2
    delay(1000);
    jeff.moveMotor2(0);
    delay(1000);

    jeff.moveMotors(127, 127); // Mover ambos os motores
    delay(1000);
    jeff.stopMotors();
    delay(1000);
}