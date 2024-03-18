#include <Arduino.h>
#include "BotFCTUC.h"

BotFCTUC jeff;

void setup() {
    Serial.begin(9600);
    Serial.println("Teste lidares");

    jeff.begin();
}

void loop() {
    uint16_t left = jeff.getLidarLeftDistance();
    uint16_t front = jeff.getLidarFrontDistance();
    uint16_t right = jeff.getLidarRightDistance();

    Serial.println(
        "Esquerda: " + String(left) + "mm" + 
        "\nFrente: " + String(front) + "mm" + 
        "\nDireita: " + String(right) + "mm"
    );

    delay(1000);
}
