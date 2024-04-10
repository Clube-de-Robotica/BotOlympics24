#include <Arduino.h>
#include "FCTUC.h"

BotFCTUC jeff;

ColorRGBC color;

void setup() {
    Serial.begin(9600);
    Serial.println("Teste sensor cor e pixel");

    jeff.begin();
}

void loop() {
    
    jeff.setPixelColor(color.r, color.g, color.b);
    jeff.setPixelBrightness(127);
    
    delay(250);
}
