#include <Arduino.h>
#include "BotFCTUC.h"

BotFCTUC jeff;

ColorRGBC color;

void setup() {
    Serial.begin(9600);
    Serial.println("Teste sensor cor e pixel");

    jeff.begin();
}

void loop() {
    color = jeff.getColorValue();
    
    Serial.println(
        "R: " + String(color.r) + 
        ", G: " + String(color.g) + 
        ", B: " + String(color.b) + 
        ", C: " + String(color.c)
    );

    jeff.setPixelColor(color.r, color.g, color.b);
    jeff.setPixelBrightness(127);
    
    delay(250);
}
