#include <SPI.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include "ADNS9800_LCD.h"

adns_ctrl ac;

void setup() {
    Serial.begin(9600);
    ac.setup();
}

void loop() {
    ac.loop();
}

