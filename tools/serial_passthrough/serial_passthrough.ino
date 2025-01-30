/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include <M5CoreS3.h>  // For M5Stack Core S3

// Serial port definitions
#define GPS_RX 18  // GPS RX pin
#define GPS_TX 17  // GPS TX pin

void setup() {
    // Initialize M5Stack Core S3
    M5.begin();
    M5.Lcd.setBrightness(200);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("GPS Passthrough App");
    
    // Initialize Serial (USB) for communication with the PC
    Serial.begin(38400);
    while (!Serial) {
        delay(10); // Wait for Serial to be ready
    }
    Serial.println("Ready to connect to GPS");

    // Initialize Serial2 for GPS communication
    Serial2.begin(38400, SERIAL_8N1, GPS_RX, GPS_TX); // Default NEO-M9N configuration
}

void loop() {
    // Pass data from GPS (Serial2) to PC (Serial)
    if (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    // Pass data from PC (Serial) to GPS (Serial2)
    if (Serial.available()) {
        Serial2.write(Serial.read());
    }
}