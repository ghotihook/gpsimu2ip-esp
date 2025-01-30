/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include <M5CoreS3.h>

void setup()
{
    CoreS3.begin();
    CoreS3.Display.setTextColor(YELLOW);            // Set the font color to yellow.
    CoreS3.Display.setTextSize(2);                  // Set the font size to 2.
    CoreS3.Display.println("M5CoreS3 I2C Tester");  // Print a string on the screen.
    CoreS3.In_I2C.begin();
}


void runScan(m5::I2C_Class &i2c)
{
    int address;
    for (address = 8; address < 0x78; address++) {
        if (i2c.start(address, false, 400000) && i2c.stop()) {
            CoreS3.Display.print(address, HEX);
            CoreS3.Display.print(" ");
            Serial.print(address, HEX);
            Serial.print(" ");
        } else {
            CoreS3.Display.print(".");
            Serial.print(".");
        }

        delay(1);
      }
}

void loop()
{ 
    CoreS3.Display.clear();
    CoreS3.Display.setCursor(0, 0);
    CoreS3.Display.println("In I2C Address [HEX]");
    runScan(CoreS3.In_I2C);
    delay(1000);
  
}