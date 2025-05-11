
#include <Wire.h>
#include "M5Module_GNSS.h"
#include <M5Unified.h>
#include "MadgwickAHRS.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "secrets.h"

// UDP Configuration
WiFiUDP udp;
const char* udpAddress = "255.255.255.255"; // Broadcast address
const int udpPort = 2002;


// BMI270 object - Either 0x68 or 0x69
BMI270::BMI270 bmi270;
#define BMI270_SENSOR_ADDR 0x69



// Status flags
bool GPS_OK = false;
bool IMU_OK = false;
bool UDP_OK = false;
bool WIFI_OK = false;

// IMU offsets
float rollOffset = 0;
float pitchOffset = 0;

// Shared GPS data
String latestGPSMessage = "";
SemaphoreHandle_t gpsMutex;


// Global averaged pitch/roll values
float globalAvgPitch = 0.0;
float globalAvgRoll  = 0.0;
SemaphoreHandle_t imuDataMutex;


// Data freshness tracking
unsigned long lastGPSUpdate = 0;
unsigned long lastIMUUpdate = 0;

// Task frequencies
float imuFrequency = 0, gpsFrequency = 0, udpFrequency = 0, displayFrequency = 0;

// FreeRTOS Task Handles
TaskHandle_t IMUTaskHandle, GPSTaskHandle, UDPTaskHandle, DisplayTaskHandle;



/***********************************************************************
 * @brief  Creates a heel and pitch NMEA sentence (XDR format).
 ***********************************************************************/
String makeHeelXDR(float heel) {
    char xdrSentence[100];
    uint8_t checksum = 0;
    snprintf(xdrSentence, sizeof(xdrSentence), "IIXDR,A,%.0f,D,HEEL", heel);
    
    for (int i = 0; xdrSentence[i] != '\0'; i++) {
        checksum ^= xdrSentence[i]; // XOR each character
    }
    char fullXDRSentence[110];
    snprintf(fullXDRSentence, sizeof(fullXDRSentence), "$%s*%02X\n", xdrSentence, checksum);
    return String(fullXDRSentence);
}


String makePitchXDR(float pitch) {
    char xdrSentence[100];
    uint8_t checksum = 0;
    snprintf(xdrSentence, sizeof(xdrSentence), "IIXDR,A,%.0f,D,PITCH",pitch);
    
    for (int i = 0; xdrSentence[i] != '\0'; i++) {
        checksum ^= xdrSentence[i]; // XOR each character
    }
    char fullXDRSentence[110];
    snprintf(fullXDRSentence, sizeof(fullXDRSentence), "$%s*%02X\n", xdrSentence, checksum);
    return String(fullXDRSentence);
}
/***********************************************************************
 * @brief  Reads IMU data and updates the Madgwick filter.
 ***********************************************************************/
void IMUTask(void* pvParameters) {
    // Create a local instance of the filter
    Madgwick localFilter;
    localFilter.begin(100.0f);

    // Rolling buffers for pitch and roll averaging
    const int NUM_AVG_SAMPLES = 5;
    static float pitchBuffer[NUM_AVG_SAMPLES] = {0};
    static float rollBuffer[NUM_AVG_SAMPLES] = {0};
    static int bufferIndex = 0;

    unsigned long imuTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        float ax, ay, az, gx, gy, gz;

        // Read sensor values directly from the BMI270
        IMU_OK = (bmi270.readAcceleration(ax, ay, az) == 1);
        IMU_OK = (bmi270.readGyroscope(gx, gy, gz) == 1);

        if (IMU_OK) {
            // Adjust axis values if necessary
            ax = ax;            // X-axis unchanged
            ay = -ay;           // Invert Y-axis
            az = -az;           // Invert Z-axis
            gx = gx;            // X-axis unchanged
            gy = -gy;           // Invert Y-axis
            gz = -gz;           // Invert Z-axis

            // Update the filter with the raw sensor data
            localFilter.updateIMU(gx, gy, gz, ax, ay, az);

            // Retrieve filtered pitch and roll (apply any offsets if needed)
            float currentRoll  = localFilter.getRoll() + rollOffset;
            float currentPitch = -localFilter.getPitch() + pitchOffset;

            // Add the filtered values to the rolling buffer
            pitchBuffer[bufferIndex] = currentPitch;
            rollBuffer[bufferIndex]  = currentRoll;

            // Compute rolling averages for pitch and roll
            float sumPitch = 0, sumRoll = 0;
            for (int i = 0; i < NUM_AVG_SAMPLES; i++) {
                sumPitch += pitchBuffer[i];
                sumRoll  += rollBuffer[i];
            }
            float avgPitch = sumPitch / NUM_AVG_SAMPLES;
            float avgRoll  = sumRoll / NUM_AVG_SAMPLES;

            // Update the global averaged values safely
            xSemaphoreTake(imuDataMutex, portMAX_DELAY);
            globalAvgPitch = avgPitch;
            globalAvgRoll  = avgRoll;
            xSemaphoreGive(imuDataMutex);

            lastIMUUpdate = millis();
            // Move to the next index in the rolling buffer
            bufferIndex = (bufferIndex + 1) % NUM_AVG_SAMPLES;
        }

        imuTaskLoopCount++;
        if (millis() - lastFrequencyUpdate >= 1000) {
            imuFrequency = imuTaskLoopCount;
            imuTaskLoopCount = 0;
            lastFrequencyUpdate = millis();
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10)); // 10 ms cycle
    }
}

/***********************************************************************
 * @brief  Reads NMEA messages from the GPS module.
 ***********************************************************************/
/*
void GPSTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned long gpsTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();

    while (true) {
        static String nmeaMessage = "";
        static bool messageStarted = false;

        while (Serial2.available() > 0) {
            char c = Serial2.read();

            if (!messageStarted && c == '$') {
                messageStarted = true;
                nmeaMessage = c;
            } else if (messageStarted) {
                nmeaMessage += c;
                if (c == '\n') {
                    messageStarted = false;

                    if (nmeaMessage.length() > 20) {
                        xSemaphoreTake(gpsMutex, portMAX_DELAY);
                        latestGPSMessage = nmeaMessage;
                        xSemaphoreGive(gpsMutex);
                        GPS_OK = true;
                        lastGPSUpdate = millis();
                    } else {
                        GPS_OK = false;
                    }
                    break;
                }
            }
        }

        if (millis() - lastGPSUpdate > 1000) {
            GPS_OK = false;
        }

        gpsTaskLoopCount++;
        if (millis() - lastFrequencyUpdate >= 1000) {
            gpsFrequency = gpsTaskLoopCount;
            gpsTaskLoopCount = 0;
            lastFrequencyUpdate = millis();
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(200)); // 20 ms cycle
    }
}
*/
void GPSTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned long gpsTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();

    while (true) {
        static String nmeaMessage = "";
        static bool messageStarted = false;

        while (Serial2.available() > 0) {
            char c = Serial2.read();

            if (!messageStarted && c == '$') {
                messageStarted = true;
                nmeaMessage = c;
            } else if (messageStarted) {
                nmeaMessage += c;
                if (c == '\n') {
                    messageStarted = false;
                    
                    // Only process the message if its length is sufficient and it starts with "$GNRMC"
                    if (nmeaMessage.length() > 20 && nmeaMessage.startsWith("$GNRMC")) {
                        xSemaphoreTake(gpsMutex, portMAX_DELAY);
                        latestGPSMessage = nmeaMessage;
                        xSemaphoreGive(gpsMutex);
                        GPS_OK = true;
                        lastGPSUpdate = millis();
                    } else {
                        GPS_OK = false;
                    }
                    
                    // Clear the message for the next sentence.
                    nmeaMessage = "";
                    break;
                }
            }
        }

        if (millis() - lastGPSUpdate > 1000) {
            GPS_OK = false;
        }

        gpsTaskLoopCount++;
        if (millis() - lastFrequencyUpdate >= 1000) {
            gpsFrequency = gpsTaskLoopCount;
            gpsTaskLoopCount = 0;
            lastFrequencyUpdate = millis();
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50)); // 200 ms cycle
    }
}
/***********************************************************************
 * @brief  Sends IMU and GPS data via UDP.
 ***********************************************************************/
void UDPTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned long udpTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();

    while (true) {
        float pitch = 0, heel = 0;
        String gpsRMC = "";

        // If the IMU data is fresh, retrieve the rolling average values safely
        if (millis() - lastIMUUpdate <= 500) {
            xSemaphoreTake(imuDataMutex, portMAX_DELAY);
            pitch = globalAvgPitch;
            heel  = globalAvgRoll;
            xSemaphoreGive(imuDataMutex);
        }

        // Get the latest GPS message (with the existing GPS mutex)
        xSemaphoreTake(gpsMutex, portMAX_DELAY);
        if (millis() - lastGPSUpdate <= 500) {
            gpsRMC = latestGPSMessage;
        }
        xSemaphoreGive(gpsMutex);

        // Create NMEA sentences using the averaged values
        String pitchXDR = makePitchXDR(pitch);
        String heelXDR  = makeHeelXDR(heel);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.print(pitchXDR);
        UDP_OK = (udp.endPacket() == 1);
        
        udp.beginPacket(udpAddress, udpPort);
        udp.print(heelXDR);
        UDP_OK = (udp.endPacket() == 1);

        udp.beginPacket(udpAddress, udpPort);
        udp.print(gpsRMC);
        UDP_OK = (udp.endPacket() == 1);

        udpTaskLoopCount++;
        if (millis() - lastFrequencyUpdate >= 1000) {
            udpFrequency = udpTaskLoopCount;
            udpTaskLoopCount = 0;
            lastFrequencyUpdate = millis();
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100)); // 100 ms cycle
    }
}

/***********************************************************************
 * @brief  Updates the M5Stack LCD screen.
 ***********************************************************************/

void DisplayTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned long displayTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();
    
    while (true) {
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(2);

        // GPS Status
        M5.Lcd.setTextColor(GPS_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("GPS : %s (%.0f Hz)\n", GPS_OK ? "OK" : "NOK", gpsFrequency);

        // IMU Status
        M5.Lcd.setTextColor(IMU_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("IMU : %s (%.0f Hz)\n", IMU_OK ? "OK" : "NOK", imuFrequency);

        // WiFi Status
        WIFI_OK = (WiFi.status() == WL_CONNECTED);
        M5.Lcd.setTextColor(WIFI_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("WiFi: %s (%s)\n", WIFI_OK ? "OK" : "NOK", WiFi.localIP().toString().c_str());

        // UDP Status
        M5.Lcd.setTextColor(UDP_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("NMEA: %s (%.0f Hz)\n", UDP_OK ? "OK" : "NOK", udpFrequency);

        M5.Lcd.setCursor(0, 150); 
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.print("<-FWD ");   
        M5.Lcd.println("^STB"); 
       
        displayTaskLoopCount++;
        if (millis() - lastFrequencyUpdate >= 1000) {
            displayFrequency = displayTaskLoopCount;
            displayTaskLoopCount = 0;
            lastFrequencyUpdate = millis();
        }

        // Delay inside the loop to let other tasks run and reset the watchdog
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
    }
}
//bool reinitializeI2CAndSensor() {
//    Wire.end(); // Stop the I2C bus
//    delay(100); // Short delay to ensure the bus is idle
//    Wire.begin(); // Restart the I2C bus
//    Wire.setClock(400000); // Reconfigure clock speed to 400kHz
    
    // Reinitialize the BMI270 sensor
//    if (bmi270.init(I2C_NUM_1, BMI270_SENSOR_ADDR)) {
//        IMU_OK = true;
//        return true;
//    } else {
//        IMU_OK = false;
//        return false;
//    }
//}

/***********************************************************************
 * @brief  Initializes the system.
 ***********************************************************************/
void setup() {
    
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.external_imu = true;
    M5.begin(cfg);
    M5.Lcd.setTextSize(2);

    //Wire.begin();
    //Wire.setClock(1000000); // Try 1 MHz, if supported by sensor


    Serial.begin(115200);
    Serial2.begin(38400, SERIAL_8N1, 18, 17);
    connect_to_wifi();

    if (bmi270.init(I2C_NUM_1, BMI270_SENSOR_ADDR)) {
        IMU_OK = true;
    } else {
        IMU_OK = false;
    }


    gpsMutex = xSemaphoreCreateMutex();
    imuDataMutex = xSemaphoreCreateMutex(); 


    xTaskCreate(IMUTask, "IMU Task", 4096, NULL, 2, &IMUTaskHandle);
    xTaskCreate(GPSTask, "GPS Task", 4096, NULL, 2, &GPSTaskHandle);
    xTaskCreate(UDPTask, "UDP Task", 4096, NULL, 1, &UDPTaskHandle);
    xTaskCreate(DisplayTask, "Display Task", 4096, NULL, 1, &DisplayTaskHandle);
}

void connect_to_wifi() {
    WiFi.mode(WIFI_STA);  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        dots = (dots + 1) % 4;
        String wifiStatus = "Connecting to WiFi";
        wifiStatus += (dots == 0 ? "" : (dots == 1 ? "." : (dots == 2 ? ".." : "...")));
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.print(wifiStatus);
        vTaskDelay(pdMS_TO_TICKS(500));
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextColor(BLACK);
        M5.Lcd.print(wifiStatus);      
    }
}





void loop() {
    // FreeRTOS handles tasks
}
