
#include "M5Module_GNSS.h"
#include <M5Unified.h>
#include "MadgwickAHRS.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Wi-Fi credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

// UDP Configuration
WiFiUDP udp;
const char* udpAddress = "255.255.255.255"; // Broadcast address
const int udpPort = 2002;


// BMI270 object
BMI270::BMI270 bmi270;
#define BMI270_SENSOR_ADDR 0x69

// Madgwick filter
Madgwick filter;

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
    TickType_t lastWakeTime = xTaskGetTickCount();
    const int NUM_SAMPLES = 20; // Rolling window size
    static float axBuffer[NUM_SAMPLES] = {0};
    static float ayBuffer[NUM_SAMPLES] = {0};
    static float azBuffer[NUM_SAMPLES] = {0};
    static float gxBuffer[NUM_SAMPLES] = {0};
    static float gyBuffer[NUM_SAMPLES] = {0};
    static float gzBuffer[NUM_SAMPLES] = {0};
    static int bufferIndex = 0;

    unsigned long imuTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();

    while (true) {
        float ax, ay, az, gx, gy, gz;

        IMU_OK = (bmi270.readAcceleration(ax, ay, az) == 1);
        IMU_OK = (bmi270.readGyroscope(gx, gy, gz) == 1);

        if (IMU_OK) {
            ax = ax; // X-axis unchanged
            ay = -ay; // Invert Y-axis
            az = -az; // Invert Z-axis
            gx = gx; // X-axis unchanged
            gy = -gy; // Invert Y-axis
            gz = -gz; // Invert Z-axis

            axBuffer[bufferIndex] = ax; ayBuffer[bufferIndex] = ay; azBuffer[bufferIndex] = az;
            gxBuffer[bufferIndex] = gx; gyBuffer[bufferIndex] = gy; gzBuffer[bufferIndex] = gz;

            bufferIndex = (bufferIndex + 1) % NUM_SAMPLES;

            float axAvg = 0, ayAvg = 0, azAvg = 0;
            float gxAvg = 0, gyAvg = 0, gzAvg = 0;

            for (int i = 0; i < NUM_SAMPLES; i++) {
                axAvg += axBuffer[i]; ayAvg += ayBuffer[i]; azAvg += azBuffer[i];
                gxAvg += gxBuffer[i]; gyAvg += gyBuffer[i]; gzAvg += gzBuffer[i];
            }

            axAvg /= NUM_SAMPLES; ayAvg /= NUM_SAMPLES; azAvg /= NUM_SAMPLES;
            gxAvg /= NUM_SAMPLES; gyAvg /= NUM_SAMPLES; gzAvg /= NUM_SAMPLES;

            filter.updateIMU(gxAvg, gyAvg, gzAvg, axAvg, ayAvg, azAvg);

            lastIMUUpdate = millis();
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

/***********************************************************************
 * @brief  Sends IMU and GPS data via UDP.
 ***********************************************************************/
void UDPTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned long udpTaskLoopCount = 0;
    unsigned long lastFrequencyUpdate = millis();

    while (true) {
        float heel = 0, pitch = 0;
        String gpsRMC = "";

        if (millis() - lastIMUUpdate <= 500) {
            heel = filter.getRoll() + rollOffset;
            pitch = -filter.getPitch() + pitchOffset;
        }

        xSemaphoreTake(gpsMutex, portMAX_DELAY);
        if (millis() - lastGPSUpdate <= 500) {
            gpsRMC = latestGPSMessage;
        }
        xSemaphoreGive(gpsMutex);

        String pitchXDR = makePitchXDR(pitch);
        String heelXDR = makeHeelXDR(heel);
        
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

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(200)); // 200 ms cycle
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




    Serial.begin(115200);
    Serial2.begin(38400, SERIAL_8N1, 18, 17);
    WiFi.begin(ssid, password);

    if (bmi270.init(I2C_NUM_1, BMI270_SENSOR_ADDR)) {
        IMU_OK = true;
    } else {
        IMU_OK = false;
    }

    filter.begin(200.0f);
 
    gpsMutex = xSemaphoreCreateMutex();

    xTaskCreate(IMUTask, "IMU Task", 4096, NULL, 2, &IMUTaskHandle);
    xTaskCreate(GPSTask, "GPS Task", 4096, NULL, 2, &GPSTaskHandle);
    xTaskCreate(UDPTask, "UDP Task", 4096, NULL, 1, &UDPTaskHandle);
    xTaskCreate(DisplayTask, "Display Task", 4096, NULL, 1, &DisplayTaskHandle);
}

void loop() {
    // FreeRTOS handles tasks
}