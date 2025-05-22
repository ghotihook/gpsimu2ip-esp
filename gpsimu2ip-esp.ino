#include <Wire.h>

#include <M5Unified.h>
#include "MadgwickAHRS.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "secrets.h"
#include <SparkFun_BMI270_Arduino_Library.h>  // SparkFun BMI270

// UDP Configuration
WiFiUDP udp;
const char* udpAddress = "255.255.255.255"; // Broadcast address
const int   udpPort    = 2002;

// SparkFun BMI270 object
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_SEC_ADDR;    // 0x69 on your board

// Madgwick filter
Madgwick filter;

// Status flags
bool GPS_OK    = false;
bool IMU_OK    = false;
bool UDP_OK    = false;
bool WIFI_OK   = false;

// IMU offsets
float rollOffset  = 0;
float pitchOffset = 0;

// Shared GPS data
String latestGPSMessage;
SemaphoreHandle_t gpsMutex;

// Data freshness tracking
unsigned long lastGPSUpdate = 0;
unsigned long lastIMUUpdate = 0;

// Task frequencies
float imuFrequency     = 0;
float gpsFrequency     = 0;
float udpFrequency     = 0;
float displayFrequency = 0;

// FreeRTOS Task Handles
TaskHandle_t IMUTaskHandle,
            GPSTaskHandle,
            UDPTaskHandle,
            DisplayTaskHandle;

/***********************************************************************
 * @brief  Creates a heel NMEA sentence (XDR format).
 ***********************************************************************/
String makeHeelXDR(float heel) {
    char xdr[100];
    uint8_t cs = 0;
    snprintf(xdr, sizeof(xdr), "IIXDR,A,%.0f,D,HEEL", heel);
    for (int i = 0; xdr[i]; i++) cs ^= xdr[i];
    char out[110];
    snprintf(out, sizeof(out), "$%s*%02X\n", xdr, cs);
    return String(out);
}

/***********************************************************************
 * @brief  Creates a pitch NMEA sentence (XDR format).
 ***********************************************************************/
String makePitchXDR(float pitch) {
    char xdr[100];
    uint8_t cs = 0;
    snprintf(xdr, sizeof(xdr), "IIXDR,A,%.0f,D,PITCH", pitch);
    for (int i = 0; xdr[i]; i++) cs ^= xdr[i];
    char out[110];
    snprintf(out, sizeof(out), "$%s*%02X\n", xdr, cs);
    return String(out);
}

/***********************************************************************
 * @brief  Reads IMU data, runs Madgwick, and tracks frequency.
 ***********************************************************************/
void IMUTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const int   NUM_SAMPLES = 10;
    static float axBuf[NUM_SAMPLES] = {0},
                 ayBuf[NUM_SAMPLES] = {0},
                 azBuf[NUM_SAMPLES] = {0},
                 gxBuf[NUM_SAMPLES] = {0},
                 gyBuf[NUM_SAMPLES] = {0},
                 gzBuf[NUM_SAMPLES] = {0};
    static int idx = 0;
    unsigned long loopCount        = 0,
                  lastFreqTimestamp = millis();

    while (true) {
        // 1) grab fresh data
        imu.getSensorData();  // fills imu.data.*
        float ax =  imu.data.accelX;
        float ay = -imu.data.accelY;  // invert axes to match your mount
        float az = -imu.data.accelZ;
        float gx =  imu.data.gyroX;
        float gy = -imu.data.gyroY;
        float gz = -imu.data.gyroZ;
        IMU_OK = true;
        lastIMUUpdate = millis();

        // 2) rolling average
        axBuf[idx] = ax; ayBuf[idx] = ay; azBuf[idx] = az;
        gxBuf[idx] = gx; gyBuf[idx] = gy; gzBuf[idx] = gz;
        idx = (idx + 1) % NUM_SAMPLES;

        float axA=0, ayA=0, azA=0,
              gxA=0, gyA=0, gzA=0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            axA += axBuf[i]; ayA += ayBuf[i]; azA += azBuf[i];
            gxA += gxBuf[i]; gyA += gyBuf[i]; gzA += gzBuf[i];
        }
        axA /= NUM_SAMPLES; ayA /= NUM_SAMPLES; azA /= NUM_SAMPLES;
        gxA /= NUM_SAMPLES; gyA /= NUM_SAMPLES; gzA /= NUM_SAMPLES;

        // 3) update filter
        filter.updateIMU(gxA, gyA, gzA, axA, ayA, azA);

        // 4) frequency calc
        loopCount++;
        if (millis() - lastFreqTimestamp >= 1000) {
            imuFrequency      = loopCount;
            loopCount         = 0;
            lastFreqTimestamp = millis();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

/***********************************************************************
 * @brief  Reads only GNRMC sentences from GPS and tracks frequency.
 ***********************************************************************/
void GPSTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    unsigned long loopCount        = 0,
                  lastFreqTimestamp = millis();

    while (true) {
        static String buf;
        static bool   inMsg = false;

        while (Serial2.available()) {
            char c = Serial2.read();
            if (!inMsg && c == '$') {
                inMsg = true;
                buf = c;
            } else if (inMsg) {
                buf += c;
                if (c == '\n') {
                    inMsg = false;
                    if (buf.startsWith("$GNRMC") && buf.length() > 20) {
                        xSemaphoreTake(gpsMutex, portMAX_DELAY);
                        latestGPSMessage = buf;
                        xSemaphoreGive(gpsMutex);
                        GPS_OK = true;
                        lastGPSUpdate = millis();
                    } else {
                        GPS_OK = false;
                    }
                    buf = "";
                    break;
                }
            }
        }

        if (millis() - lastGPSUpdate > 1000) GPS_OK = false;

        loopCount++;
        if (millis() - lastFreqTimestamp >= 1000) {
            gpsFrequency      = loopCount;
            loopCount         = 0;
            lastFreqTimestamp = millis();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
    }
}

/***********************************************************************
 * @brief  Sends heel, pitch, and GNRMC over UDP broadcast.
 ***********************************************************************/
void UDPTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    unsigned long loopCount        = 0,
                  lastFreqTimestamp = millis();

    while (true) {
        float    heel = filter.getRoll() + rollOffset;
        float   pitch = -filter.getPitch() + pitchOffset;
        String gpsRMC;

        if (millis() - lastGPSUpdate <= 500) {
            xSemaphoreTake(gpsMutex, portMAX_DELAY);
            gpsRMC = latestGPSMessage;
            xSemaphoreGive(gpsMutex);
        }

        // pitch
        udp.beginPacket(udpAddress, udpPort);
        udp.print(makePitchXDR(pitch));
        UDP_OK = (udp.endPacket() == 1);

        // heel
        udp.beginPacket(udpAddress, udpPort);
        udp.print(makeHeelXDR(heel));
        UDP_OK = (udp.endPacket() == 1);

        // GNRMC
        udp.beginPacket(udpAddress, udpPort);
        udp.print(gpsRMC);
        UDP_OK = (udp.endPacket() == 1);

        loopCount++;
        if (millis() - lastFreqTimestamp >= 1000) {
            udpFrequency      = loopCount;
            loopCount         = 0;
            lastFreqTimestamp = millis();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

/***********************************************************************
 * @brief  Updates the M5 LCD with status & frequencies.
 ***********************************************************************/
void DisplayTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    unsigned long loopCount        = 0,
                  lastFreqTimestamp = millis();

    while (true) {
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.setTextSize(2);

        // GPS
        M5.Lcd.setTextColor(GPS_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("GPS : %s (%.0f Hz)\n",
                      GPS_OK ? "OK" : "NOK", gpsFrequency);

        // IMU
        M5.Lcd.setTextColor(IMU_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("IMU : %s (%.0f Hz)\n",
                      IMU_OK ? "OK" : "NOK", imuFrequency);

        // Wi-Fi
        WIFI_OK = (WiFi.status() == WL_CONNECTED);
        M5.Lcd.setTextColor(WIFI_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("WiFi: %s (%s)\n",
                      WIFI_OK ? "OK" : "NOK", WiFi.localIP().toString().c_str());

        // UDP
        M5.Lcd.setTextColor(UDP_OK ? TFT_GREEN : TFT_RED);
        M5.Lcd.printf("NMEA: %s (%.0f Hz)\n",
                      UDP_OK ? "OK" : "NOK", udpFrequency);

        M5.Lcd.setCursor(0, 150);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.print("<-FWD ");
        M5.Lcd.println("^STB");

        loopCount++;
        if (millis() - lastFreqTimestamp >= 1000) {
            displayFrequency  = loopCount;
            loopCount         = 0;
            lastFreqTimestamp = millis();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
}

void connect_to_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        dots = (dots + 1) % 4;
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.printf("Connecting to WiFi%s", (dots == 0 ? "" :
                                               dots == 1 ? "." :
                                               dots == 2 ? ".." : "..."));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    // 1) Kick off M5
    auto cfg = M5.config();
    cfg.clear_display = true;
    cfg.external_imu  = false;  // we manage BMI270 ourselves
    M5.begin(cfg);
    M5.Lcd.setTextSize(2);

    // 2) Initialize Serial + I2C + Wi-Fi
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    Wire.setClock(1000000);

    connect_to_wifi();
    Serial.print("Init BMI270 @0x");
    Serial.print(i2cAddress, HEX);
    Serial.print(" ...");
    while (imu.beginI2C(i2cAddress) != BMI2_OK) {
      Serial.print('.');
      delay(500);
    }
    Serial.println(" OK!");
    IMU_OK = true;

    // 3) Start Madgwick
    filter.begin(100.0f);

    // 4) GPS serial
    Serial2.begin(38400, SERIAL_8N1, 18, 17);

    // 5) Semaphore
    gpsMutex = xSemaphoreCreateMutex();

    // 6) Spawn tasks
    xTaskCreate(IMUTask,    "IMU Task",     4096, NULL, 2, &IMUTaskHandle);
    xTaskCreate(GPSTask,    "GPS Task",     4096, NULL, 2, &GPSTaskHandle);
    xTaskCreate(UDPTask,    "UDP Task",     4096, NULL, 1, &UDPTaskHandle);
    xTaskCreate(DisplayTask,"Display Task", 4096, NULL, 1, &DisplayTaskHandle);
}

void loop() {
    // All work is done in tasks
}