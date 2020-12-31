/*
 * nRF52840Allfunction.ino: Demonstrate nRF full-function example
 * Copyright 2020 Lewis he
 */

/*******************************************************
       ______ ______  _____  _____  _____    ___  _____
       | ___ \|  ___||  ___|/ __  \|  _  |  /   ||  _  |
 _ __  | |_/ /| |_   |___ \ `' / /' \ V /  / /| || |/' |
| '_ \ |    / |  _|      \ \  / /   / _ \ / /_| ||  /| |
| | | || |\ \ | |    /\__/ /./ /___| |_| |\___  |\ |_/ /
|_| |_|\_| \_|\_|    \____/ \_____/\_____/    |_/ \___/

*********************************************************/
#include "utilities.h"
#include <SPI.h>
#include <Wire.h>

#include <GxEPD.h>
// #include <GxGDEP015OC1/GxGDEP015OC1.h>    // 1.54" b/w
#include <GxGDEH0154D67/GxGDEH0154D67.h>  // 1.54" b/w
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

#include <SerialFlash.h>
#include <pcf8563.h>
#include <RadioLib.h>
#include <Air530.h>
#ifdef USING_BMP280
#include <Adafruit_BMP280.h>
#else
#include <Adafruit_BME280.h>
#endif
#include <bluefruit.h>



void loopGPS();
void loopSensor();
void loopGPS();
void loopSender();
void loopReciver();
void sleepnRF52840();

/***********************************
   ____  ____       _
  / __ \|  _ \     | |
 | |  | | |_) |    | |
 | |  | |  _ < _   | |
 | |__| | |_) | |__| |
  \____/|____/ \____/

************************************/
SPIClass        *dispPort  = nullptr;
SPIClass        *rfPort    = nullptr;
GxIO_Class      *io        = nullptr;
GxEPD_Class     *display   = nullptr;
Air530          *gps       = nullptr;
SX1262          radio      = nullptr;       //SX1262
PCF8563_Class   rtc;
#ifdef USING_BMP280
Adafruit_BMP280 bmp;
#else
Adafruit_BME280 bme;
#endif
// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

uint32_t        blinkMillis = 0;
uint32_t        last = 0;
uint8_t         funcSelectIndex = 0;
uint8_t         prevFuncSelectIndex = 0;
bool            sleepIn = false;
bool            transmittedFlag = false;
bool            enableInterrupt = true;
bool            startListing = true;
int             transmissionState = 0;

const uint8_t index_max = 5;
typedef void (*funcCallBackTypedef)(void);
funcCallBackTypedef LilyGoCallBack[] = {loopGPS, loopSensor,  loopSender, loopReciver, sleepnRF52840};
const char *playload = "Hello World";

typedef enum {
    nRF52_RESETPIN = 1,         /*Reset from pin-reset detected*/
    nRF52_DOG,                  /*Reset from watchdog detected*/
    nRF52_SREQ,                 /*Reset from soft reset detected*/
    nRF52_LOCKUP,               /*Reset from CPU lock-up detected*/
    nRF52_OFF = bit(16),        /*Reset due to wake up from System OFF mode when wakeup is triggered from DETECT signal from GPIO*/
    nRF52_LPCOMP,               /*Reset due to wake up from System OFF mode when wakeup is triggered from ANADETECT signal from LPCOM*/
    nRF52_DIF,                  /*Reset due to wake up from System OFF mode when wakeup is triggered from entering into debug interface mode*/
    nRF52_NFC,                  /*Reset due to wake up from System OFF mode by NFC field*/
    nRF52_VBUS,                 /*Reset due to wake up from System OFF mode by VBUS rising*/
} nRF52_ResetReason;

/***********************************
   _____   _____     _____
  / ____| |  __ \   / ____|
 | |  __  | |__) | | (___
 | | |_ | |  ___/   \___ \
 | |__| | | |       ____) |
  \_____| |_|      |_____/

************************************/
bool setupGPS()
{
    SerialMon.println("[Air530] Initializing ... ");
    SerialMon.flush();
#ifndef PCA10056
    SerialGPS.setPins(Gps_Rx_Pin, Gps_Tx_Pin);
#endif
    SerialGPS.begin(9600);
    SerialGPS.flush();

    gps = new Air530(&SerialGPS, Gps_Wakeup_Pin);

    return true;
}

void loopGPS()
{
    gps->process();

    if (gps->location.isUpdated()) {
        SerialMon.print(F("LOCATION   Fix Age="));
        SerialMon.print(gps->location.age());
        SerialMon.print(F("ms Raw Lat="));
        SerialMon.print(gps->location.rawLat().negative ? "-" : "+");
        SerialMon.print(gps->location.rawLat().deg);
        SerialMon.print("[+");
        SerialMon.print(gps->location.rawLat().billionths);
        SerialMon.print(F(" billionths],  Raw Long="));
        SerialMon.print(gps->location.rawLng().negative ? "-" : "+");
        SerialMon.print(gps->location.rawLng().deg);
        SerialMon.print("[+");
        SerialMon.print(gps->location.rawLng().billionths);
        SerialMon.print(F(" billionths],  Lat="));
        SerialMon.print(gps->location.lat(), 6);
        SerialMon.print(F(" Long="));
        SerialMon.println(gps->location.lng(), 6);
    }

    else if (gps->date.isUpdated()) {
        SerialMon.print(F("DATE       Fix Age="));
        SerialMon.print(gps->date.age());
        SerialMon.print(F("ms Raw="));
        SerialMon.print(gps->date.value());
        SerialMon.print(F(" Year="));
        SerialMon.print(gps->date.year());
        SerialMon.print(F(" Month="));
        SerialMon.print(gps->date.month());
        SerialMon.print(F(" Day="));
        SerialMon.println(gps->date.day());
    }

    else if (gps->time.isUpdated()) {
        SerialMon.print(F("TIME       Fix Age="));
        SerialMon.print(gps->time.age());
        SerialMon.print(F("ms Raw="));
        SerialMon.print(gps->time.value());
        SerialMon.print(F(" Hour="));
        SerialMon.print(gps->time.hour());
        SerialMon.print(F(" Minute="));
        SerialMon.print(gps->time.minute());
        SerialMon.print(F(" Second="));
        SerialMon.print(gps->time.second());
        SerialMon.print(F(" Hundredths="));
        SerialMon.println(gps->time.centisecond());
    }

    else if (gps->speed.isUpdated()) {
        SerialMon.print(F("SPEED      Fix Age="));
        SerialMon.print(gps->speed.age());
        SerialMon.print(F("ms Raw="));
        SerialMon.print(gps->speed.value());
        SerialMon.print(F(" Knots="));
        SerialMon.print(gps->speed.knots());
        SerialMon.print(F(" MPH="));
        SerialMon.print(gps->speed.mph());
        SerialMon.print(F(" m/s="));
        SerialMon.print(gps->speed.mps());
        SerialMon.print(F(" km/h="));
        SerialMon.println(gps->speed.kmph());
    }

    else if (gps->course.isUpdated()) {
        SerialMon.print(F("COURSE     Fix Age="));
        SerialMon.print(gps->course.age());
        SerialMon.print(F("ms Raw="));
        SerialMon.print(gps->course.value());
        SerialMon.print(F(" Deg="));
        SerialMon.println(gps->course.deg());
    }

    else if (gps->altitude.isUpdated()) {
        SerialMon.print(F("ALTITUDE   Fix Age="));
        SerialMon.print(gps->altitude.age());
        SerialMon.print(F("ms Raw="));
        SerialMon.print(gps->altitude.value());
        SerialMon.print(F(" Meters="));
        SerialMon.print(gps->altitude.meters());
        SerialMon.print(F(" Miles="));
        SerialMon.print(gps->altitude.miles());
        SerialMon.print(F(" KM="));
        SerialMon.print(gps->altitude.kilometers());
        SerialMon.print(F(" Feet="));
        SerialMon.println(gps->altitude.feet());
    }

    else if (gps->satellites.isUpdated()) {
        SerialMon.print(F("SATELLITES Fix Age="));
        SerialMon.print(gps->satellites.age());
        SerialMon.print(F("ms Value="));
        SerialMon.println(gps->satellites.value());
    }

    else if (gps->hdop.isUpdated()) {
        SerialMon.print(F("HDOP       Fix Age="));
        SerialMon.print(gps->hdop.age());
        SerialMon.print(F("ms Value="));
        SerialMon.println(gps->hdop.value());
    }

    else if (millis() - last > 5000) {
        SerialMon.println();

        SerialMon.print(F("DIAGS      Chars="));
        SerialMon.print(gps->charsProcessed());
        SerialMon.print(F(" Sentences-with-Fix="));
        SerialMon.print(gps->sentencesWithFix());
        SerialMon.print(F(" Failed-checksum="));
        SerialMon.print(gps->failedChecksum());
        SerialMon.print(F(" Passed-checksum="));
        SerialMon.println(gps->passedChecksum());

        int xoffset =  92;
        display->fillRect(xoffset, 0, GxEPD_WIDTH - xoffset, GxEPD_HEIGHT, GxEPD_WHITE);

        display->setCursor(0, 25);
        display->print("[DIAGS]");
        display->setCursor(xoffset, 25);
        display->print(":");
        display->println(gps->charsProcessed());

        display->print("[Fix]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->println(gps->sentencesWithFix());

        display->print("[Pass]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->println(gps->passedChecksum());

        display->print("[SATE]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->println(gps->satellites.value());

        display->print("[Year]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->println(gps->date.year());
        display->print("[MM:DD]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->print(gps->date.month());
        display->print("/");
        display->println(gps->date.day());

        display->print("[H:M]");
        display->setCursor(xoffset, display->getCursorY());
        display->print(":");
        display->print(gps->time.hour());
        display->print(":");
        display->print(gps->time.minute());

        display->updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

        if (gps->charsProcessed() < 10) {
            SerialMon.println(F("WARNING: No GPS data.  Check wiring."));

        }
        last = millis();
        SerialMon.println();
    }
}


void sleepGPS()
{
    // gps->setTrackingMode();
}

void wakeupGPS()
{
    // gps->wakeup();
}


/***********************************
  _                    ___   ___   ___
 | |                  |__ \ / _ \ / _ \
 | |__  _ __ ___  _ __   ) | (_) | | | |
 | '_ \| '_ ` _ \| '_ \ / / > _ <| | | |
 | |_) | | | | | | |_) / /_| (_) | |_| |
 |_.__/|_| |_| |_| .__/____|\___/ \___/
                 | |
                 |_|
************************************/
bool setupSensor()
{
    SerialMon.print("[SENSOR ] Initializing ...  ");
#ifdef USING_BMP280
    if (bmp.begin()) {
        SerialMon.println("success");
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        return true;
    }
#else
    if (bme.begin()) {
        SerialMon.println("success");
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // temperature
                        Adafruit_BME280::SAMPLING_X16, // pressure
                        Adafruit_BME280::SAMPLING_X1,  // humidity
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5 );
        return true;
    }
#endif
    SerialMon.println("failed");
    return false;
}

void sleepSensor()
{
#ifdef USING_BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
#else
    bme.setSampling(Adafruit_BME280::MODE_NORMAL);
#endif
}

void wakeupSensor()
{
#ifdef USING_BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
#else
    bme.setSampling(Adafruit_BME280::MODE_NORMAL);
#endif
}

void loopSensor()
{
    if (millis() - last <  3000) {
        return;
    }

    sensors_event_t temp_event, pressure_event;
#ifdef USING_BMP280
    bmp.getTemperatureSensor()->getEvent(&temp_event);
    bmp.getPressureSensor()->getEvent(&pressure_event);
#else
    sensors_event_t humidity_event;
    bme.getTemperatureSensor()->getEvent(&temp_event);
    bme.getPressureSensor()->getEvent(&pressure_event);
    bme.getHumiditySensor()->getEvent(&humidity_event);

    SerialMon.print("Humidity = ");
    SerialMon.print(humidity_event.relative_humidity);
    SerialMon.println(" %");
#endif
    SerialMon.print(F("Temperature = "));
    SerialMon.print(temp_event.temperature);
    SerialMon.println(" *C");

    SerialMon.print(F("Pressure = "));
    SerialMon.print(pressure_event.pressure);
    SerialMon.println(" hPa");

    SerialMon.println();

    display->fillRect(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_WHITE);

    display->setCursor(0, 25);
    display->setFont(&FreeMonoBold12pt7b);
    display->println("[Temperature]");

    display->setFont(&FreeMonoBold18pt7b);
    display->setCursor(30, 80);
    display->print(temp_event.temperature);

    display->setFont(&FreeMonoBold12pt7b);
    display->setCursor(display->getCursorX(), display->getCursorY() - 20);
    display->println("*C");

    display->setCursor(0, 120);
    display->println("[Pressure]");

    display->setFont(&FreeMonoBold18pt7b);
    display->setCursor(30, 175);
    display->print(pressure_event.pressure);

    display->setFont(&FreeMonoBold12pt7b);
    display->setCursor(display->getCursorX(), display->getCursorY() - 20);
    display->println("%");

    display->updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);


    last = millis();
}

/***********************************
  ______ _                _____ _    _
 |  ____| |        /\    / ____| |  | |
 | |__  | |       /  \  | (___ | |__| |
 |  __| | |      / /\ \  \___ \|  __  |
 | |    | |____ / ____ \ ____) | |  | |
 |_|    |______/_/    \_\_____/|_|  |_|

************************************/
bool setupFlash()
{
    SerialMon.print("[FLASH ] Initializing ...  ");

    if (SerialFlash.begin(Flash_Cs)) {
        SerialMon.println("success");
        uint8_t buf[256];
        uint32_t  chipsize, blocksize;
        // Read the chip identification
        SerialMon.println();
        SerialMon.println("Read Chip Identification:");
        SerialFlash.readID(buf);
        SerialMon.print("  JEDEC ID:     ");
        SerialMon.print(buf[0], HEX);
        SerialMon.print(" ");
        SerialMon.print(buf[1], HEX);
        SerialMon.print(" ");
        SerialMon.println(buf[2], HEX);
        SerialMon.print("  Part Nummber: ");
        if (buf[0] == 0xC2 && buf[1] == 0x28 && buf[2] == 0x15) {
            SerialMon.println("MX25R1635FZUIL0");
        } else {
            SerialMon.println("unknown chip");
        }
        SerialMon.print("  Memory Size:  ");
        chipsize = SerialFlash.capacity(buf);
        SerialMon.print(chipsize);
        SerialMon.println(" bytes");
        if (chipsize != 0) {
            SerialMon.print("  Block Size:   ");
            blocksize = SerialFlash.blockSize();
            SerialMon.print(blocksize);
            SerialMon.println(" bytes");
        }
        return true;
    }
    SerialMon.println("failed");
    return false;
}

void sleepFlash()
{
    SerialFlash.sleep();
}

void wakeupFlash()
{
    SerialFlash.wakeup();
}
/***********************************
  _____ _______ _____
 |  __ \__   __/ ____|
 | |__) | | | | |
 |  _  /  | | | |
 | | \ \  | | | |____
 |_|  \_\ |_|  \_____|
***********************************/

bool setupRTC()
{
    SerialMon.print("[PCF8563] Initializing ...  ");
    Wire.begin();
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    if (Wire.endTransmission() != 0) {
        SerialMon.println("failed");
        return false;
    }
    SerialMon.println("success");
    rtc.begin(Wire);
    return true;
}


/***********************************
  _____ _____  _____ _____  _           __     __
 |  __ \_   _|/ ____|  __ \| |        /\\ \   / /
 | |  | || | | (___ | |__) | |       /  \\ \_/ /
 | |  | || |  \___ \|  ___/| |      / /\ \\   /
 | |__| || |_ ____) | |    | |____ / ____ \| |
 |_____/_____|_____/|_|    |______/_/    \_\_|

************************************/
void enableBacklight(bool en)
{
    digitalWrite(ePaper_Backlight, en);
}

void setupDisplay()
{
    dispPort = new SPIClass(
        /*SPIPORT*/NRF_SPIM2,
        /*MISO*/ ePaper_Miso,
        /*SCLK*/ePaper_Sclk,
        /*MOSI*/ePaper_Mosi);

    io = new GxIO_Class(
        *dispPort,
        /*CS*/ ePaper_Cs,
        /*DC*/ ePaper_Dc,
        /*RST*/ePaper_Rst);

    display = new GxEPD_Class(
        *io,
        /*RST*/ ePaper_Rst,
        /*BUSY*/ ePaper_Busy);

    dispPort->begin();
    display->init(/*115200*/);
    display->setRotation(0);
    display->fillScreen(GxEPD_WHITE);
    display->setTextColor(GxEPD_BLACK);
    display->setFont(&FreeMonoBold12pt7b);
}

/***********************************
  _                _____
 | |              |  __ \
 | |        ___   | |__) |   __ _
 | |       / _ \  |  _  /   / _` |
 | |____  | (_) | | | \ \  | (_| |
 |______|  \___/  |_|  \_\  \__,_|

************************************/

void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    transmittedFlag = true;
}

bool setupLoRa()
{
    rfPort = new SPIClass(
        /*SPIPORT*/NRF_SPIM3,
        /*MISO*/ LoRa_Miso,
        /*SCLK*/LoRa_Sclk,
        /*MOSI*/LoRa_Mosi);
    rfPort->begin();

    SPISettings spiSettings;

    radio = new Module(LoRa_Cs, LoRa_Dio1, LoRa_Rst, LoRa_Busy, *rfPort, spiSettings);

    SerialMon.print("[SX1262] Initializing ...  ");
    // carrier frequency:           868.0 MHz
    // bandwidth:                   125.0 kHz
    // spreading factor:            9
    // coding rate:                 7
    // sync word:                   0x12 (private network)
    // output power:                14 dBm
    // current limit:               60 mA
    // preamble length:             8 symbols
    // TCXO voltage:                1.6 V (set to 0 to not use TCXO)
    // regulator:                   DC-DC (set to true to use LDO)
    // CRC:                         enabled
    int state = radio.begin(868.0);
    if (state != ERR_NONE) {
        SerialMon.print(("failed, code "));
        SerialMon.println(state);
        return false;
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setDio1Action(setFlag);

    SerialMon.println(" success");
    return true;
}

void sleepLoRa()
{
    radio.sleep();
}

void wakeupLoRa()
{
    radio.standby();
}

void loopSender()
{

    if (millis() - last <  3000) {
        return;
    }

    if (startListing) {
        startListing = false;
        // start transmitting the first packet
        Serial.print(F("[SX1262] Sending first packet ... "));
        // you can transmit C-string or Arduino string up to
        // 256 characters long
        transmissionState = radio.startTransmit(playload);
    }

    // check if the previous transmission finished
    if (transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;
        // reset flag
        transmittedFlag = false;


        display->fillRect(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_WHITE);
        display->setCursor(0, 25);
        // display->setFont(&FreeMonoBold9pt7b);
        display->setFont(&FreeMonoBold12pt7b);
        display->println("[LoRa Sender]");
        display->println("[STATE]");

        display->setFont(&FreeMonoBold18pt7b);
        display->setCursor(50, display->getCursorY() + 20);
        if (transmissionState == ERR_NONE) {
            display->println("PASS");
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()

        } else {
            display->println("FAIL");
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);

        }
        display->setFont(&FreeMonoBold12pt7b);
        display->println("[Playload]");
        display->setCursor(0, display->getCursorY() + 10);
        // display->setFont(&FreeMonoBold18pt7b);
        display->println(playload);
        display->updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

        // send another one
        Serial.print(F("[SX1262] Sending another packet ... "));

        // you can transmit C-string or Arduino string up to
        // 256 characters long
        transmissionState = radio.startTransmit(playload);

        // you can also transmit byte array up to 256 bytes long
        /*
          byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                            0x89, 0xAB, 0xCD, 0xEF};
          int state = radio.startTransmit(byteArr, 8);
        */


        // we're ready to send more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
    last = millis();
}

void loopReciver()
{
    // check if the flag is set
    if (!startListing) {
        startListing = true;
        transmissionState = radio.startReceive();
        if (transmissionState == ERR_NONE) {
            Serial.println(F("success!"));
        } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
            while (true);
        }
    }

    if (transmittedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        transmittedFlag = false;

        // you can read received data as an Arduino String
        String str;
        int state = radio.readData(str);

        // you can also read received data as byte array
        /*
          byte byteArr[8];
          int state = radio.readData(byteArr, 8);
        */
        display->fillRect(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_WHITE);
        display->setCursor(0, 25);
        display->setFont(&FreeMonoBold9pt7b);
        display->println("[LoRa Reciver]");
        display->print("[STATE]");

        if (state == ERR_NONE) {
            // packet was successfully received
            Serial.println(F("[SX1262] Received packet!"));

            // print data of the packet
            Serial.print(F("[SX1262] Data:\t\t"));
            Serial.println(str);

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("[SX1262] RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("[SX1262] SNR:\t\t"));
            Serial.print(radio.getSNR());
            Serial.println(F(" dB"));


            display->println("PASS");

            // display->setCursor(0, display->getCursorY());
            display->print("[RSSI]");
            display->print(radio.getRSSI());
            display->println("dBm");

            // display->setCursor(0, display->getCursorY());
            display->print("[SNR]");
            display->print(radio.getSNR());
            display->println("dB");

            display->println("[Playload]");
            display->setFont(&FreeMonoBold18pt7b);
            display->setCursor(0, display->getCursorY() + 30);
            // display->setFont(&FreeMonoBold12pt7b);
            display->print(str);


        } else if (state == ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));
            display->print("ERROR");
        } else {
            display->print("FAIL");
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }

        // put module back to listen mode
        radio.startReceive();

        // we're ready to receive more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
    display->updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}



/**********************************************
*  _____  _____  _   _  _____  _   _
* |_   _||  _  || | | |/  __ \| | | |
*   | |  | | | || | | || /  \/| |_| |
*   | |  | | | || | | || |    |  _  |
*   | |  \ \_/ /| |_| || \__/\| | | |
*   \_/   \___/  \___/  \____/\_| |_/
*
************************************************/
bool touched()
{
    if (digitalRead(Touch_Pin)) {
        delay(200);
        //Wait for release
        while (digitalRead(Touch_Pin));
        return true;
    }
    return false;
}

/**********************************************
  ____  _      ______
 |  _ \| |    |  ____|
 | |_) | |    | |__
 |  _ <| |    |  __|
 | |_) | |____| |____
 |____/|______|______|

************************************************/

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
    // Get the reference to current connection
    BLEConnection *connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));
    Serial.print("Connected to ");
    Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;
    Serial.println();
    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

void startAdv(void)
{
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     *
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setupBLE()
{
    Bluefruit.autoConnLed(false);
    // Config the peripheral connection with maximum bandwidth
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
    Bluefruit.setName("Bluefruit52");
    //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();
    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Start BLE Battery Service
    blebas.begin();
    blebas.write(100);

    // Set up and start advertising
    startAdv();

    Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
    Serial.println("Once connected, enter character(s) that you wish to send");
}


/***********************************
  __  __           _
 |  \/  |         (_)
 | \  / |   __ _   _   _ __
 | |\/| |  / _` | | | | '_ \
 | |  | | | (_| | | | | | | |
 |_|  |_|  \__,_| |_| |_| |_|

************************************/

void boardInit()
{
    uint8_t rlst = 0;

    SerialMon.begin(MONITOR_SPEED);
    // delay(5000);
    // while (!SerialMon);
    SerialMon.println("Start\n");

    Bluefruit.begin();

    uint32_t reset_reason;
    sd_power_reset_reason_get(&reset_reason);
    SerialMon.print("sd_power_reset_reason_get:");
    SerialMon.println(reset_reason, HEX);


    pinMode(Power_Enable_Pin, OUTPUT);
    digitalWrite(Power_Enable_Pin, HIGH);

    pinMode(ePaper_Backlight, OUTPUT);
    digitalWrite(ePaper_Backlight, HIGH);

    pinMode(GreenLed_Pin, OUTPUT);
    pinMode(RedLed_Pin, OUTPUT);
    pinMode(BlueLed_Pin, OUTPUT);

    pinMode(Touch_Pin, INPUT_PULLDOWN_SENSE);

    int i = 5;
    while (i--) {
        digitalWrite(GreenLed_Pin, !digitalRead(GreenLed_Pin));
        digitalWrite(RedLed_Pin, !digitalRead(RedLed_Pin));
        digitalWrite(BlueLed_Pin, !digitalRead(BlueLed_Pin));
        delay(500);
    }
    digitalWrite(GreenLed_Pin, HIGH);
    digitalWrite(RedLed_Pin, HIGH);
    digitalWrite(BlueLed_Pin, HIGH);

    // setupBLE();

    setupDisplay();

    rlst |= setupGPS() ? bit(1) : 0 ;

    rlst |= setupLoRa() ? bit(2) : 0;

    rlst |= setupFlash() ? bit(3) : 0;

    rlst |= setupRTC() ? bit(4) : 0;

    rlst |= setupSensor() ? bit(5) : 0;

    display->setCursor(15, 25);
    display->setFont(&FreeMonoBold9pt7b);
    display->println("ePaper SeftTest");
    display->drawFastHLine(0, display->getCursorY() - 5, display->width(), GxEPD_BLACK);
    display->println();
    display->setFont(&FreeMonoBold12pt7b);
    display->print("[Air530] ");
    display->println(rlst & bit(1) ? "PASS" : "FAIL");
    display->print("[SX1262] ");
    display->println(rlst & bit(2) ? "PASS" : "FAIL");
    display->print("[FLASH ] ");
    display->println(rlst & bit(3) ? "PASS" : "FAIL");
    display->print("[PCF8563]");
    display->println(rlst & bit(4) ? "PASS" : "FAIL");
    display->print("[SENSOR ]");
    display->println(rlst & bit(5) ? "PASS" : "FAIL");

    display->print("[GPS VER]");
    display->println(gps->getSoftVersion());

    display->update();
}

void wakeupPeripherals()
{
    wakeupGPS();
    wakeupLoRa();
    wakeupSensor();
    wakeupFlash();
    enableBacklight(true);
    pinMode(GreenLed_Pin, OUTPUT);
    pinMode(RedLed_Pin, OUTPUT);
    pinMode(BlueLed_Pin, OUTPUT);
}

void sleepPeripherals()
{
    sleepGPS();
    sleepFlash();
    sleepLoRa();
    sleepSensor();
    enableBacklight(false);

    digitalWrite(Power_Enable_Pin, LOW);
    pinMode(Power_Enable_Pin, INPUT);

    pinMode(GreenLed_Pin, INPUT);
    pinMode(RedLed_Pin, INPUT);
    pinMode(BlueLed_Pin, INPUT);
}

void sleepnRF52840()
{
    sleepIn = true;
    sleepPeripherals();

    // power down nrf52.
    sd_power_system_off();       // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.

    // wakeupPeripherals();

}

void setup()
{
    boardInit();
    delay(2000);
    display->setFont(&FreeMonoBold12pt7b);
    display->fillScreen(GxEPD_WHITE);
    display->update();

}

uint8_t rgb = 0;
void loop()
{
    if (millis() - blinkMillis > 1000) {
        blinkMillis = millis();
        switch (rgb) {
        case 0:
            digitalWrite(GreenLed_Pin, LOW);
            digitalWrite(RedLed_Pin, HIGH);
            digitalWrite(BlueLed_Pin, HIGH);
            break;
        case 1:
            digitalWrite(GreenLed_Pin, HIGH);
            digitalWrite(RedLed_Pin, LOW);
            digitalWrite(BlueLed_Pin, HIGH);
            break;
        case 2:
            digitalWrite(GreenLed_Pin, HIGH);
            digitalWrite(RedLed_Pin, HIGH);
            digitalWrite(BlueLed_Pin, LOW);
            break;
        default :
            break;
        }
        rgb++;
        rgb %= 3;
    }

    if (touched() || sleepIn) {
        sleepIn = false;
        funcSelectIndex++;
        funcSelectIndex %= index_max;
        Serial.print("funcSelectIndex:");
        Serial.println(funcSelectIndex);

        if (prevFuncSelectIndex != funcSelectIndex) {
            display->fillScreen(GxEPD_WHITE);
            display->update();
            prevFuncSelectIndex = funcSelectIndex;
        }
    }

    if (LilyGoCallBack[funcSelectIndex]) {
        LilyGoCallBack[funcSelectIndex]();
    }
    __SEV();
    __WFE();
    __WFE();
}

















