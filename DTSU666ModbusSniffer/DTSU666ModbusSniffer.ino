/*
 *  DTSU666ModbusSniffer.cpp
 *
 *  The program listens to the DTSU666 reply and show values on a 1602 LCD display.
 *  The requests are generated by a Deye inverter.
 *
 *  Because Modbus protocol is a master slave one, we can listen to the TX and RX of the communication
 *  by connecting both with a schottky diode to the Arduino RX pin.
 *
 *
 *  Copyright (C) 2023-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/PVUtils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include "LongUnion.h"

#define MODBUS_BAUDRATE              9600
//#define STANDALONE_TEST                   // If activated, random power is displayed on LCD.
#define DEBUG_PIN                       3 // If low, print raw protocol data. Pin 2 is left for a future button.

#define MODBUS_MINIMUM_FRAME_LENGTH     7 // 0x01 0x03 0x02 0x00 0x00 0xB8 0x44 - Reply of one 16 bit word
#define MODBUS_REQUEST_FRAME_LENGTH     8 // /*ID*/ 0x01 /*FUNCTION*/ 0x03 /*ADDR*/ 0x00 0x13 /*NUMBER*/ 0x00 0x01 /*CRC*/ 0x75 0xCF

#define MODBUS_ID_INDEX                 0
#define MODBUS_FUNCTION_INDEX           1
#define MODBUS_REQUEST_ADDRESS_INDEX    2
#define MODBUS_REQUEST_NUMBER_INDEX     4
#define MODBUS_REPLY_LENGTH_INDEX       2
#define MODBUS_REPLY_DATA_INDEX         3
#define MODBUS_RECEIVE_INTRA_FRAME_SILENCE_MILLISECONDS     3 // Standard says: 3.5 character, which is 4 ms at 9600 baud. Then I can check CRC and switch to 105200 for log sending.
#define MODBUS_RECEIVE_INTRA_REQUEST_GAP_MILLISECONDS       100 // Between sending of Modbus request and answer and new request I observed around 25 ms.

#define MODBUS_FRAME_TYPE_CRC_ERROR     0
#define MODBUS_FRAME_TYPE_REQUEST       1
#define MODBUS_FRAME_TYPE_REPLY         2

struct ModbusFrameInfoStruct {
//    uint16_t RegisterAddress;
//    uint16_t RequestRegisterRange;
    uint8_t ReplyBytes;
    uint8_t FrameType;              // One of MODBUS_FRAME_TYPE_CRC_ERROR, MODBUS_FRAME_TYPE_REQUEST, MODBUS_FRAME_TYPE_REPLY
    uint8_t RXBuffer[128];          // 90 bytes is the maximum I saw
    uint16_t RXBufferIndex = 0;     // Index of next byte to write to array. Starting with 0.
    uint32_t MillisOfLastReceivedByte;
} sModbusFrameInfo;

//bool sValuesPrinted = true;
int16_t sPower[3];
int16_t sPowerLowpass4[3];

#define VERSION_EXAMPLE "0.9"

void dumpBuffer();
void readModbusByte();
void processModbusPDUFrame(bool tPrintBuffers);
// Content specific function
void interpreteModbusPDUContent();
void printDataToSerialAndLCD();
void printDataToLCD();
void blinkForever();
uint16_t calculateCrc(uint8_t *aBuffer, uint16_t aBufferLength);

/*
 * LCD stuff
 */
#define LCD_COLUMNS     16
#define LCD_ROWS         2
#define LCD_I2C_ADDRESS 0x27     // Default LCD address is 0x27 for a I2C to LCD converter
#include "LiquidCrystal_I2C.hpp" // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use the modified version delivered with this program!
LiquidCrystal_I2C myLCD(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
bool sSerialLCDAvailable;
char sStringBufferForLCDRow[LCD_COLUMNS + 1]; // For rendering LCD lines with sprintf()

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(DEBUG_PIN, INPUT_PULLUP);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
    Serial.println(F("Data is printed with 115200 baud during the 7 seconds gap between the logger requests."));
    Serial.println(F("If you connect debug pin " STR(DEBUG_PIN) " to ground, raw protocol data is printed"));
    Serial.println(F("Wait for 9600 baud modbus data on RX"));

    /*
     * Initialize I2C and check for bus lockup
     */
    if (!i2c_init()) {
        Serial.println(F("I2C init failed"));
    }
    /*
     * Check for LCD connected
     */
    sSerialLCDAvailable = i2c_start(LCD_I2C_ADDRESS << 1);
    i2c_stop();

    if (sSerialLCDAvailable) {
        /*
         * Print program, version and date on the upper two LCD lines
         */
        myLCD.init();
        myLCD.clear();
        myLCD.backlight();
        myLCD.setCursor(0, 0);
        myLCD.print(F("Modbus Sniffer"));
        myLCD.setCursor(0, 1);
        myLCD.print(F(VERSION_EXAMPLE " " __DATE__));
        delay(2000);
        myLCD.setCursor(0, 1);
        myLCD.print(F("Debug pin = " STR(DEBUG_PIN) "   "));
        delay(1000);
        myLCD.setCursor(0, 1);
#if defined(STANDALONE_TEST)
        myLCD.print(F("Standalone test "));
#else
        myLCD.print(F("Wait for RX data"));
#endif
        delay(1000);

    } else {
        Serial.println(F("No I2C 2004 LCD connected at address " STR(LCD_I2C_ADDRESS)));
        blinkForever();
    }

    Serial.println();
    Serial.flush();
    Serial.begin(MODBUS_BAUDRATE);
}

/*
 * Check for 9600 baud signal at RX and store it in buffer.
 */
void loop() {

#if defined(STANDALONE_TEST)
    sPower[0] = random(-1111,3333);
    sPower[1] = random(-1111,3333);
    sPower[2] = random(-1111,3333);
    if (sSerialLCDAvailable) {
        printDataToLCD();
    }
    delay(3000);
#else
    bool tPrintBuffers = !digitalRead(DEBUG_PIN);

//    if (!sValuesPrinted
//            && sModbusFrameInfo.RXBufferIndex
//                    == 0&& (millis() - sModbusFrameInfo.MillisOfLastReceivedByte) > MODBUS_RECEIVE_INTRA_REQUEST_GAP_MILLISECONDS) {
//        /*
//         * We are in the 7 seconds gap here :-)
//         */
//        printDataToSerialAndLCD();
//    }

    /*
     * Try to read byte if buffer empty or last byte was received just before - otherwise we have a timeout signaling end of modbus frame
     */
    if (sModbusFrameInfo.RXBufferIndex
            == 0|| (millis() - sModbusFrameInfo.MillisOfLastReceivedByte) < MODBUS_RECEIVE_INTRA_FRAME_SILENCE_MILLISECONDS) {
        /*
         * Modbus byte is expected, test and read
         */
        readModbusByte();
    } else {
        /*
         * We are in a gap between request and reply ore vice versa.
         * Time to interpret buffer and print raw log output if debug pin is low.
         */
        processModbusPDUFrame(tPrintBuffers);
        if (sModbusFrameInfo.FrameType == MODBUS_FRAME_TYPE_REPLY) {
            interpreteModbusPDUContent();
            printDataToSerialAndLCD();
            //        sValuesPrinted = false; // enable new print
        }
    }
#endif
}

void readModbusByte() {
    /*
     * Modbus byte is expected, test and read
     */
    if (Serial.available()) {
        uint8_t tReceivedValue = Serial.read();
        sModbusFrameInfo.RXBuffer[sModbusFrameInfo.RXBufferIndex] = tReceivedValue;
// avoid buffer overflow
        if (sModbusFrameInfo.RXBufferIndex < sizeof(sModbusFrameInfo.RXBuffer) - 1) {
            sModbusFrameInfo.RXBufferIndex++;
        }
        sModbusFrameInfo.MillisOfLastReceivedByte = millis();
    }
}

/*
 * Interpret PDU and print raw log output if debug pin is low.
 * Do not check for CRC!!!
 */
void processModbusPDUFrame(bool tPrintBuffers) {
    if (sModbusFrameInfo.RXBufferIndex >= MODBUS_MINIMUM_FRAME_LENGTH) {
        if (tPrintBuffers) {
            Serial.begin(115200);
        }
        /*
         * Decide if request or reply frame
         */
        if (sModbusFrameInfo.RXBufferIndex == MODBUS_REQUEST_FRAME_LENGTH) {
            /*
             * Request frame here
             */
            sModbusFrameInfo.FrameType = MODBUS_FRAME_TYPE_REQUEST;
//            sModbusFrameInfo.RegisterAddress = sModbusFrameInfo.RXBuffer[MODBUS_REQUEST_ADDRESS_INDEX] << 8
//                    | sModbusFrameInfo.RXBuffer[MODBUS_REQUEST_ADDRESS_INDEX + 1];
//            sModbusFrameInfo.RequestRegisterRange = sModbusFrameInfo.RXBuffer[MODBUS_REQUEST_NUMBER_INDEX] << 8
//                    | sModbusFrameInfo.RXBuffer[MODBUS_REQUEST_NUMBER_INDEX + 1];
            if (tPrintBuffers) {
                Serial.print(F("Request="));
//                Serial.print(sModbusFrameInfo.RegisterAddress, HEX);
//                Serial.print(F(", L="));
//                Serial.print(sModbusFrameInfo.RequestRegisterRange);
            }

        } else {
            /*
             * Assume reply frame
             */
            sModbusFrameInfo.ReplyBytes = sModbusFrameInfo.RXBuffer[MODBUS_REPLY_LENGTH_INDEX];
            uint16_t tComputedCRC = calculateCrc(sModbusFrameInfo.RXBuffer, sModbusFrameInfo.ReplyBytes + 3);
            if (tComputedCRC
                    == (uint16_t) (sModbusFrameInfo.RXBuffer[sModbusFrameInfo.RXBufferIndex - 2] << 8
                            | sModbusFrameInfo.RXBuffer[sModbusFrameInfo.RXBufferIndex - 1])) {
                sModbusFrameInfo.FrameType = MODBUS_FRAME_TYPE_REPLY;

                if (tPrintBuffers) {
                    Serial.print(F("Reply="));
                }
            } else {
                // CRC Error here
                sModbusFrameInfo.FrameType = MODBUS_FRAME_TYPE_CRC_ERROR;
                myLCD.setCursor(0, 0);
                myLCD.print('C');
            }

        }
        if (tPrintBuffers) {
            dumpBuffer();
            Serial.println();
            Serial.flush();
            Serial.begin(MODBUS_BAUDRATE);
        }
    }
// prepare for next receiving of request or reply
    sModbusFrameInfo.RXBufferIndex = 0;
}

/*
 * Deye Request frame 8 ms every 100 ms -> Reply frame 18 ms
 * 01 03  15 1E  00 06  A1 C2 -> 01 03 0C  00 00 00 00  00 00 00 00  00 00 00 00  93 70
 * At address 15 1E Deye expects float power values in kW units
 */
void interpreteModbusPDUContent() {
    LongUnion tPower;
    uint8_t tReplyBufferIndex = MODBUS_REPLY_DATA_INDEX;
    for (uint_fast8_t i = 0; i < 3; ++i) {
        // Reverse bytes. We received MSB . . LSB
        for (int_fast8_t j = 3; j >= 0; j--) {
            tPower.UBytes[j] = sModbusFrameInfo.RXBuffer[tReplyBufferIndex++];
        }
        sPower[i] = tPower.Float * 1000.0; // At address 15 1E Deye expects float power values in kW units
    }
}

/*
 * Dump buffer, 3 character per byte
 */
void dumpBuffer() {
    Serial.print(F("0x"));
    for (uint_fast8_t i = 0; i < sModbusFrameInfo.RXBufferIndex; i++) {
        Serial.print(sModbusFrameInfo.RXBuffer[i], HEX);
        Serial.print(F(" "));
    }
}

/*
 * Print data to serial and LCD
 */
void printDataToSerialAndLCD() {
//    Serial.begin(115200);
//    Serial.println();
//
//    Serial.println();
//    Serial.flush();
//    Serial.begin(MODBUS_BAUDRATE);

    if (sSerialLCDAvailable) {
        printDataToLCD();
    }
//    sValuesPrinted = true;
}

/*
 * 11 ms. 6 ms with delayMicroseconds(40); and 3.4 ms with delayMicroseconds(2) instead of delayMicroseconds(100);
 * commands need > 37us to settle
 * Called every MILLISECONDS_BETWEEN_LCD_OUTPUT (320 ms)
 */
void print6DigitsWatt(int aWattToPrint) {
    sprintf_P(sStringBufferForLCDRow, PSTR("%6d W"), aWattToPrint); // force use of 6 columns
    myLCD.print(sStringBufferForLCDRow);
}

void printDataToLCD() {
    myLCD.setCursor(0, 0);

    /*
     * Apply EMA lowpass alpha = 0.0625 | 1/16 to the values.
     * Cutoff frequency is 0.106 Hz @10Hz.
     * https://github.com/ArminJo/Arduino-Utils?tab=readme-ov-file#simpleemafilters
     * Skip lowpass if delta is more than 20 (guessed value),
     * otherwise we see a slow increasing value at a power jump, caused by switching a load.
     */
    for (uint_fast8_t i = 0; i < 3; ++i) {
        if (abs(sPowerLowpass4[i] - sPower[i]) > 20) {
            sPowerLowpass4[i] = sPower[i]; // Fast response, reinitialize lowpass.
        } else {
            sPowerLowpass4[i] += ((sPower[i] - sPowerLowpass4[i]) + (1 << 3)) >> 4; // 2.2 us, alpha = 0.0625, cutoff frequency 10.6 Hz @1kHz
        }
    }

    print6DigitsWatt(sPowerLowpass4[0]);
    print6DigitsWatt(sPowerLowpass4[1]);

    myLCD.setCursor(0, 1);
    print6DigitsWatt(sPowerLowpass4[2]);
    print6DigitsWatt(sPowerLowpass4[0] + sPowerLowpass4[1] + sPowerLowpass4[2]); // Print sum of all
}

void blinkForever() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
    }
}

/* Table of CRC values for high-order byte */
#if defined(ARDUINO) && defined(__AVR__)
static PROGMEM const uint8_t CRCHighTable[] = {
#else
const uint8_t CRCHighTable[] = {
#endif
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low order-byte */
#if defined(ARDUINO) && defined(__AVR__)
static PROGMEM const uint8_t CRCLowTable[] = {
#else
const uint8_t CRCLowTable[] = {
#endif
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

uint16_t calculateCrc(uint8_t *aBuffer, uint16_t aBufferLength) {
    uint8_t tCRCHigh = 0xFF; /* high CRC byte initialized */
    uint8_t tCRCLow = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (aBufferLength--) {
        i = tCRCHigh ^ *aBuffer++; /* calculate the CRC  */
#if defined(ARDUINO) && defined(__AVR__)
        tCRCHigh = tCRCLow ^ pgm_read_byte_near(CRCHighTable + i);
        tCRCLow = pgm_read_byte_near(CRCLowTable + i);
#else
        tCRCHigh = tCRCLow ^ CRCHighTable[i];
        tCRCLow = CRCLowTable[i];
#endif
    }

    return (tCRCHigh << 8 | tCRCLow);
}
