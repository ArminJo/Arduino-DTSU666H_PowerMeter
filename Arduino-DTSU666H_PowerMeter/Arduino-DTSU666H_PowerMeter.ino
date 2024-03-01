/**
 * Arduino-DTSU666H_PowerMeter.cpp
 *
 *  Substitutes the RS485 modbus service of DTSU-H 3 phase power meter for a Deye hybrid inverter.
 *  At address 0x15 1E Deye asks for 3 float power values in kW units.
 *  Measuring is done with an Arduino Nano and 3 30A CT's.
 *
 *
 *  Copyright (C) 2023-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-DTSU666H_PowerMeter https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter.
 *
 *  Arduino-DTSU666H_PowerMeter is free software: you can redistribute it and/or modify
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

// https://www.ni.com/en/shop/seamlessly-connect-to-third-party-devices-and-supervisory-system/the-modbus-protocol-in-depth.html
// https://ipc2u.de/artikel/wissenswertes/modbus-rtu-einfach-gemacht-mit-detaillierten-beschreibungen-und-beispielen/
/*
 * We assume, that voltage waveform of the 3 phases are equal and negative and positive values are symmetric
 * so we take only one half wave as voltage reference.
 *
 * Phase A (L1) is reference (the one, which supplies the voltage and which can be negative),
 *   phase B has a delay of 6.6 ms / 120 degree and C a delay of 13.3 ms / 240 degree.
 *
 * 1. Read "positive" part voltage values of phase A for 10 ms and store 385 values in RAM to be used as reference for all 3 phases.
 *    This starts 16 Bit timer 1 to keep track of the phase for the next current measurements.
 * 2. Wait 3.3 ms.
 * 3. After 10 + 3.333 ms do 10 ms phase C current measurement. Multiply values with voltage.
 * 4. Wait 3.3 ms.
 * 5. After 20 + 6.666 ms do 10 ms phase B current measurement. Multiply values with voltage.
 * 6. Wait 3.3 ms
 * 7. After 40 ms do 20 ms phase A current measurement, which will also cover negative current. Multiply values with voltage.
 * 8. 20 ms for math and reply to RS485 or output to LCD until starting again at 80 ms - x ms.
 *
 * Alternative timing for covering negative values of all 3 phases:
 * 13.3 | phase C | 20 ms, 40 | phase A | 20 ms, 66.6 phase B | 20 ms, 100 | voltage phase A | 113.3 ...
 *
 * The Deye inverter sends a 9600 baud modbus request 01 03  15 1E  00 06  A1 C2 every 100 ms to 120 ms.
 * We send the reply at pin 2 with software serial at a 80 ms raster.
 * This means, around every 400 ms we have one loop where we do not need to reply and can update the LCD instead.
 * Sometimes the Deye sends the request, while we do a reply.
 */

/*
 * Every watt-hour, the build-in LED flashes for 80 ms (a complete measurement loop).
 *
 * There are 4 LCD pages
 * Power    Power in watt
 * Energy   Energy in watt-hour
 * Info     Time for energy and power correction percentage.
 * Extra    Period of one loop which is 4 mains phases, around 80 ms.
 *
 * If you press the page button for longer than 1 second to switch from Power to Energy page, the energy will be reset to 0.
 *
 */
#include <Arduino.h>

#include "MillisUtils.h"    // For enableMillisInterrupt(), disableMillisInterrupt()
#include "ADCUtils.h"       // For ADC_PRESCALE32 and SHIFT_VALUE_FOR_REFERENCE
#include "AVRUtils.h"       // For initStackFreeMeasurement(), printRAMInfo()
#include "digitalWriteFast.h"
#include "LongUnion.h"
#include "LiquidCrystal.h"
#include <avr/wdt.h>

#define VERSION_EXAMPLE "1.2"

/*
 * Mapping of power phase A, B, C to ADC channels 1, 2, 3, which is also the internal index and display and modbus position.
 * The voltage input (ADC channel 0) must be from the line, which can be negative
 * Adapt it to your needs
 */
#define LINE_WHICH_CAN_BE_NEGATIVE      2 // (A) The internal index / channel of the power line, which can be negative
#define LINE_WITH_7_MS_DELAY            1 // (B) The internal index / channel of the power line, which is 6ms delayed to the line, which can be negative
#define LINE_WITH_13_MS_DELAY           3 // (C) The internal index / channel of the power line, which is 13.3 ms delayed to the line, which can be negative

#define ADC_CHANNEL_FOR_VOLTAGE     0

//#define TIMING_DEBUG  Output Timing waveform at pin A5
/*
 * Modbus stuff
 */
struct ModbusRTURequestStruct {
    uint8_t SlaveAddress;
    uint8_t FunctionCode; // 03 -> Read Multiple Registers
    uint16_t FirstRegisterAddress; // high byte is sent first
    uint16_t ReplyLengthInWord;
    uint16_t CRC; // CRC-16-MODBUS x^16 + x^15 + x^2 + 1
};

#define MODBUS_REQUEST_LENGTH           (sizeof(ModbusRTURequestStruct)) // 8

union ModbusRTURequestUnion {
    uint8_t ReceiveByteBuffer[MODBUS_REQUEST_LENGTH];
    ModbusRTURequestStruct ModbusRTURequest;
};
ModbusRTURequestUnion sModbusRTURequestUnion;

struct ModbusRTUMinimalReplyStruct {
    uint8_t SlaveAddress;
    uint8_t FunctionCode; // 03 -> Read Multiple Registers
    uint8_t ReplyByteLength;
    uint16_t SwappedFirstRegisterContent; // high byte was sent first
    uint16_t CRC; // CRC-16-MODBUS x^16 + x^15 + x^2 + 1
};
#define MODBUS_REPLY_ONE_WORD_LENGTH    (sizeof(ModbusRTUMinimalReplyStruct))

struct ModbusRTU3FloatReplyStruct {
    uint8_t SlaveAddress;
    uint8_t FunctionCode; // 03 -> Read Multiple Registers
    uint8_t ReplyByteLength;
    float SwappedRegisterContent[3]; // high byte was sent first
    uint16_t CRC; // CRC-16-MODBUS x^16 + x^15 + x^2 + 1
};
#if defined SUPPORT_SOFAR_EXTENSIONS
struct ModbusRTU6FloatReplyStruct {
    uint8_t SlaveAddress;
    uint8_t FunctionCode; // 03 -> Read Multiple Registers
    uint8_t ReplyByteLength;
    float SwappedRegisterContent[6]; // high byte was sent first
    uint16_t CRC; // CRC-16-MODBUS x^16 + x^15 + x^2 + 1
};
#define MODBUS_REPLY_POWER_LENGTH      (sizeof(ModbusRTU6FloatReplyStruct)) // 29
#else
#define MODBUS_REPLY_POWER_LENGTH      (sizeof(ModbusRTU3FloatReplyStruct)) // 17
#endif

union ModbusRTUReplyUnion {
    uint8_t ReplyByteBuffer[MODBUS_REPLY_POWER_LENGTH] = { 0x01, 0x03 };
    ModbusRTUMinimalReplyStruct ModbusRTUMinimalReply;
    ModbusRTU3FloatReplyStruct ModbusRTU3FloatReply;
#if defined SUPPORT_SOFAR_EXTENSIONS
    ModbusRTU3FloatReplyStruct ModbusRTU6FloatReply;
#endif
};
ModbusRTUReplyUnion sModbusRTUReplyUnion;

#define MODBUS_BAUDRATE                         9600
bool checkAndReplyToModbusRequest();
uint16_t calculateCrc(uint8_t *aBuffer, uint16_t aBufferLength);

/*
 * Pin mappings
 */
#define MODBUS_RX_PIN                           RX // 0 - Only for documentation. We use the Serial RX pin. Not used in program.
#if !defined(MODBUS_TX_PIN)                        // Allow override by global symbol
#define MODBUS_TX_PIN                           2
#endif

#define PAGE_BUTTON_PIN                         3 // Only for documentation. Handled by EasyButtonAtInt01.
#define POWER_CORRECTION_MINUS_PIN              4
#define POWER_CORRECTION_PLUS_PIN               5

#define ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN       6 // Print periodically info messages on serial output suited for Arduino Serial Plotter
#if defined(TIMING_DEBUG)
#define TIMING_DEBUG_OUTPUT_PIN                 A5
#define TIMING_PIN_HIGH()                       digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH)
#define TIMING_PIN_LOW()                        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW)
#else
#define TIMING_PIN_HIGH()                       void()
#define TIMING_PIN_LOW()                        void()

#endif

#define DURATION_OF_ONE_MEASUREMENT_MILLIS      60 // Cannot be changed! From start of voltage measurement to end of L1 negative current measurement
#define DURATION_OF_ONE_LOOP_MILLIS             80 // Cannot be changed! One measurement and 20 ms for synchronizing voltage used for communication and print
#define LOOPS_PER_MINUTE                        (60000 / DURATION_OF_ONE_LOOP_MILLIS)
uint16_t sLastTCNT1;
uint16_t sDeltaTCNT1; // Difference between two synchronizing points, with 26 us resolution because of ADC period of 26 us e.g. 79911, 79885, 79859

/*
 * LCD stuff
 */
LiquidCrystal myLCD(11, 12, 7, 8, 9, 10); // Pins 7 to 12

#define LCD_COLUMNS                             20
#define LCD_ROWS                                4
#define MILLISECONDS_BETWEEN_LCD_OUTPUT         (8 * DURATION_OF_ONE_LOOP_MILLIS) // 640
void printDataOnLCD();                  // Called every MILLISECONDS_BETWEEN_LCD_OUTPUT

uint32_t sMillisOfLastLCDOutput;
char sStringBufferForLCDRow[17];        // For rendering LCD lines with sprintf()
uint8_t sLCDInfoPageCounter;            // To update Info page only once every 20.5 seconds
/*
 * Counts 12 per second.
 * Reset page to POWER_METER_PAGE_POWER after 20 min
 */
uint16_t sLCDLoopCounter = 0;           // Counts 12 per second.

#define LOOPS_OF_CORRECTION_MESSAGE_DISPLAY_FREEZE  36
#define LOOPS_OF_RECEIVE_MESSAGE_DISPLAY_FREEZE     1440 // 2 minutes
uint16_t sCounterForDisplayFreeze = 0; // 12 per second. Freezes the lower right quarter of the display, i.e does not print power sum.

void printPaddedHexOnMyLCD(uint8_t aHexByteValue);

/*
 * Software serial for Modbus reply sending
 */
#include "SoftwareSerialTX.h"
/*
 * Use a 115200 baud software serial for the short request frame.
 * If available, we also can use a second hardware serial here :-).
 */
SoftwareSerialTX TxToModbusClient(MODBUS_TX_PIN);

/*
 * For power data acquisition
 * for LCD we accumulate up to 10 periods, so we have an overflow above around 3.2 kW if we use int16_t
 */
uint8_t sNumberOfPowerSamplesForLCD;    //
int32_t sPowerForLCDAccumulator[3]; // Index 0 is for L1, 1 is for L2 at ADC channel 2 etc. We can have up to 10 samples before displayed on LCD.
uint8_t sNumberOfPowerSamplesForModbus;
int32_t sPowerForModbusAccumulator[3];  // Index 0 is for L1, 1 is for L2 at ADC channel 2 etc.

int32_t sEnergyAccumulator[3];          // Contains sum of sNumberOfSamplesForEnergy entries of power
int32_t sEnergyAccumulatorSumForFlash;
uint8_t sWattHourFlashCounter;
uint32_t sNumberOfEnergySamplesForLCD;

int16_t sPowerSum;

#define NUMBER_OF_SAMPLES_FOR_10_MILLIS 384 // 10000 us / (26 us / sample) = 384.6153846153846
/*
 * Place this array at end of BSS to be first overwritten by stack. Only one array available for printing of current.
 */
uint16_t sCurrentArray[NUMBER_OF_SAMPLES_FOR_10_MILLIS] __attribute__((section(".noinit")));
uint16_t sVoltageArray[NUMBER_OF_SAMPLES_FOR_10_MILLIS];

#define MILLISECONDS_BETWEEN_SERIAL_PLOTTER_OUTPUT     2000
uint32_t sMillisOfLastSerialPlotterOutput;

/*
 * Assume current full range (1.1 V) at 30 A and voltage full range at 400 V with divider 3.63 MOhm to 10 kOhm.
 * => current LSB is 29 mA and voltage LSB is 390.6 mV => Power LSB is 11.4 mW
 * We sum 384 samples per measurement so here we have LSB of 12 mW / 384 = 0.0298 mW
 * 1 / 0.0298 = 33554
 */
#define POWER_SCALE_DIVISOR         32768 // We take a power of 2 (and use correction percentage) instead of NUMBER_OF_SAMPLES_FOR_10_MILLIS / 0.00114 watt
#define ENERGY_DIVISOR              ((3600L * 1000L) / DURATION_OF_ONE_LOOP_MILLIS) // 45000. 3600 seconds in a hour and 1000 ms / 80 ms samples per second
/*
 * Power correction
 */
uint8_t sPowerCorrectionPercentage = 100;
EEMEM uint8_t sPowerCorrectionPercentageEeprom;    // Storage in EEPROM for sTemperatureCorrectionFloat
uint8_t s80MillisecondsAutorepeatCounter = 0;
#define POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE      1 // One percent. The value to add or subtract to sPowerCorrectionFloat at each correction button press
void checkPowerCorrectionPins();
void printCorrectionPercentageOnLCD();

void readVoltage(bool aDoFindZeroCrossing);
uint32_t readCurrentRaw(bool aStoreInArray = false);
void printPower();

void checkAndPrintInputSignalValuesForArduinoPlotter();
volatile uint8_t sIndexOfCurrentToPrint = 1; // 0 = negative L1, 1 = LI, 2 = L2, 3 = L3.
volatile bool sPageButtonJustPressed = false;
void handlePageButtonPressForArduinoPlotterLineSelect();

uint16_t swap(uint16_t aWordToSwapBytes);
uint32_t swap(uint32_t aLongToSwapBytes);

#define POWER_METER_PAGE_POWER          0
#define POWER_METER_PAGE_ENERGY         1
#define POWER_METER_PAGE_INFO           2   // Shows time of energy and power correction percentage
#define POWER_METER_PAGE_EXTRA          3   // Shows microseconds of mains period
#define POWER_METER_PAGE_MAX            POWER_METER_PAGE_EXTRA
volatile uint8_t sLCDDisplayPage = POWER_METER_PAGE_POWER;

/*
 * Button at INT0 / D2 for switching display pages, or if plotter output enabled, select power lines to be printed on plotter
 */
#define USE_BUTTON_1              // Enable code for 1. button at INT1 / D3
#define BUTTON_DEBOUNCING_MILLIS 120 // With this you can adapt to the characteristic of your button. Default is 50.
#define NO_BUTTON_RELEASE_CALLBACK  // Disables the code for release callback. This saves 2 bytes RAM and 64 bytes program memory.

#include "EasyButtonAtInt01.hpp"
#define LONG_PRESS_BUTTON_DURATION_MILLIS   1000

void handlePageButtonPress(bool aButtonToggleState __attribute__((unused))) {
    sPageButtonJustPressed = true;
    if (!digitalReadFast(ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN)) {
        // Select phase to be plotted
        sIndexOfCurrentToPrint = ((sIndexOfCurrentToPrint + 1) & 0x03);    // increment the buffer to print an wrap at 4
    } else {
        // switch LCD page. Long press handling for reset Energy page is in loop.
        sLCDInfoPageCounter = 0;
        sLCDDisplayPage++;
        if (sLCDDisplayPage > POWER_METER_PAGE_MAX) {
            sLCDDisplayPage = POWER_METER_PAGE_POWER;
        }
    }
}
EasyButton PageButtonAtPin3(&handlePageButtonPress); // Button is connected to INT1

uint8_t sMCUSRStored; // content of MCUSR register at startup

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void setup() {
    sMCUSRStored = MCUSR; // content of MCUSR register at startup
    MCUSR = 0; // To reset old boot flags for next boot
    /*
     * Disable watchdog for setup
     */
    wdt_disable();

    initStackFreeMeasurement();

// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);
    pinMode(POWER_CORRECTION_PLUS_PIN, INPUT_PULLUP);
    pinMode(POWER_CORRECTION_MINUS_PIN, INPUT_PULLUP);
    pinMode(ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN, INPUT_PULLUP);
#if defined(TIMING_DEBUG)
    pinMode(TIMING_DEBUG_OUTPUT_PIN, OUTPUT);
#endif

    // set up the LCD's number of columns and rows:
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
    myLCD.print(F("Power meter " VERSION_EXAMPLE));

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    /*
     * Underscores at date and baud are to avoid autoscaling Serial Plotter window with these values
     */
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__ "_"));

    if (sMCUSRStored & (1 << WDRF)) {
        Serial.println(F("Reset by watchdog"));
        myLCD.setCursor(0, 1);
        myLCD.print(F("Res. by watchdog"));
        delay(2000); // to show the message
    }
    Serial.println();

    /*
     * Read power correction percentage
     */
    sPowerCorrectionPercentage = eeprom_read_byte(&sPowerCorrectionPercentageEeprom);
    if (sPowerCorrectionPercentage < 10 || sPowerCorrectionPercentage > 200) {
        sPowerCorrectionPercentage = 100;
        eeprom_write_byte(&sPowerCorrectionPercentageEeprom, sPowerCorrectionPercentage);
    }

    if (!digitalReadFast(ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN)) {
        Serial.println(F("Serial plotter mode enabled"));
    } else {
        Serial.println(
                F(
                        "If you connect periodically print pin " STR(ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN) " to ground, Serial plotter data is printed every " STR(MILLISECONDS_BETWEEN_SERIAL_PLOTTER_OUTPUT) " ms"));
#if defined(TIMING_DEBUG)
    Serial.println(            F(                    "Timing output is on pin " STR(TIMING_DEBUG_OUTPUT_PIN)));
#endif
        Serial.println(F("Power correction + pin is " STR(POWER_CORRECTION_PLUS_PIN)));
        Serial.println(F("Power correction - pin is " STR(POWER_CORRECTION_MINUS_PIN)));

        Serial.println(F("LCD data is printed every " STR(MILLISECONDS_BETWEEN_LCD_OUTPUT) " ms"));

        Serial.println(F("Reference line, which can be negative is " STR(LINE_WHICH_CAN_BE_NEGATIVE)));
        Serial.println(F("Line, with 6.6 ms delay is " STR(LINE_WITH_7_MS_DELAY)));
        Serial.println(F("Line, with 13.3 ms delay is " STR(LINE_WITH_13_MS_DELAY)));

        Serial.print(F("Power correction is "));
        Serial.print(sPowerCorrectionPercentage);
        Serial.println(F(" %"));

        Serial.println(
                F(
                        "Waiting for requests to ID " STR(MODBUS_SLAVE_ADDR) " with _" STR(MODBUS_BAUDRATE) "_ baud at pin " STR(MODBUS_RX_PIN) ". Reply / TX is on pin " STR(MODBUS_TX_PIN)));
    }

    printCorrectionPercentageOnLCD();

    Serial.flush();

    // Timer 1 for sample timing runs continuously in Normal mode
    TCCR1A = 0;
    TCCR1B = _BV(CS11); // clock divider is 8
    TIFR1 = 0; // Clear all compare flags
    TCNT1 = 0;

    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE);

    /*
     * Wait 2 seconds to show LCD content and blink 3 times to indicate booting
     */
    for (uint8_t i = 0; i < 3; ++i) {
        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(50);
        digitalWriteFast(LED_BUILTIN, LOW);
        delay(450);
    }
    delay(500);

    Serial.println();
    printRAMInfo(&Serial); // Stack used is 126 bytes

    myLCD.setCursor(0, 1);
    myLCD.print(F("Wait for U at L" STR(LINE_WHICH_CAN_BE_NEGATIVE)));

    delay(2000); // delay to show LCD content

    // Show Reset by Watchdog indicator
    myLCD.setCursor(0, 1);
    if (sMCUSRStored & (1 << WDRF)) {
        myLCD.print('W');
    } else {
        myLCD.print(' ');
    }

    Serial.println();
    Serial.println(F("First waiting for voltage at line" STR(LINE_WHICH_CAN_BE_NEGATIVE)));

    /*
     * Switch Arduino Serial to MODBUS_BAUDRATE and clear bytes from receive buffer
     */
    Serial.println(F("Enable 120 ms watchdog"));
    Serial.println();
    Serial.flush();
    Serial.begin(MODBUS_BAUDRATE);
    while (Serial.available()) {
        Serial.read();
    }
    /*
     * 9600 baud soft serial to Modbus client. For serial from client we use the hardware Serial RX.
     */
    TxToModbusClient.begin(MODBUS_BAUDRATE);

    /*
     * Enable Watchdog of 120 ms - 100 ms at my CPU :-(
     */
    wdt_enable(WDTO_120MS);
}

void loop() {

    TIMING_PIN_HIGH();
//    disableMillisInterrupt(); // Required if called readVoltage(false);. Disable Timer0 (millis()) overflow interrupt.
    /*
     * Read voltage of phase A
     */
    readVoltage(true);
    TIMING_PIN_LOW();

    /*
     * - Change ADC channel, set new timer compare value, and clear all timer compare flags.
     * - Wait for timer
     * - Read current and compute power for 10 ms.
     */
    ADMUX = LINE_WITH_13_MS_DELAY | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE);
    TIFR1 = _BV(OCF1A);  // Clear all timer compare flags

    digitalWriteFast(LED_BUILTIN, LOW);

    uint8_t tIndexOfCurrentToPrint = 0xFF;  // Disable store to array

    bool tPeriodicallyPrintIsEnabled = !digitalReadFast(ENABLE_ARDUINO_PLOTTER_OUTPUT_PIN); // active if low
    if (tPeriodicallyPrintIsEnabled) {
        tIndexOfCurrentToPrint = sIndexOfCurrentToPrint;
    }

    loop_until_bit_is_set(TIFR1, OCF1A); // Wait for timer
    TIMING_PIN_HIGH();
    /*
     * Read current values and compute power of phase C
     */
    int32_t tPowerRaw = readCurrentRaw(tIndexOfCurrentToPrint == LINE_WITH_13_MS_DELAY); // 3.396 ms
    TIMING_PIN_LOW();

    digitalWriteFast(LED_BUILTIN, HIGH); // To signal, that loop is still running
    /*
     * - Change ADC channel, set new timer compare value, and clear all timer compare flags.
     * - Wait for timer
     * - Read current and compute power for 10 ms.
     */
    ADMUX = LINE_WITH_7_MS_DELAY | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE);
    OCR1A = OCR1A + (2 * 13333); // set next compare to 13333 us after last compare. Timer has a resolution of 0.5 us.
    TIFR1 = _BV(OCF1A);  // Clear all timer compare flags
    // Store values for with 13.3 ms delay now
    int16_t tPower;
    if (sPowerCorrectionPercentage == 100) {
        tPower = (tPowerRaw + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    } else {
        tPower = (((tPowerRaw / 100) * sPowerCorrectionPercentage) + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    }
    digitalWriteFast(LED_BUILTIN, LOW); // To signal, that loop is still running

    sPowerForLCDAccumulator[LINE_WITH_13_MS_DELAY - 1] += tPower;
    sPowerForModbusAccumulator[LINE_WITH_13_MS_DELAY - 1] += tPower;
    sEnergyAccumulator[LINE_WITH_13_MS_DELAY - 1] += tPower;
    sEnergyAccumulatorSumForFlash += tPower;
    loop_until_bit_is_set(TIFR1, OCF1A);

    TIMING_PIN_HIGH(); // 3.34 ms
    /*
     * Read current values and compute power of phase B
     */
    tPowerRaw = readCurrentRaw(tIndexOfCurrentToPrint == LINE_WITH_7_MS_DELAY);
    TIMING_PIN_LOW();

    // prepare for next line reading
    ADMUX = LINE_WHICH_CAN_BE_NEGATIVE | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE);
    ADCSRA = ((1 << ADEN) | (1 << ADIF) | ADC_PRESCALE32);
    OCR1A = OCR1A + (2 * 13333); // set next compare to 13333 us after last compare. Timer has a resolution of 0.5 us.
    TIFR1 = _BV(OCF1A);  // Clear all timer compare flags

    // Store values for line with 6.6 ms delay now
    if (sPowerCorrectionPercentage == 100) {
        tPower = (tPowerRaw + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    } else {
        tPower = (((tPowerRaw / 100) * sPowerCorrectionPercentage) + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    }

    sPowerForLCDAccumulator[LINE_WITH_7_MS_DELAY - 1] += tPower;
    sPowerForModbusAccumulator[LINE_WITH_7_MS_DELAY - 1] += tPower;
    sEnergyAccumulator[LINE_WITH_7_MS_DELAY - 1] += tPower;
    sEnergyAccumulatorSumForFlash += tPower;

    // Do it here and not after the last reading
    sNumberOfPowerSamplesForLCD++;
    sNumberOfPowerSamplesForModbus++;
    sNumberOfEnergySamplesForLCD++;
    if (sCounterForDisplayFreeze > 0) {
        sCounterForDisplayFreeze--;
    }
    sLCDLoopCounter++;
    checkPowerCorrectionPins();

    loop_until_bit_is_set(TIFR1, OCF1A);

    TIMING_PIN_HIGH(); // 3.34 ms
    /*
     * Read current values and compute power of reference phase A
     */
    tPowerRaw = readCurrentRaw(tIndexOfCurrentToPrint == LINE_WHICH_CAN_BE_NEGATIVE);
    /*
     * Read negative half wave phase A, since this is the channel, where we may sell power
     */
    tPowerRaw -= readCurrentRaw(tIndexOfCurrentToPrint == 0); // negative half wave of reference phase A is always stored in 0
    TIMING_PIN_LOW();

    /*
     * fast actions
     */
    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE); // prepare for next reading voltage
    enableMillisInterrupt(DURATION_OF_ONE_MEASUREMENT_MILLIS); // compensate for 60 ms of ADC reading

    /*
     * Reset watchdog here
     */
    wdt_reset();

    /*
     * 60 ms of measurement are gone now, do computing and slow actions
     */
    if (sPowerCorrectionPercentage == 100) {
        tPower = (tPowerRaw + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    } else {
        tPower = (((tPowerRaw / 100) * sPowerCorrectionPercentage) + (POWER_SCALE_DIVISOR / 2)) / POWER_SCALE_DIVISOR; // shift by 15 -> 12 us
    }

    sPowerForLCDAccumulator[LINE_WHICH_CAN_BE_NEGATIVE - 1] += tPower;
    sPowerForModbusAccumulator[LINE_WHICH_CAN_BE_NEGATIVE - 1] += tPower;
    sEnergyAccumulator[LINE_WHICH_CAN_BE_NEGATIVE - 1] += tPower;
    sEnergyAccumulatorSumForFlash += tPower;

    /*
     * Check for 2. flash, indicating negative energy
     */
    if (sWattHourFlashCounter > 0) {
        sWattHourFlashCounter--;
        if (sWattHourFlashCounter == 0) {
            digitalWriteFast(LED_BUILTIN, HIGH); // Flash again for 30 ms
        }
    }

    /*
     * Check for next watt-hour
     */
    if (sEnergyAccumulatorSumForFlash > ENERGY_DIVISOR) {
        sEnergyAccumulatorSumForFlash -= ENERGY_DIVISOR;
        digitalWriteFast(LED_BUILTIN, HIGH); // one 30 ms flash
    } else if (sEnergyAccumulatorSumForFlash < -ENERGY_DIVISOR) {
        sEnergyAccumulatorSumForFlash += ENERGY_DIVISOR;
        sWattHourFlashCounter = 2; // 2 30 ms flashes on negative energy
        digitalWriteFast(LED_BUILTIN, HIGH);
    }

    /*
     * Here we have a new set of values.
     * Start computing, communication LCD output and optional Serial Plotter output.
     * This may last up to 20 ms, since after 20 ms the L1 voltage starts again with the positive half wave.
     */

    /*
     * Handle periodical print request.
     * Must be before printDataOnLCD, because this resets the power value
     */
    if (tPeriodicallyPrintIsEnabled) {
        checkAndPrintInputSignalValuesForArduinoPlotter();
        handlePageButtonPressForArduinoPlotterLineSelect();
    }

    /*
     * Check if we have a modbus request
     * Enable fast response to button press
     */
    if (!checkAndReplyToModbusRequest() || sPageButtonJustPressed) {
        if (!(PageButtonAtPin3.readDebouncedButtonState() && sLCDDisplayPage == POWER_METER_PAGE_INFO)) {
            /*
             * Here no long press at page Energy!
             * If no reply was sent -which took 18 ms-, we have time to print on LCD which takes 3.4 ms
             */
            if (sPageButtonJustPressed || millis() - sMillisOfLastLCDOutput >= MILLISECONDS_BETWEEN_LCD_OUTPUT) {

                sPageButtonJustPressed = false;
                sMillisOfLastLCDOutput = millis(); // set for next check
                printDataOnLCD(); // 3.4 ms
            }
        }
    }

    if (sLCDDisplayPage == POWER_METER_PAGE_INFO
            && PageButtonAtPin3.checkForLongPress(LONG_PRESS_BUTTON_DURATION_MILLIS) == EASY_BUTTON_LONG_PRESS_DETECTED) {
        /*
         * Long press detected at page Energy.
         * Reset power to 0
         * Because page was switched to POWER_METER_PAGE_INFO by press we must set page back to energy here.
         */
        sLCDDisplayPage = POWER_METER_PAGE_ENERGY;
        sNumberOfEnergySamplesForLCD = 0;
        sEnergyAccumulator[0] = 0;
        sEnergyAccumulator[1] = 0;
        sEnergyAccumulator[2] = 0;
    }

//    delay(100); // to test watchdog
}

/*
 * Changes the power correction by 1% if button pressed.
 *
 * Is called every 80 ms seconds
 * For continuous press, autorepeat 2/s is entered after 2 seconds.
 */
void checkPowerCorrectionPins() {
    bool tMinusActivated = !digitalReadFast(POWER_CORRECTION_MINUS_PIN);
    bool tPlusActivated = !digitalReadFast(POWER_CORRECTION_PLUS_PIN);
    if (tMinusActivated || tPlusActivated) {
        if (s80MillisecondsAutorepeatCounter == 0 || s80MillisecondsAutorepeatCounter > 24) {
            /*
             * here one button just gets active or long press for more than 1 second -> change value
             */
            Serial.begin(115200);
            if (tMinusActivated) {
                sPowerCorrectionPercentage -= POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE;
                Serial.print(F("De"));
            }
            if (tPlusActivated) {
                sPowerCorrectionPercentage += POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE;
                Serial.print(F("In"));
            }
            Serial.print(F("crement power correction to "));
            Serial.println(sPowerCorrectionPercentage, 2);
            // Write value to EEPROM
            eeprom_write_byte(&sPowerCorrectionPercentageEeprom, sPowerCorrectionPercentage);

            s80MillisecondsAutorepeatCounter = 12; // If long press, then next change in 1000 ms

            /*
             * Force display of power page
             */
            sLCDDisplayPage = POWER_METER_PAGE_POWER;
            myLCD.setCursor(8, 1);
            myLCD.print(F("Cor="));
            if (sPowerCorrectionPercentage < 100) {
                myLCD.print(' ');
            }
            myLCD.print(sPowerCorrectionPercentage);
            myLCD.print('%');
            Serial.flush();
            Serial.begin(MODBUS_BAUDRATE);

            sCounterForDisplayFreeze = LOOPS_OF_CORRECTION_MESSAGE_DISPLAY_FREEZE; // 3 seconds
        }
        s80MillisecondsAutorepeatCounter++;

    } else {
        /*
         * No button pressed here
         */
        s80MillisecondsAutorepeatCounter = 0;
    }
}

/*
 * Around 250 to 350 ms depending on the size of numbers to send
 */
void checkAndPrintInputSignalValuesForArduinoPlotter() {
    if (millis() - sMillisOfLastSerialPlotterOutput >= MILLISECONDS_BETWEEN_SERIAL_PLOTTER_OUTPUT) {
        sMillisOfLastSerialPlotterOutput = millis(); // set for next check
        Serial.begin(115200);
        Serial.print(F("Voltage Current_x_10_Line_"));
        auto tIndexOfCurrentToPrint = sIndexOfCurrentToPrint;
        Serial.print(tIndexOfCurrentToPrint);
        if (tIndexOfCurrentToPrint == 0) {
            tIndexOfCurrentToPrint = LINE_WHICH_CAN_BE_NEGATIVE; // 0 is negative of reference line
        }
        Serial.print(F(" Power_of_Line"));
        Serial.print(tIndexOfCurrentToPrint);
        Serial.print(F("_is_"));
        Serial.println(sPowerForLCDAccumulator[tIndexOfCurrentToPrint - 1] / sNumberOfPowerSamplesForLCD);

        for (unsigned int i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; ++i) {
            Serial.print(sVoltageArray[i]);
            Serial.print(' ');
            Serial.println(sCurrentArray[i] * 10);
        }
        Serial.println();
        Serial.flush();
        Serial.begin(MODBUS_BAUDRATE);
    }
}

void handlePageButtonPressForArduinoPlotterLineSelect() {
    if (sPageButtonJustPressed) {
        sPageButtonJustPressed = false;
        myLCD.setCursor(8, 1);
        myLCD.print(F(" Index="));
        myLCD.print(sIndexOfCurrentToPrint);
        sCounterForDisplayFreeze = LOOPS_OF_CORRECTION_MESSAGE_DISPLAY_FREEZE; // 3 seconds
    }
}

void printCorrectionPercentageOnLCD() {
    myLCD.setCursor(0, 1);
    myLCD.print(F("Correction="));
    if (sPowerCorrectionPercentage < 100) {
        myLCD.print(' ');
    }
    myLCD.print(sPowerCorrectionPercentage);
    myLCD.print(F("% "));
}

void print6DigitsWatt(int aWattToPrint) {
    sprintf_P(sStringBufferForLCDRow, PSTR("%6d W"), aWattToPrint); // force use of 6 columns
    myLCD.print(sStringBufferForLCDRow);
}

/*
 * 11 ms. 6 ms with delayMicroseconds(40); and 3.4 ms with delayMicroseconds(2) instead of delayMicroseconds(100);   // commands need > 37us to settle
 * Called every MILLISECONDS_BETWEEN_LCD_OUTPUT (320 ms)
 */
void printDataOnLCD() {
    myLCD.setCursor(0, 0);

    if (sLCDDisplayPage == POWER_METER_PAGE_POWER) {
        if (sNumberOfPowerSamplesForLCD > 0) {
            print6DigitsWatt(sPowerForLCDAccumulator[0] / sNumberOfPowerSamplesForLCD);
            print6DigitsWatt(sPowerForLCDAccumulator[1] / sNumberOfPowerSamplesForLCD);

            // Do not overwrite Reset by Watchdog indicator
            myLCD.setCursor(1, 1);
            sprintf_P(sStringBufferForLCDRow, PSTR("%5d W"), sPowerForLCDAccumulator[2] / sNumberOfPowerSamplesForLCD); // force use of 5 columns
            myLCD.print(sStringBufferForLCDRow);
            if (sCounterForDisplayFreeze == 0) {
                int16_t tPowerSum = (sPowerForLCDAccumulator[0] + sPowerForLCDAccumulator[1] + sPowerForLCDAccumulator[2])
                        / sNumberOfPowerSamplesForLCD;
                print6DigitsWatt(tPowerSum);
            }

            // Clear accumulator after printing
            sNumberOfPowerSamplesForLCD = 0;
            sPowerForLCDAccumulator[0] = 0;
            sPowerForLCDAccumulator[1] = 0;
            sPowerForLCDAccumulator[2] = 0;
        }

    } else if (sLCDDisplayPage == POWER_METER_PAGE_ENERGY) {
        /*
         * ENERGY_DIVISOR is 45000 so we have only 16 bit resolution after division
         */
        int16_t tEnergyL1 = sEnergyAccumulator[0] / ENERGY_DIVISOR;
        int16_t tEnergyL2 = sEnergyAccumulator[1] / ENERGY_DIVISOR;
        sprintf_P(sStringBufferForLCDRow, PSTR("%6dWh%6dWh"), tEnergyL1, tEnergyL2); // force use of 6 columns
        myLCD.print(sStringBufferForLCDRow);

        // Second line. L3 and Sum energy
        int16_t tEnergyL3 = sEnergyAccumulator[2] / ENERGY_DIVISOR;
        if (sCounterForDisplayFreeze == 0) {
            int16_t tEnergySum = tEnergyL1 + tEnergyL2 + tEnergyL3;
            sprintf_P(sStringBufferForLCDRow, PSTR("%6dWh%6dWh"), tEnergyL3, tEnergySum); // force use of 6 columns
        } else {
            sprintf_P(sStringBufferForLCDRow, PSTR("%5dWh "), tEnergyL3); // left 8 character for message
        }
        myLCD.setCursor(0, 1);
        myLCD.print(sStringBufferForLCDRow);
    } else if (sLCDDisplayPage == POWER_METER_PAGE_INFO) {
        /*
         * Shows time of energy and power correction percentage
         * "9999D23H12M" is 11 bytes long
         */
        if (sLCDInfoPageCounter == 0) {
            sLCDInfoPageCounter = 60;
            uint32_t tEnergyMinutes = sNumberOfEnergySamplesForLCD / LOOPS_PER_MINUTE;
            sprintf_P(sStringBufferForLCDRow, PSTR("Time %4uD%02uH%02uM"), (uint16_t) (tEnergyMinutes / (60 * 24)),
                    (uint16_t) ((tEnergyMinutes / 60) % 24), (uint16_t) tEnergyMinutes % 60);
            myLCD.print(sStringBufferForLCDRow);

            printCorrectionPercentageOnLCD();
        }
        sLCDInfoPageCounter--; // To update info page only once every 60 counts, i.e. 20 seconds

    } else {
        /*
         * Page POWER_METER_PAGE_EXTRA
         * Shows microseconds of mains period
         */
        // Mains period as measured by timer 1
        myLCD.print(F("Period = ")); // micro sign
        myLCD.print((uint32_t) (sDeltaTCNT1 / 2) + (256L * 256L)); // We have one overflow in timer each 64 ms. 5 digits.
        myLCD.print(F("\xE4s")); // micro sign
    }
}

/*
 * 1.1 V at 30 A gives a resolution of 30 mA => 7 W @ 230 V
 * It seems, that the receive interrupt does not disturb the timing :-)
 * @return sum of 384 times (current * voltage) from sVoltageArray
 */
uint32_t readCurrentRaw(bool aStoreInArray) {
//    digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH);
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | ADC_PRESCALE32);
    uint32_t tPower = 0; // Theoretical maximum value is NUMBER_OF_SAMPLES_FOR_10_MILLIS * 1023 * 1023 = < 2^29 with sine voltage < 2^28
    /*
     * Now read 384 samples. Each loop lasts 26 us.
     */
    for (unsigned int i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; i++) {
        loop_until_bit_is_set(ADCSRA, ADIF);
//        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW);

        ADCSRA |= _BV(ADIF); // clear bit to enable recognizing next conversion has finished with "loop_until_bit_is_set(ADCSRA, ADIF)".
        uint16_t tValue = ADCL | (ADCH << 8); // using WordUnionForADCUtils does not save space here
        // tValue = (ADCH << 8) | ADCL; // this does NOT work!
        if (aStoreInArray) {
            sCurrentArray[i] = tValue; // store value at current counter index
        }
        tPower += (uint32_t) tValue * sVoltageArray[i];
        // 3 us after loop_until_bit_is_set() until here, if aStoreInArray is false
//        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH);
//        if (i % 16 == 0) {
//            Serial.print(F("i="));
//            Serial.print(i);            Serial.print(F(" P="));
//            Serial.print(tPower);
//            Serial.print(F(" 0x"));
//            Serial.print(tPower, HEX);
//            Serial.println();
//        }
    }
    ADCSRA = ((1 << ADEN) | (1 << ADIF) | ADC_PRESCALE32); // Disable auto-triggering (free running mode), but the last conversion is still running
//    digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW);
//    Serial.print(F("PEnd="));
//    Serial.print(tPower);
//    Serial.print(F(" 0x"));
//    Serial.print(tPower, HEX);
//    Serial.println();
    return tPower;
}

/*
 * Store voltage in array. sVoltageArray[0] contains the first non zero value
 * 26 us conversion time / 38.46 kHz
 */
void readVoltage(bool aDoFindZeroCrossing) {

//  ADCSRB = 0; // free running mode  - not required, since it is default
// use ADC_PRESCALE32 which gives 26 us conversion time / 38.46 kHz and good linearity
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | ADC_PRESCALE32);

    uint16_t tValue;
    bool tDoSearchForStart = false;
    unsigned int tCounter = 0;

    while (true) {
        loop_until_bit_is_set(ADCSRA, ADIF);
        ADCSRA |= _BV(ADIF); // clear bit to enable recognizing next conversion has finished with "loop_until_bit_is_set(ADCSRA, ADIF)".
        tValue = ADCL | (ADCH << 8); // using WordUnionForADCUtils does not save space here

        if (aDoFindZeroCrossing) {
            /*
             * Check for negative half wave, which mean we have 16 consecutive zero values
             */
            if (tValue == 0) {
                tCounter++;
                if (tCounter >= 16) {
                    aDoFindZeroCrossing = false;
                    tCounter = 0;
                    tDoSearchForStart = true; // Enable next step: Check for 3 consecutive NON zero values
                }
            } else {
                // reset counter
                tCounter = 0;
            }
        } else {
            /*
             * Store value at current counter index
             */
            sVoltageArray[tCounter++] = tValue;

            if (tDoSearchForStart) {
                /*
                 * Check for 3 consecutive NON zero values
                 */
                if (tValue != 0) {
                    if (tCounter >= 3) {
                        sDeltaTCNT1 = TCNT1 - sLastTCNT1;
                        sLastTCNT1 = TCNT1;
                        OCR1A = TCNT1 + ((2 * 13333) - (3 * 26)); // set next compare to 13333 us after start of data. Timer has a resolution of 0.5 us.
                        disableMillisInterrupt(); // disable it here to have it exact 60 ms disabled
                        tDoSearchForStart = false;
                    }
                } else {
                    // reset counter, overwrite array content
                    tCounter = 0;
                }
            }
            if (tCounter >= NUMBER_OF_SAMPLES_FOR_10_MILLIS) {
                break;
            }
        }
    }

    ADCSRA = ((1 << ADEN) | (1 << ADIF) | ADC_PRESCALE32); // Disable auto-triggering (free running mode), but the last conversion is still running
}

void printPaddedHexOnMyLCD(uint8_t aHexByteValue) {
    if (aHexByteValue < 0x10) {
        myLCD.print('0'); // leading 0
    }
    myLCD.print(aHexByteValue, HEX);
}

uint16_t swap(uint16_t aWordToSwapBytes) {
    return ((aWordToSwapBytes << 8) | (aWordToSwapBytes >> 8));
}

uint32_t swap(uint32_t aLongToSwapBytes) {
    return ((aLongToSwapBytes << 24) | ((aLongToSwapBytes & 0xFF00) << 8) | ((aLongToSwapBytes >> 8) & 0xFF00)
            | (aLongToSwapBytes >> 24));
}

/*
 * Requested by Sofar Inverter with
 * 01 03  00 06  00 01  64 0B
 */
void replyCurrentTransformerRatio() {
    sModbusRTUReplyUnion.ModbusRTUMinimalReply.ReplyByteLength = 2; // Length of 1 word
    // write swapped value. Assume CurrentTransformerRatio as 40 (for CT's of 200A/5A ?)
    sModbusRTUReplyUnion.ModbusRTUMinimalReply.SwappedFirstRegisterContent = (40 << 8);
//    sReplyBuffer[sReplyBufferIndex++] = 0x00; // CRC is constant here
//    sReplyBuffer[sReplyBufferIndex++] = 0x00;

    uint16_t tCRC = calculateCrc(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_ONE_WORD_LENGTH - 2);
    sModbusRTUReplyUnion.ModbusRTUMinimalReply.CRC = swap(tCRC);
    Serial.print(F("CRC=0x"));
    Serial.println(tCRC, HEX);
    TxToModbusClient.write(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_ONE_WORD_LENGTH); // blocking write of 18 ms
}

/*
 * Requested by Sofar Inverter with
 * 01 03  10 1E  00 0C  21 09
 */
void replyTotalActiveEnergy() {
    LongUnion tPower;
    sModbusRTUReplyUnion.ModbusRTU3FloatReply.ReplyByteLength = 24; // Length of 6 float values

    for (uint_fast8_t i = 0; i < 3; ++i) {
        /*
         * Convert to little endian float and copy it to big endian buffer
         */
        tPower.Float = (float) ((sPowerForModbusAccumulator[i] / sNumberOfPowerSamplesForModbus));
        tPower.Float /= 1000; // At address 15 1E Deye expects float power values in kW units
        tPower.ULong = swap(tPower.ULong); // reverse bytes. We must send MSB . . LSB
        sModbusRTUReplyUnion.ModbusRTU3FloatReply.SwappedRegisterContent[i] = tPower.Float;
    }
    uint16_t tCRC = calculateCrc(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH - 2);
    //            Serial.print(F("CRC=0x"));
    //            Serial.println(tCRC, HEX);
    sModbusRTUReplyUnion.ModbusRTU3FloatReply.CRC = swap(tCRC);
    TxToModbusClient.write(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH); // blocking write of 18 ms
    sNumberOfPowerSamplesForModbus = 0;
    sPowerForModbusAccumulator[0] = 0;
    sPowerForModbusAccumulator[1] = 0;
    sPowerForModbusAccumulator[2] = 0;
}

/*
 * Sofar Request frame
 * 01 03  20 14  00 06  8E 0C -> 01 03 0C  00 00 00 00  00 00 00 00  00 00 00 00  CRCH CRCL
 * At address 20 14 DTSU666 sends float power values in 0.1 W units
 */
void replyPower0Point1W() {
    LongUnion tPower;

    sModbusRTUReplyUnion.ModbusRTU3FloatReply.ReplyByteLength = 12; // Length of 3 float values

    for (uint_fast8_t i = 0; i < 3; ++i) {
        /*
         * Convert to little endian float and copy it to big endian buffer
         */
        tPower.Float = (float) ((sPowerForModbusAccumulator[i] / sNumberOfPowerSamplesForModbus));
        tPower.Float *= 10; // At address 20 14 DTSU666 sends float power values in 0.1 W units
        tPower.ULong = swap(tPower.ULong); // reverse bytes. We must send MSB . . LSB
        sModbusRTUReplyUnion.ModbusRTU3FloatReply.SwappedRegisterContent[i] = tPower.Float;
    }

    uint16_t tCRC = calculateCrc(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH - 2);
    //            Serial.print(F("CRC=0x"));
    //            Serial.println(tCRC, HEX);
    sModbusRTUReplyUnion.ModbusRTU3FloatReply.CRC = swap(tCRC);
    TxToModbusClient.write(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH); // blocking write of 18 ms
    sNumberOfPowerSamplesForModbus = 0;
    sPowerForModbusAccumulator[0] = 0;
    sPowerForModbusAccumulator[1] = 0;
    sPowerForModbusAccumulator[2] = 0;
}

/*
 * Deye Request frame 8 ms every 100 ms -> Reply frame 18 ms
 * 01 03  15 1E  00 06  A1 C2 -> 01 03 0C  00 00 00 00  00 00 00 00  00 00 00 00  93 70
 * At address 15 1E Deye expects float power values in kW units
 */
void replyPower() {
    LongUnion tPower;
    sModbusRTUReplyUnion.ModbusRTU3FloatReply.ReplyByteLength = 12; // Length of 3 float values

    for (uint_fast8_t i = 0; i < 3; ++i) {
        /*
         * Convert to little endian float and copy it to big endian buffer
         */
        tPower.Float = (float) ((sPowerForModbusAccumulator[i] / sNumberOfPowerSamplesForModbus));
        tPower.Float /= 1000; // At address 15 1E Deye expects float power values in kW units
        tPower.ULong = swap(tPower.ULong); // reverse bytes. We must send MSB . . LSB
        sModbusRTUReplyUnion.ModbusRTU3FloatReply.SwappedRegisterContent[i] = tPower.Float;
    }
    uint16_t tCRC = calculateCrc(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH - 2);
    //            Serial.print(F("CRC=0x"));
    //            Serial.println(tCRC, HEX);
    sModbusRTUReplyUnion.ModbusRTU3FloatReply.CRC = swap(tCRC);
    TxToModbusClient.write(sModbusRTUReplyUnion.ReplyByteBuffer, MODBUS_REPLY_POWER_LENGTH); // blocking write of 18 ms
    sNumberOfPowerSamplesForModbus = 0;
    sPowerForModbusAccumulator[0] = 0;
    sPowerForModbusAccumulator[1] = 0;
    sPowerForModbusAccumulator[2] = 0;
}

/*
 * Deye Request frame 01 03  15 1E  00 06  A1 C2 with duration 8 ms every 100 ms -> Reply frame 18 ms
 * @return true, if reply was sent (which took 18ms)
 *     sJKFAllReplyPointer = reinterpret_cast<JKReplyStruct*>(&JKReplyFrameBuffer[JK_BMS_FRAME_HEADER_LENGTH + 2
 + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH]]);
 */
bool checkAndReplyToModbusRequest() {
    uint8_t tNumberOfAvaliableBytes = Serial.available();
    if (tNumberOfAvaliableBytes >= MODBUS_REQUEST_LENGTH) {
        Serial.readBytes(sModbusRTURequestUnion.ReceiveByteBuffer, MODBUS_REQUEST_LENGTH);
        if (tNumberOfAvaliableBytes > MODBUS_REQUEST_LENGTH) {
            /*
             * Too many bytes available. Incorrect request, maybe because the Deye sent a request, while we were responding the last time.
             * Clear additional bytes from receive buffer
             */
            while (Serial.available()) {
                Serial.read();
            }

        } else {
            uint8_t tBufferErrorPrintOffset = 0;
            if (sModbusRTURequestUnion.ModbusRTURequest.SlaveAddress == 1
                    && sModbusRTURequestUnion.ModbusRTURequest.FunctionCode == 3) {

                uint16_t tAddress = swap(sModbusRTURequestUnion.ModbusRTURequest.FirstRegisterAddress);
                uint16_t tCRC = swap(sModbusRTURequestUnion.ModbusRTURequest.CRC);
                uint16_t tReplyLength = swap(sModbusRTURequestUnion.ModbusRTURequest.ReplyLengthInWord);
                if (tAddress == 0x151E) {
                    // check other fields of request
                    if (tReplyLength == 6 && tCRC == 0xA1C2) {
                        replyPower();
                        return true;
                    }
                    // Here we have length or CRC Error for request 151E
                    myLCD.setCursor(0, 0);
                    myLCD.print('C'); // "CRC" Error
                    tBufferErrorPrintOffset = 4; // show last 4 bytes of wrong request

                } else if (tAddress == 0x0006) {
                    replyCurrentTransformerRatio();
                    return true;

                } else if (tAddress == 0x2014) {
                    replyPower0Point1W();
                    return true;

                } else if (tAddress == 0x101E) {
                    // 01 03  10 1E  00 0C  21 09
                    // 0x101E positive total active energy / float
                    // 0x1028 negative total active energy / float
                    replyTotalActiveEnergy();
                    return true;

                }
                // every case with no return will show the error line
            }
            /*
             * incorrect request of right length
             * I have seen 0x3E and 0xFE instead of 01 03 15 1E
             * -> Show error, but send power anyway
             */
            myLCD.setCursor(8, 1);
            // print first or last 4 bytes of request dependent of tBufferErrorPrintOffset
            for (uint_fast8_t i = 0; i < 4; ++i) {
                printPaddedHexOnMyLCD(sModbusRTURequestUnion.ReceiveByteBuffer[i + tBufferErrorPrintOffset]);
            }

            sCounterForDisplayFreeze = 1440; // 12 per second => 1440 is 2 minutes
            replyPower();

            return true;
        }
    }
    return false;
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

// Currently unused
void printPower() {
    for (uint_fast8_t i = 0; i < 3; ++i) {
        Serial.print('L');
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(sPowerForLCDAccumulator[i] / sNumberOfPowerSamplesForLCD);
        Serial.print(F(" W "));
        if (i != 2) {
            Serial.print(F("+ "));
        }
    }
    Serial.print(F(" = "));
    Serial.print(sPowerSum);
    Serial.print(F(" W"));
    Serial.println();
}
