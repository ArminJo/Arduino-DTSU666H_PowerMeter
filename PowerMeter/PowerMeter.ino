/**
 * PowerMeter.cpp
 *
 *  Measuring is done with an Arduino Nano and 10A Current Transformer.
 *
 *  On a 2004 LCD the program displays the current power as big number or 4 lines:
 *  1. Current power, average power of last 5 minutes, last hour and last 24 hours.
 *  2. Energy as kWh of last 24 hour, of last day and of day before
 *  3. Negative part of power, power correction percentage
 *  4. Time, clock correction  -  real time after connection with BD App
 *  Example:
 *    94W  167   37   85    94W current power, 167W average power of last 5 minutes, 37W average power of last hour, 85W average power of last 24 hours
 *   2.04kWh  3.58  1.22    Energy of last 24 hours, of last day (from 3 o clock to 3 o clock / FIRST_HOUR_OF_DAY), of day before
 *      0W          100%    0W negative power,  100% scale factor
 *  22:44:24  + 3:46/day    Time, 3:46 faster per day time correction
 *
 *  The power correction percentage can be modified by push buttons at pin 2 and 3.
 *
 *
 *  Chart of the last 2 days with 5 minutes resolution can be displayed on an Android device running the BlueDisplay app.
 *  The clock correction e.g. for Arduino Nano ceramic resonators, can be modified by 2 buttons with the BD App.
 *
 *
 *
 *  Copyright (C) 2026  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-PowerMeter https://github.com/ArminJo/Arduino-PowerMeter.
 *
 *  Arduino-PowerMeter is free software: you can redistribute it and/or modify
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

/*
 * We assume, that the positive and negative voltage and current values are symmetrical.
 * Therefore, we can take the positive half wave as the voltage reference, for example.
 *
 * Program flow:
 * 1. Wait for voltage zero crossing.
 * 2. Read "positive" part voltage values for 10 ms and store 385 values in RAM to be used as reference also for negative part of current.
 * 3. Do 20 ms current measurement, starting with negative current. Positive values are at the last 10 ms. Multiply values with voltage.
 * 4. 10 ms for math and output to LCD until starting again at 40 ms - x ms.
 *
 * In short notation:
 * 0 ms: U phase | 10 ms: I phase A | 20 ms: -I phase | 30 ms: computations, statistics, LCD display etc. | 40 ms: start again
 *
 * Timer 1 is used for a 1 second tick to keep track of correct energy value.
 * The loop not always lasts constant 40 ms. Especially it lasts 110 ms for the cyclic 5 minutes refresh of BlueDisplay.
 *
 * Every watt-hour, the build-in LED flashes for 40 ms / one loop. This is e.g. a flash each 36 seconds at 100 W.
 * During ever loop the LED flashes for a few microseconds as "alive" signal.
 *
 */
#include <Arduino.h>

#define VERSION_EXAMPLE "1.1"

#include "MillisUtils.h"    // For enableMillisInterrupt(), disableMillisInterrupt()
#include "ADCUtils.h"       // For ADC_PRESCALE32 and SHIFT_VALUE_FOR_REFERENCE
#include "AVRUtils.h"       // For initTimeoutWithWatchdog(), initStackFreeMeasurement(), printRAMAndStackInfo()
#include "digitalWriteFast.h"
#include "LiquidCrystal.h"

#define FIRST_HOUR_OF_DAY       3 // Energy accumulation for last days starts at 3 o clock
//#define STANDALONE_TEST // Do not wait for zero crossing of voltage and leave reference voltage at VCC
//#define FAST_CLOCK // Increment seconds by 30 every second
//#define TIMING_DEBUG    // Output Timing waveform at pin A5
//#define ENABLE_STACK_ANALYSIS
//#define TRACE

//#define DO_TRACING // search and print PC of lowest Stackpointer
#if defined(DO_TRACING)
#define SEARCH_LOWEST_STACKPOINTER_MODE // Instead of printing program addresses of traced section, store lowest SP and corresponding PC
#include "AvrTracing.hpp"
#endif

/*
 * Voltage uses 1.1 V range. So we have full range at 399.3 V with divider 3.620 MOhm to 10 kOhm. 620 kOhm is a standard value.
 * => voltage LSB is 390 mV. (Raw * 399.3) / 1023
 *
 * With ZMCT103C and 50 Ohm we get 0.5V at 10 A and full 1.1 V range at 22 A. (22A is peak value for 15.5 A effective value)
 * => current LSB is 21.5 mA  => Power LSB is 8.394 mW.
 * Maximum power value for 15 A and 230 V is: 3.45 kW
 *
 * We sum 384 samples per measurement so here we have 8.4 mW / 384 = 0.02186 mW per sample LSB. 1000 mW / 0.0298 mW = 45747.
 *
 * To compensate for to little readings at low current, we connect a 1 MOhm resistor and a 1 MOhm potentiometer from A1 to 5V
 * and adjust it so that the negative power remains at 0W when there is no current.
 * This offsets the quantization of 5 Watt per current LSB at 230 Volt.
 */
#if !defined(POWER_SCALE_DIVISOR_FOR_1_WATT)
#define POWER_SCALE_DIVISOR_FOR_1_WATT          45747 // Computed value for 100%. We have 1 Watt, if we have 45747 as result of readCurrentAndComputeRawPower()
#endif

#if !defined(ADC_CHANNEL_FOR_VOLTAGE)
#define ADC_CHANNEL_FOR_VOLTAGE                 0
#endif
#if !defined(ADC_CHANNEL_FOR_CURRENT)
#define ADC_CHANNEL_FOR_CURRENT                 1
#endif

#define POWER_CORRECTION_PLUS_PIN               2
#define POWER_CORRECTION_MINUS_PIN              3
#define ENABLE_SHOW_BIG_NUMBERS_PIN             4
#define ENABLE_TEST_PIN                         5

#if defined(TIMING_DEBUG)
#define TIMING_DEBUG_OUTPUT_PIN                 A5
#define TIMING_PIN_HIGH()                       digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH)
#define TIMING_PIN_LOW()                        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW)
#else
#define TIMING_PIN_HIGH()                       void()
#define TIMING_PIN_LOW()                        void()
#endif

#define DURATION_OF_ONE_MEASUREMENT_MILLIS      30 // Constant value depending on implementation of loop()! From start of voltage measurement to end of L1 negative current measurement
#define DURATION_OF_ONE_LOOP_MILLIS             40 // Constant value depending on implementation of loop()! One measurement and 10 ms for synchronizing voltage used for communication and print
//uint32_t sSecondsSinceStart = 0; // Incremented each second by timer 1 overflow
#define SECONDS_PER_STORAGE                     300 // 5 Minutes
uint16_t sSecondsSinceLastStorage = 0;
uint16_t sSecondsSinceLastHour = 0;

/*
 * TIMING CORRECTION
 * This count is subtracted from the value 15625 which works for exact 16MHz.
 * Each count is 64 us per second = 5.5 seconds per day
 * Positive value -> 1 s clock will go faster, negative value -> 1 s clock will go slower
 */
int8_t sOneSecondTimer64usCorrectionCount;
EEMEM int8_t sOneSecondTimer64usCorrectionCountEeprom; // Storage in EEPROM for sOneSecondTimer64usCorrectionCount
void init1SecondTimer1();
void set1SecondTimer1Correction();

/*
 * LCD stuff - Pins 7 to 12
 */
//#define USE_PARALLEL_2004_LCD
#include "LCDPrintUtils.hpp" // sets USE_PARALLEL_LCD or USE_SERIAL_LCD
LiquidCrystal myLCD(11, 12, 7, 8, 9, 10); // Pins 7 to 12

/*
 * Big numbers stuff
 */
#include "LCDBigNumbers.hpp" // Include sources for LCD big number generation
LCDBigNumbers BigNumbersLCD(&myLCD, BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_1); // Use 3x4 numbers, 1. variant
bool sShowBigDigits;
void checkShowBigDigitsPin();

bool LCD_checkAndPrintData();                          // Called every MILLISECONDS_BETWEEN_LCD_OUTPUT

uint8_t sLCDInfoPageCounter;                    // To update Info page only once every 20.5 seconds

#define DELTA_WATT_FOR_DISPLAY_REFRESH      5   // Display is refreshed if low pass filtered watt changes more or equal than this delta
/*
 * For power data acquisition
 * with 4kW (17A) maximum we get a maximum of 32,000 every 480 ms
 */
#define POWER_SAMPLES_BETWEEN_LCD_OUTPUT    8   // for LCD we accumulate 8 periods, which is 480 ms
uint8_t sCounterOfPowerSamplesForLCD;
uint16_t sPowerAccumulatorWatt;                 // Maximum is 8.191 kW for 8 samples and 7.281 kW for 9 samples per display period
uint16_t sNegativePowerAccumulatorWatt;         // After 8 (POWER_SAMPLES_BETWEEN_LCD_OUTPUT) samples it is displayed an reseted.
// Next 3 are signed because they are EMA filtered
int16_t sPowerAverageWatt;                     // = sPowerAccumulatorWatt / sCounterOfPowerSamplesForLCD
int16_t sPowerAverageWattLowpass;               // For BD output
int16_t sLastDisplayedPowerAverageWatt;

uint16_t sNumberOfPowerSamplesFor5Minutes;      // from 0 to (5 * 60 * 25) 7500
uint32_t sPowerFor5MinutesAccumulatorWatt;      // one sample every 40 ms
uint16_t sPowerAverageOfLast5MinutesWatt;
uint16_t sNumberOfPowerSamplesFor1Hour;
uint16_t sPowerFor1HourAccumulatorWatt;         // 12 samples, 1 sample every 5 minutes
uint16_t sPowerAverageOfLastHourWatt;
uint16_t sPowerAverageOfLast24HoursWatt;

uint32_t sEnergyAccumulatorFor1WattHourFlash;
#define ENERGY_ACCUMULATOR_1_WATT_HOUR  ((SECONDS_IN_ONE_HOUR * 1000UL) / DURATION_OF_ONE_LOOP_MILLIS) // 90,000 if sEnergyAccumulatorFor1WattHourFlash contains this value, we have 1 Wh

/*
 * Use 64 bit value here, since the divisor for Wh is 45000 and we want to see more than 47 kWh at page ENERGY
 */
//uint64_t sTotalEnergyAccumulator;          // Contains sum of sNumberOfSamplesForEnergy entries of power
//uint32_t sNumberOfTotalEnergySamples;
/*
 * Power correction
 */
uint8_t sPowerCorrectionPercentage = 100;
EEMEM uint8_t sPowerCorrectionPercentageEeprom;    // Storage in EEPROM for sPowerCorrectionPercentage

bool sPowerCorrectionButtonIsActive;
#define POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE      1 // One percent. The value to add or subtract to sPowerCorrectionFloat at each correction button press
void checkPowerCorrectionPins();

/*
 * BlueDisplay stuff
 */
//#define BLUETOOTH_BAUD_RATE BAUD_115200   // Activate this, if you have reprogrammed the HC05 module for 115200
#if !defined(BLUETOOTH_BAUD_RATE)
#define BLUETOOTH_BAUD_RATE     9600    // Default baud rate of my HC-05 modules, which is not very reactive
#endif

#define NUMBER_OF_SAMPLES_FOR_10_MILLIS 384U // 0x180 10000 us / (26 us / sample) = 384.6153846153846
#include "PowerLoggerAndChart.hpp"

void readVoltage(bool aDoFindZeroCrossing);
uint32_t readCurrentAndComputeRawPower(bool aStoreInArray = false);

void printStartupInfo();
bool checkAndProcessSeconds();

void setup() {
    /*
     * Disable watchdog for setup
     */
    wdt_disable();

    // Initialize the digital pin as an output and set it to LOW
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(POWER_CORRECTION_PLUS_PIN, INPUT_PULLUP);
    pinMode(POWER_CORRECTION_MINUS_PIN, INPUT_PULLUP);
    pinMode(ENABLE_SHOW_BIG_NUMBERS_PIN, INPUT_PULLUP);
    pinMode(ENABLE_TEST_PIN, INPUT_PULLUP);

#if defined(TIMING_DEBUG)
    pinMode(TIMING_DEBUG_OUTPUT_PIN, OUTPUT);
#endif

    // set up the LCD's number of columns and rows:
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
    myLCD.print(F("Power Meter " VERSION_EXAMPLE));

    BigNumbersLCD.begin(); // This also sets cursor to 0.0

#if defined(ESP32)
    Serial.begin(115200);
    initSerial("ESP-BD_Example");
    Serial.println("Start ESP32 BT-client with name \"ESP-BD_Example\"");
#else
    initSerial(BLUETOOTH_BAUD_RATE); // converted to Serial.begin(BLUETOOTH_BAUD_RATE);
#endif

#if defined(SEARCH_LOWEST_STACKPOINTER_MODE)
    initTrace();
#endif

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    /*
     * Underscores at date are to avoid auto scaling Serial Plotter window with these values
     */
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__ "_"));
    Serial.println();

    /*
     * Read power correction percentage
     */
    sPowerCorrectionPercentage = eeprom_read_byte(&sPowerCorrectionPercentageEeprom);
    if (sPowerCorrectionPercentage < 10 || sPowerCorrectionPercentage > 200) {
        sPowerCorrectionPercentage = 100;
        eeprom_write_byte(&sPowerCorrectionPercentageEeprom, sPowerCorrectionPercentage);
    }

    printStartupInfo();

    sOneSecondTimer64usCorrectionCount = eeprom_read_byte((uint8_t*) &sOneSecondTimer64usCorrectionCountEeprom);
    Serial.print(F("1 second clock correction is "));
    Serial.print(sOneSecondTimer64usCorrectionCount);
    Serial.print(F(" * 64 us per second = "));
    Serial.print(((long) SECONDS_IN_ONE_DAY * sOneSecondTimer64usCorrectionCount) / 15625);
    Serial.println(F(" s per day faster"));

    Serial.flush();

#if defined(STANDALONE_TEST)
    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE); // Leave reference at 5 V
#else
    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE); // Set reference to 1.1 volt
#endif

    /*
     * Wait 2 seconds to show LCD content and blink 34 times to indicate booting
     * and to show "Power meter " VERSION_EXAMPLE for 2 seconds
     */
    for (uint8_t i = 0; i < 4; ++i) {
        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(50);
        digitalWriteFast(LED_BUILTIN, LOW);
        delay(450);
    }

    Serial.println();
#if defined(ENABLE_STACK_ANALYSIS)
    printRAMAndStackInfo(&Serial); // Stack used is 126 bytes
#endif

    LCDClearLine(&myLCD, 1);
    myLCD.print(F("Wait for U"));

    delay(2000); // delay to show LCD content

    Serial.println();
    Serial.println(F("Enable 8 s watchdog and waiting for voltage at ADC channel " STR(ADC_CHANNEL_FOR_VOLTAGE)));
    Serial.println();

    LCDClearLine(&myLCD, 1);

    InitPowerLoggerAndChart();  // Do it after all delays, to get timestamp synchronized
    handleEventAndFlags();      // Handle the init Display event set by InitPowerLoggerAndChart()

    /*
     * Enable Watchdog of 1 s
     */
    initTimeoutWithWatchdog(WDTO_1S);

    init1SecondTimer1();

#if defined(FAST_CLOCK)
    handleEventAndFlags(); // process event, which sets sysTime
    sysTime = (sysTime / 30) * 30; // round to full 30 seconds, otherwise sysTime % STORAGE_INTERVAL_SECONDS) == 0 is never true
#endif
}

/*
 * Loop timing is 40 ms 25 Hz by default
 */
void loop() {

    TIMING_PIN_HIGH(); // 10 to 20 ms high, last 10 ms are measurement, first x ms is for is waiting for zero crossing
    /************************************************************
     * Read voltage values to array VoltageArray
     ************************************************************/
    /*
     * Read voltage of phase A
     */
#if defined(STANDALONE_TEST)
    readVoltage(false); // Do not wait for zero crossing of voltage
#else
    readVoltage(digitalRead(ENABLE_TEST_PIN)); // Wait for zero crossing of voltage
#endif
    digitalWriteFast(LED_BUILTIN, LOW); // reset watt hour flash here

    /************************************************************
     * Read current values during negative voltage.
     * If we have negative current, we see positive values here
     * 0.65 us
     ************************************************************/
    // prepare for reading of current
#if defined(STANDALONE_TEST)
    ADMUX = ADC_CHANNEL_FOR_CURRENT | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE); // Leave reference at 5 V
#else
    ADMUX = ADC_CHANNEL_FOR_CURRENT | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE); // Set reference to 1.1 volt
#endif

#if defined(TRACE)
    Serial.print(sCounterOfPowerSamplesForLCD);
    Serial.print(F("*"));
    Serial.print(tPowerInWatt);
    Serial.print(F("="));
    Serial.print(sPowerAccumulatorWatt[0]);
    Serial.print(F(" "));
    Serial.print(tPowerRaw); // 401 867 136
    Serial.print(F(" | "));
#endif

    TIMING_PIN_LOW();
    // 20 ms low
    uint32_t tPowerRawNegative = readCurrentAndComputeRawPower(false); // gives > 0 for negative power. Spurious 1 to 3 seen if not connected.
    // Maximum tPowerRawNegative 401,867,136
    /************************************************
     * Read the half wave phase with positive voltage.
     ************************************************/
#if defined(STANDALONE_TEST)
    uint32_t tPowerRaw = readCurrentAndComputeRawPower(tPeriodicallyPrintIsEnabled); // skip recent reading
#else
    /*
     * Maximum tPowerRaw value is 401,867,136 > 7.340 kW with POWER_SCALE_DIVISOR_FOR_1_WATT = 45747
     * Maximum for uint32 is 93.952 kW with POWER_SCALE_DIVISOR_FOR_1_WATT = 45747
     */
    uint32_t tPowerRaw = readCurrentAndComputeRawPower(sShowRawDataDisplay & ACQUIRE_RAW_DATA_FLAG);
    ADCSRA = ((1 << ADEN) | (1 << ADIF) | ADC_PRESCALE32); // Disable auto-triggering (free running mode), but the last conversion is still running

    /*
     *  POWER_SCALE_DIVISOR_FOR_1_WATT / 2 is a guess to suppress spurious low values if not connected.
     *  if negative and positive values are almost equal we get random positive power, because of adding multiple values
     */
    if (tPowerRaw >= tPowerRawNegative && tPowerRawNegative > (POWER_SCALE_DIVISOR_FOR_1_WATT / 2)) {
        tPowerRaw -= tPowerRawNegative;
    }
#endif
    // 64 us from here to check for seconds processing

    /*
     * 30 ms of measurement are gone now, do computing and statistics
     * Actions:
     * - Set LED to high for alive signal
     * - Set ADC channel back to voltage channel
     * - Enable millis interrupts again
     * - WDT reset
     * - compute watt
     * - Set LED to low for alive signal
     * - Check for next watt-hour for display by LED
     * - Do only one of them per loop:
     *   - Check for display of raw data
     *   - Check for secondly statistics
     *   - Print data on LCD
     *   - Keep GUI responsive
     */
    digitalWriteFast(LED_BUILTIN, HIGH); // To signal, that loop is still running, is
#if defined(STANDALONE_TEST)
    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE); // Leave reference at 5 V
#else
    ADMUX = ADC_CHANNEL_FOR_VOLTAGE | (INTERNAL << SHIFT_VALUE_FOR_REFERENCE); // prepare for next reading of voltage
#endif
    enableMillisInterrupt(DURATION_OF_ONE_MEASUREMENT_MILLIS); // compensate for exact 30 ms of ADC reading

    wdt_reset(); // Reset 1 s watchdog here

    uint16_t tPowerAsWatt;
    if (sPowerCorrectionPercentage == 100) {
        tPowerAsWatt = tPowerRaw / POWER_SCALE_DIVISOR_FOR_1_WATT;
        sNegativePowerAccumulatorWatt += tPowerRawNegative / POWER_SCALE_DIVISOR_FOR_1_WATT;
    } else {
        tPowerAsWatt = ((tPowerRaw / 100) * sPowerCorrectionPercentage) / POWER_SCALE_DIVISOR_FOR_1_WATT;
        sNegativePowerAccumulatorWatt += ((tPowerRawNegative / 100) * sPowerCorrectionPercentage) / POWER_SCALE_DIVISOR_FOR_1_WATT;
    }
    sPowerAccumulatorWatt += tPowerAsWatt; // Maximum is 8.191 kW for 8 samples and 7.281 kW for 9 samples per display period
    sCounterOfPowerSamplesForLCD++;

    sPowerFor5MinutesAccumulatorWatt += tPowerAsWatt;
    sNumberOfPowerSamplesFor5Minutes++;

//    sTotalEnergyAccumulator += tPowerAsWatt;
//    sNumberOfTotalEnergySamples++;

    if (sShowRawDataDisplay & ACQUIRE_RAW_DATA_FLAG) {
        if (sShowRawDataDisplay == ACQUIRE_RAW_DATA_NO_SHIFT && (sMaxCurrent >> 8) != 0) { // if ((sMaxCurrent >> 8) != 0) { // same as (sMaxCurrent >= 0x100) but shorter
        // Here we had overflow while acquiring data with NO shift ->, so try again with shift
            sShowRawDataDisplay = ACQUIRE_RAW_DATA_WITH_SHIFT;
        } else {
            // No overflow here while acquiring data -> show it once
            sShowRawDataDisplay = RAW_DATA_IS_VALID_FOR_DISPLAY; // Data just acquired and no overflow
            sPowerForShowRawData = tPowerAsWatt; // for later display
        }
    }

    /*
     * Check for next watt-hour
     */
    digitalWriteFast(LED_BUILTIN, LOW); // To signal, that loop is still running
    sEnergyAccumulatorFor1WattHourFlash += tPowerAsWatt;
    if (sEnergyAccumulatorFor1WattHourFlash > ENERGY_ACCUMULATOR_1_WATT_HOUR) {
        sEnergyAccumulatorFor1WattHourFlash -= ENERGY_ACCUMULATOR_1_WATT_HOUR;
        digitalWriteFast(LED_BUILTIN, HIGH); // One 40 ms flash, i.e. until voltage reading of next loop
    }

    TIMING_PIN_HIGH();
// 13 us up to 110 ms for BD Display update

    /*
     * Now do the long lasting display parts
     */
    if (sShowRawDataDisplay == RAW_DATA_IS_VALID_FOR_DISPLAY) {
        drawRawDataChart();
        sShowRawDataDisplay = DO_NOT_SHOW_CHART_DATA; // avoid overwriting of raw data

    } else if (!checkAndProcessSeconds()) {
        if (!BD_checkAndPrintChangedPowerValue()) {
#if !defined(SEARCH_LOWEST_STACKPOINTER_MODE)
            checkPowerCorrectionPins();
#endif
            checkShowBigDigitsPin();
            if (!LCD_checkAndPrintData()) { // uses and resets sCounterOfPowerSamplesForLCD
                handleEventAndFlags(); // Keep GUI responsive
            }
        }
    }
    TIMING_PIN_LOW();
#if defined(TIMING_DEBUG)
    delayMicroseconds(10); // 11 us until timing pin high
#endif
}

void printStartupInfo() {
#if defined(STANDALONE_TEST)
    myLCD.setCursor(0, 1);
    myLCD.print(F("Standalone test"));
    delay(2000); // delay to show LCD content
#endif

    Serial.print(F("ADC Channel for voltage is " STR(ADC_CHANNEL_FOR_VOLTAGE)));
    Serial.println(F(", for current is " STR(ADC_CHANNEL_FOR_CURRENT)));
#if defined(TIMING_DEBUG)
    Serial.println(F("Timing output is on pin " STR(TIMING_DEBUG_OUTPUT_PIN)));
#endif
#if !defined(SEARCH_LOWEST_STACKPOINTER_MODE)
    Serial.print(F("Power correction +pin is " STR(POWER_CORRECTION_PLUS_PIN)));
    Serial.println(F(", -pin is " STR(POWER_CORRECTION_MINUS_PIN)));
#endif

    Serial.println(F("LCD data is printed every " STR(MILLISECONDS_BETWEEN_LCD_OUTPUT) " ms"));

    Serial.print(F("Power correction is "));
    Serial.print(sPowerCorrectionPercentage);
    Serial.println(F(" %"));
}

void init1SecondTimer1() {
    /*
     * Set Timer 1 for 1 s timing
     * It runs continuously in CTC mode with clock divider 1024 and OCR1A 15625 - 1 as TOP
     */
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS12); // CTC with OCR1A and clock divider is 1024 -> 15.526 kHz / 64 us.
    TCNT1 = 0; // reset counter to 0
    TIFR1 = 0xFF; // Clear all flags initially
    set1SecondTimer1Correction();
}

void set1SecondTimer1Correction() {
    OCR1A = ((15625 - 1) - sOneSecondTimer64usCorrectionCount); // divider for 1 s
}

/**
 *
 * @return  true if processing was done, false otherwise
 * 40 us minimum, 420 us for writing to array without BlueDisplay update and 110 ms with BlueDisplay update
 */
bool checkAndProcessSeconds() {
    if (TIFR1 & _BV(OCF1A)) {
        /*
         * Now 1 second is gone
         * Check for and compute 5 minutes, 1 hour and 1 day averages
         */
        TIFR1 = _BV(OCF1A);
#if defined(FAST_CLOCK)
//        sSecondsSinceStart += 30;
        sysTime += 30;
#else
//        sSecondsSinceStart++;
        sysTime++;
#endif

        if (millis() < (STORAGE_INTERVAL_SECONDS * MILLIS_IN_ONE_SECOND)) {
            // here we are in the first 5 minutes -> compute an estimate of sPowerAverageOfLast5MinutesWatt
            sPowerAverageOfLast5MinutesWatt = sPowerFor5MinutesAccumulatorWatt / sNumberOfPowerSamplesFor5Minutes;
        }

        if ((sysTime % STORAGE_INTERVAL_SECONDS) == 0) {
//#if !defined(FAST_CLOCK)
//            if (sNumberOfPowerSamplesFor5Minutes != (25 * STORAGE_INTERVAL_SECONDS)) {
//                // It was not exact the number of samples expected for 5 minutes
//                Serial.print(sNumberOfPowerSamplesFor5Minutes);
//                Serial.println(F(" is not the expected 7500 samples for 5 minutes"));
//            }
//#endif
            sPowerAverageOfLast5MinutesWatt = sPowerFor5MinutesAccumulatorWatt / sNumberOfPowerSamplesFor5Minutes;
            sNumberOfPowerSamplesFor1Hour++;
            sPowerFor1HourAccumulatorWatt += sPowerAverageOfLast5MinutesWatt;
            sPowerFor5MinutesAccumulatorWatt = 0;
            sNumberOfPowerSamplesFor5Minutes = 0;

            /*
             * Store to Power array
             * 110 ms, if BlueDisplay connected
             */
            writeToPowerArray(sPowerAverageOfLast5MinutesWatt); //
            sNextStorageSeconds += STORAGE_INTERVAL_SECONDS;

            // Check for first hour since boot
            if (millis() < (SECONDS_IN_ONE_HOUR * MILLIS_IN_ONE_SECOND)) {
                // Here we are in the first hour -> compute an estimate of sPowerAverageOfLastHourWatt
                sPowerAverageOfLastHourWatt = sPowerFor1HourAccumulatorWatt / sNumberOfPowerSamplesFor1Hour;
            }
            if (minute() == 0) {
                /*
                 * Now 1 hour is gone. sNumberOfPowerSamplesFor1Hour == 12
                 */
                sPowerAverageOfLastHourWatt = sPowerFor1HourAccumulatorWatt / sNumberOfPowerSamplesFor1Hour;

                sNumberOfPowerSamplesFor1Hour = 0;
                sPowerFor1HourAccumulatorWatt = 0;

                // Compute Energy per last 24 hour
                uint16_t tEnergyPerLast24HourWattHour = 0;
                sNoinitData.EnergyPerHourWattHour[hour()] = sPowerAverageOfLastHourWatt; // set kWh for this hour
                for (uint16_t i = 0; i < HOURS_IN_ONE_DAY; ++i) {
                    tEnergyPerLast24HourWattHour += sNoinitData.EnergyPerHourWattHour[i];
                }

                if (sNoinitData.EntriesInWattPerHour < HOURS_IN_ONE_DAY) {
                    sNoinitData.EntriesInWattPerHour++;
                    // Here we are in the first day -> compute an estimate of tEnergyPerLast24Hour
                    tEnergyPerLast24HourWattHour = (tEnergyPerLast24HourWattHour * (long) HOURS_IN_ONE_DAY)
                            / sNoinitData.EntriesInWattPerHour;
                }
                sPowerAverageOfLast24HoursWatt = tEnergyPerLast24HourWattHour / HOURS_IN_ONE_DAY;
                if (BlueDisplay1.isConnectionEstablished() && sShowRawDataDisplay == DO_NOT_SHOW_RAW_DATA) {
                    BD_printDaylyEnergyValue();
                }
                if (hour() == FIRST_HOUR_OF_DAY) {
                    // three o clock in the morning, store energy for last days
                    sNoinitData.sEnergyOfLastDaysWattHourPerDay[0] = sNoinitData.sEnergyOfLastDaysWattHourPerDay[1];
                    sNoinitData.sEnergyOfLastDaysWattHourPerDay[1] = tEnergyPerLast24HourWattHour;
                    if (BlueDisplay1.isConnectionEstablished() && sShowRawDataDisplay == DO_NOT_SHOW_RAW_DATA) {
                        BD_printDayHistoricEnergyValues();
                    }
                }
            }
        }
        return true;
    }
    return false;
}
/*
 * Changes the power correction by 1% if button pressed.
 *
 * Is called every 40 ms seconds
 * For continuous press, autorepeat 2/s is entered after 2 seconds.
 */
void checkPowerCorrectionPins() {
    bool tMinusActivated = !digitalReadFast(POWER_CORRECTION_MINUS_PIN);
    bool tPlusActivated = !digitalReadFast(POWER_CORRECTION_PLUS_PIN);
    if (tMinusActivated || tPlusActivated) {
        if (!sPowerCorrectionButtonIsActive) {
            sPowerCorrectionButtonIsActive = true;
            /*
             * here one button just gets active -> change value
             */
            if (tMinusActivated) {
                sPowerCorrectionPercentage -= POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE;
            }
            if (tPlusActivated) {
                sPowerCorrectionPercentage += POWER_CORRECTION_PERCENTAGE_CHANGE_VALUE;
            }
            // Write value to EEPROM
            eeprom_write_byte(&sPowerCorrectionPercentageEeprom, sPowerCorrectionPercentage);
            if (BlueDisplay1.isConnectionEstablished()) {
                BD_printPowerCorrection();
            }
        }

    } else {
        /*
         * No button pressed here
         */
        sPowerCorrectionButtonIsActive = false;
    }
}

void checkShowBigDigitsPin() {
    bool tShowBigDigits = !digitalRead(ENABLE_SHOW_BIG_NUMBERS_PIN);
    if (sShowBigDigits != tShowBigDigits) {
        sShowBigDigits = tShowBigDigits;
//        if (sShowBigDigits) {
//            Serial.print(F("Do"));
//        } else {
//            Serial.print(F("Do not"));
//        }
//        Serial.print(F(" show big digits"));
        myLCD.clear(); // clear content of former page
//        LCD_checkAndPrintData(); // is done anyway in loop
    }
}

/*
 * 11 ms. 6 ms with delayMicroseconds(40); and 3.4 ms with delayMicroseconds(2) instead of delayMicroseconds(100);   // commands need > 37us to settle
 * Called every MILLISECONDS_BETWEEN_LCD_OUTPUT (320 ms)
 * @return  true if printing was done, false otherwise
 */
bool LCD_checkAndPrintData() {
    /*
     * 1. Row: ENERGY_ACCUMULATOR_1_WATT_HOUR is 45000 so we have only 16 bit resolution after division
     * 2 ms
     */
    if (sCounterOfPowerSamplesForLCD >= POWER_SAMPLES_BETWEEN_LCD_OUTPUT) {
        /*
         * 1. Row: Average power last 8 samples, last 5 minutes, last hour and last 24 hours
         * Maximum is 8.191 kW for 8 samples and 7.281 kW for 9 samples per display period
         */
        sPowerAverageWatt = sPowerAccumulatorWatt / sCounterOfPowerSamplesForLCD;
        snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings,
                sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("%4uW %4u %4u %4u"), sPowerAverageWatt,
                sPowerAverageOfLast5MinutesWatt, sPowerAverageOfLastHourWatt, sPowerAverageOfLast24HoursWatt);

        if (sShowBigDigits) {
            /*
             * Print the 4 line big digits here
             */
            BigNumbersLCD.setBigNumberCursor(0, 0);
            sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings[4] = '\0'; // terminate string after first number
            BigNumbersLCD.print(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);
            myLCD.setCursor(16, 1);
            myLCD.print(F("W")); // is shorter than 'W'
        } else {

            myLCD.setCursor(0, 0);
            myLCD.print(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);

            /*
             * 2. Row: kWh of last 24 hour, of last day and of day before
             */
            TIMING_PIN_LOW();
            myLCD.setCursor(0, 1);
            float tEnergyOfLastDayWattHour = sPowerAverageOfLast24HoursWatt * 0.024;
            LCDPrintFloatValueRightAligned(&myLCD, tEnergyOfLastDayWattHour, 5, false);
            myLCD.print(F("kWh "));

            LCDPrintFloatValueRightAligned(&myLCD, sNoinitData.sEnergyOfLastDaysWattHourPerDay[1] / 1000.0, 5, false);
            myLCD.print(' ');

            LCDPrintFloatValueRightAligned(&myLCD, sNoinitData.sEnergyOfLastDaysWattHourPerDay[0] / 1000.0, 5, false);

            /*
             * 3. Row: Negative part of power, power correction percentage
             * 2 ms
             */
            TIMING_PIN_HIGH();
            myLCD.setCursor(0, 2);

            // Print negative power
            snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings,
                    sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("%5dW          %3u%%"),
                    -(sNegativePowerAccumulatorWatt / sCounterOfPowerSamplesForLCD), sPowerCorrectionPercentage);
            myLCD.print(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);

            /*
             * 4. Row: Total time "9999D23H12M" is 11 characters long and clock correction
             * 2.3 ms
             */
            TIMING_PIN_LOW();
            myLCD.setCursor(0, 3);
            snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings,
                    sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("%2u:%02u:%02u  "), hour(), tm.Minute,
                    tm.Second);
            myLCD.print(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);
            printClockCorrection(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);
            myLCD.print(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);
        }

        /*
         * Clear accumulator after printing
         * 1 us
         */
        TIMING_PIN_HIGH();
        sCounterOfPowerSamplesForLCD = 0;
        sPowerAccumulatorWatt = 0;
        sNegativePowerAccumulatorWatt = 0;
        return true;
    }
    return false;
}

/*
 * It seems, that the send interrupt does not disturb the timing :-)
 * Integral of x^2 between 0 and Pi/2 is Pi/4 = 0.7854 = 1/2 of integral of constant 1
 * @return Theoretical maximum sum is: 384 samples * 1023 * 1023 = 401,867,136 (1/2 Giga), expected maximum is 1/4 Giga.
 *         Maximum is 3.552 kW for uint32 with POWER_SCALE_DIVISOR_FOR_1_WATT = 45747
 */
uint32_t readCurrentAndComputeRawPower(bool aStoreInArray) {
//    digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH);
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
//    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | ADC_PRESCALE32);
    /*
     * Theoretical maximum return value is NUMBER_OF_SAMPLES_FOR_10_MILLIS * 1023 * 1023 = 384 * 1046529 = 401,867,136
     */
    uint32_t tPower = 0;
    bool tShiftOfRawData; // To increase resolution of displayed data
    if (aStoreInArray) {
        tShiftOfRawData = (sShowRawDataDisplay == ACQUIRE_RAW_DATA_WITH_SHIFT); // Increase resolution of displayed data
        sMaxCurrent = 0;
        sCurrentAccumulator = 0;
        sPowerForShowRawData = 0;
    }
    /*
     * Now read 384 samples. Each loop lasts 26 us.
     */
    for (unsigned int i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; i++) {
        loop_until_bit_is_set(ADCSRA, ADIF);
//        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW);

        ADCSRA |= _BV(ADIF); // clear bit to enable recognizing next conversion has finished with "loop_until_bit_is_set(ADCSRA, ADIF)".
        uint16_t tCurrentValue = ADCL | (ADCH << 8); // using WordUnionForADCUtils does not save space here
// tValue = (ADCH << 8) | ADCL; // this does NOT work!
        if (aStoreInArray) {
            /*
             * Compute maximum and average of current
             * Convert current values to 8 bit ones for display
             */
            sCurrentAccumulator += tCurrentValue; // 512 = 11A
            if (sMaxCurrent < tCurrentValue) {
                sMaxCurrent = tCurrentValue;
            }
            if (tShiftOfRawData) {
                tCurrentValue = tCurrentValue >> 2;
            }
            sNoinitData.Arrays.ArrayForDisplayOfRawCurrent[i] = 0xFF - tCurrentValue; // store 8 bit value at current counter index[i] = 0xFF - tCurrentValue; // store 8 bit value at current counter index
        }
        tPower += (uint32_t) tCurrentValue * sNoinitData.Buffer.VoltageArray[i];
// 3 us after loop_until_bit_is_set() until here, if aStoreInArray is false
//        digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, HIGH);
//#if defined(TRACE)
//        if (i % 64 == 0) {
//            Serial.print(F("i="));
//            Serial.print(i);
//            Serial.print(F(" "));
//            Serial.print(tValue);
//            Serial.print(F("*"));
//            Serial.print(VoltageArray[i]);
//            Serial.print(F("="));
//            Serial.print(tPower);
//            Serial.print(F("|0x"));
//            Serial.print(tPower, HEX);
//            Serial.println();
//        }
//#endif
    }
//    digitalWriteFast(TIMING_DEBUG_OUTPUT_PIN, LOW);
//    Serial.print(F("PEnd="));
//    Serial.print(tPower);
//    Serial.print(F(" 0x"));
//    Serial.print(tPower, HEX);
//    Serial.println();
    return tPower;
}

/*
 * Store voltage in array. VoltageArray[0] contains the first non zero value
 * 26 us conversion time / 38.46 kHz
 * Voltage uses 1.1 V range. So we have full range at 399.3 V with divider 3.620 MOhm to 10 kOhm. 620 kOhm is a standard value.
 * => voltage LSB is 390 mV. (Raw * 399.3) / 1023
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
            sNoinitData.Buffer.VoltageArray[tCounter++] = tValue;

            if (tDoSearchForStart) {
                /*
                 * Check for 3 consecutive NON zero values
                 */
                if (tValue != 0) {
                    if (tCounter >= 3) {
                        disableMillisInterrupt(); // disable it here to have it exact 30 ms disabled
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
    // let the ADC run further for adjacent current measurement
}

/*
 * This interrupt prints the timeout message
 */
ISR(WDT_vect) {
    wdt_reset(); // avoid CPU reset
    myLCD.setCursor(0, 2);
    myLCD.print(F("No voltage detected "));
}
