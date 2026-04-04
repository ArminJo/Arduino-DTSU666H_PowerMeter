/*
 *  PowerLoggerAndChart.hpp
 *
 *  Stores Power values in an array and show them on a Android mobile or tablet running the BlueFisplay app.
 *
 *  Values are stored as 8 bits with in 10 watt units so maximum is 2550 W,
 *  Each 5 minutes the average of power is stored. I.e. 1 hour has 12 values, 72-> 6 hours, 288->1 day, 1152->4 days, 1440->5 days
 *
 *  DISPLAY_WIDTH is defined as 3.3/2 of CHART_WIDTH and CHART_WIDTH is POWER_ARRAY_SIZE / 2.
 *
 *
 *  Copyright (C) 2026  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-BlueDisplay https://github.com/ArminJo/Arduino-BlueDisplay.
 *
 *  BlueDisplay is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _POWER_LOGGER_AND_CHART_HPP
#define _POWER_LOGGER_AND_CHART_HPP

//#define DO_NOT_NEED_BASIC_TOUCH_EVENTS    // Disables basic touch events down, move and up. Saves 620 bytes program memory and 36 bytes RAM
#define DO_NOT_NEED_LONG_TOUCH_DOWN_AND_SWIPE_EVENTS    // Disables LongTouchDown and SwipeEnd events.
#define DO_NOT_NEED_SPEAK_EVENTS            // Disables SpeakingDone event handling. Saves up to 54 bytes program memory and 18 bytes RAM.
#define ONLY_CONNECT_EVENT_REQUIRED         // Disables reorientation, redraw and SensorChange events
#include "Chart.hpp"

void getTimeEventCallbackForLogger(uint8_t aSubcommand, uint8_t aByteInfo, uint16_t aShortInfo, ByteShortLongFloatUnion aLongInfo);
#define SYS_TIME_IS_NOT_INCREMENTED_BY_MILLIS
#define TIME_EVENTCALLBACK_FUNCTION     getTimeEventCallbackForLogger // use this function, which will also set sNextStorageSeconds
#include "BDTimeHelper.hpp"

//#define LOCAL_DEBUG
//#define ENABLE_STACK_ANALYSIS
#if defined(ENABLE_STACK_ANALYSIS)
#include "AVRUtils.h" // include for initStackFreeMeasurement() and printRAMAndStackInfo()
#include "AVRTracing.h"
#endif

/*
 * DISPLAY_WIDTH is defined as 3.3/2 of CHART_WIDTH
 * Scale the screen such, that this fit horizontally.
 * Border - YLabels - Chart with POWER_ARRAY_SIZE / 2 - Border - Buttons for 6 big characters - Border
 * Values for POWER_ARRAY_SIZE = 576
 */
#define CHART_WIDTH         (POWER_ARRAY_SIZE / 2)          // 288 => 1 day for uncompressed data
#define DISPLAY_WIDTH       ((POWER_ARRAY_SIZE * 33) / 40)  // 475,2 DISPLAY_WIDTH is defined as 3.3/2 of CHART_WIDTH
#define BASE_TEXT_SIZE      (POWER_ARRAY_SIZE / 40)         // 576 / 40 = 14.4
#define BASE_TEXT_SIZE_2    (POWER_ARRAY_SIZE / 20)         // 576 / 20 = 28.8
#define BASE_TEXT_SIZE_HALF (POWER_ARRAY_SIZE / 80)         // 7.2
#define BASE_TEXT_SIZE_1_5  ((POWER_ARRAY_SIZE * 3) / 80)   // 21.6
#define SMALL_BUTTON_WIDTH  (BASE_TEXT_SIZE * 4)            // 57 POWER_ARRAY_SIZE / 10
#define BUTTON_WIDTH        ((SMALL_BUTTON_WIDTH * 2) + BASE_TEXT_SIZE_HALF)   // 128
#define CHART_START_X       (3 * BASE_TEXT_SIZE)            // 42 Origin of chart
#define CHART_START_Y       (BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE_1_5) // Origin of chart
#define CHART_AXES_SIZE     (BASE_TEXT_SIZE / 8)            // 1
#define BUTTONS_START_X     (CHART_WIDTH + (4 * BASE_TEXT_SIZE))
#define TIME_INFO_START_Y   (BlueDisplay1.getRequestedDisplayHeight() - ((7 * BASE_TEXT_SIZE) / 2)) // start of date time print
#define TIME_MARKER_START_Y (BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE)

#define DIRECT_VOLTAGE_CHART_INDEX  1   // The index (1 to 16) of direct voltage chart for clearing. Index 0 is used for chart
#define DIRECT_CURRENT_CHART_INDEX  2   // The index of direct current chart for clearing.

#define CHART_BACKGROUND_COLOR  COLOR16_WHITE
#define CHART_DATA_COLOR        COLOR16_RED
#define CHART_AXES_COLOR        COLOR16_BLUE
#define CHART_GRID_COLOR        COLOR16_GREEN
#define CHART_DATA_COLOR        COLOR16_RED
#define CHART_TEXT_COLOR        COLOR16_BLACK
#define DAY_BUTTONS_COLOR       COLOR16_GREEN

Chart PowerChart;
//BDButton TouchButton4days;
BDButton TouchButton2days;
BDButton TouchButton1day;
BDButton TouchButton12Hour;
BDButton TouchButtonBrightness;
BDButton TouchButtonShowRawData;
BDButton TouchButtonClockCorrectionPlus;
BDButton TouchButtonClockCorrectionMinus;

#define DO_NOT_SHOW_RAW_DATA            0x00
#define ACQUIRE_RAW_DATA_FLAG           0x01 // Acquire RAW current data until no overflow
#define ACQUIRE_RAW_DATA_NO_SHIFT       ACQUIRE_RAW_DATA_FLAG // Acquire RAW current data and later check for overflow
#define ACQUIRE_RAW_DATA_WITH_SHIFT     (0x02 | ACQUIRE_RAW_DATA_FLAG) // Acquire RAW current data with shift -> no overflow possible
#define RAW_DATA_IS_VALID_FOR_DISPLAY   0x04 // Data just acquired and no overflow
#define DO_NOT_SHOW_CHART_DATA          0x06 // No overwrite of raw data already displayed
uint8_t sShowRawDataDisplay;
uint16_t sMaxCurrent;
uint32_t sCurrentAccumulator;
uint16_t sPowerForShowRawData;

BDButton TouchButtonShowTimeToNextStorage; // overlay for time text, not drawn, only activated

void initDisplay(void);
void drawDisplay();

void printClockCorrection(char *aBuffer);

void initPowerChart();
void drawPowerChart();

bool sDoInitDisplay = false; // used to call initDisplay on main loop which does save stack space.
bool sDoRefreshOrChangeBrightness = false; // used to call changeBrightness on main loop which does save stack space.
unsigned long sMillisOfLastRefreshOrChangeBrightness;
#define TIMEOUT_FOR_BRIGHTNESS_MILLIS      4000 // Before 4 seconds the next touch is interpreted as brightness change request

#define BRIGHTNESS_LOW      2
#define BRIGHTNESS_MIDDLE   1
#define BRIGHTNESS_HIGH     0
#define START_BRIGHTNESS    BRIGHTNESS_HIGH
uint8_t sCurrentBrightness = START_BRIGHTNESS;
color16_t sBackgroundColor = CHART_BACKGROUND_COLOR;
color16_t sTextColor = CHART_TEXT_COLOR;

/*
 * Power storage in RAM
 */
#define NUMBER_OF_DAYS_IN_BUFFER    2
// At 5 minutes / sample: 576 -> 2 days, 864 -> 3 days, 1152-> 4 days, 1440->5 days
#define POWER_ARRAY_SIZE  (NUMBER_OF_DAYS_IN_BUFFER * HOURS_IN_ONE_DAY * 12UL) // 576 - UL is essential!
#define POWER_COMPRESSION_FACTOR    10L // 10 W = 1 100, W = 10, 2550 W = 255

uint16_t sChartHoursToDisplay;
/*
 * We must use a structure here, otherwise we can not determine the sequence of variables in the .noinit section
 */
struct noinitStruct {
    uint8_t EntriesInWattPerHour;
    uint16_t EnergyPerHourWattHour[HOURS_IN_ONE_DAY]; // To compute rolling energy per day value
    uint16_t sEnergyOfLastDaysWattHourPerDay[2]; // last day (1) and day before (0) from 3:00 to 3:00
    uint16_t PowerArrayValuesChecksum; // must be in noinit, otherwise it is set to 0 at boot
    /*
     * Used for display of power  at BlueDisplay or current (at Aduino Plotter)
     * Place this array at end of BSS to be first overwritten by stack.
     */
    union {
        uint8_t PowerArray[POWER_ARRAY_SIZE]; // 576 (2 days) values for display. Contains power in 10 Watt units (POWER_COMPRESSION_FACTOR)
        uint8_t ArrayForDisplayOfRawCurrent[NUMBER_OF_SAMPLES_FOR_10_MILLIS]; // First 384 entries of 2 day display are overwritten by display of raw data
    } Arrays;

    /*
     * Place this array at end of BSS to be first overwritten by stack.
     * It is restored every time loop starts
     */
    union {
        char sStringBufferForLCDRowAndBDStrings[LCD_COLUMNS + 1]; // For rendering LCD lines with snprintf_P() and for "Next in %d min %02d sec", which requires exact 21 character
        uint16_t VoltageArray[NUMBER_OF_SAMPLES_FOR_10_MILLIS];
    } Buffer;
} sNoinitData __attribute__((section(".noinit")));
uint8_t *sPowerArrayDisplayStart = sNoinitData.Arrays.PowerArray; // Start of displayed values determined by doDays()

void initializePowerArray();
void writeToPowerArray(uint8_t aPowerValue);

void signalInitDisplay(void);
void doDays(BDButton *aTheTouchedButton, int16_t aChartHoursToDisplay);
void doClockCorrection(BDButton *aTheTouchedButton, int16_t aCorrectionValue);
void doSignalChangeBrightness(BDButton *aTheTouchedButton, int16_t aValue);
void doShowTimeToNextStorage(BDButton *aTheTouchedButton, int16_t aValue);
void doShowRawData(BDButton *aTheTouchedButton, int16_t aValue);

bool handleEventAndFlags();
void checkAndHandlePowerLoggerEventFlags();

bool BD_checkAndPrintChangedPowerValue();
void BD_printDaylyEnergyValue();
void BD_printDayHistoricEnergyValues();
bool BD_printChangedPowerValue();
void changeBrightness();

/*
 * Date and time handling for X scale
 */
#define STORAGE_INTERVAL_SECONDS (5 * SECONDS_IN_ONE_MINUTE)
// sNextStorageSeconds is required by PowerLoggerAndChart.hpp for toast at connection startup
uint32_t sNextStorageSeconds = SECONDS_IN_ONE_MINUTE; // For seconds until next storage. If not connected, first storage in 1 minute after boot.
void printTimeToNextStorage();

// Get absolute value of difference of two unsigned values
#if !defined(uintDifferenceAbs)
#define uintDifferenceAbs(a, b) ((a >= b) ? a - b : b - a)
#endif

/*
 * Code starts here
 */

/*
 * Is intended to be called by setup()
 */
void InitPowerLoggerAndChart() {
#if defined(ENABLE_STACK_ANALYSIS)
    initStackFreeMeasurement();
#endif

    /*
     * Register callback handler and wait for 1500 ms (CONNECTIOM_TIMEOUT_MILLIS) if Bluetooth connection is still active.
     * For ESP32 and after power on of the Bluetooth module (HC-05) at other platforms, Bluetooth connection is most likely not active here.
     *
     * If active, mCurrentDisplaySize and mHostUnixTimestamp are set and callback handler (initDisplay() and drawGui()) functions are called.
     * If not active, the periodic call of checkAndHandleEvents() in the main loop waits for the (re)connection and then performs the same actions.
     *
     * Here the initDisplay() is delayed and called by the loop().
     */
    BlueDisplay1.initCommunication(&Serial, &signalInitDisplay); // introduces up to 1.5 seconds delay, no
    initializePowerArray();

// Simulate a connection for testing
//    BlueDisplay1.mHostDisplaySize.XWidth = 1280;
//    BlueDisplay1.mHostDisplaySize.YHeight = 800;
//    BlueDisplay1.mHostUnixTimestamp = 1733191057;
//    BlueDisplay1.mBlueDisplayConnectionEstablished = true;
//    sDoInitDisplay = true;

#if defined(STANDALONE_TEST)
    uint8_t *tArrayFillPointer = sNoinitData.Arrays.PowerArray;
    for (int i = 0; i < 200; ++i) {
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
    }
    for (int i = 200; i > 120; --i) {
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
        *tArrayFillPointer++ = i;
    }
    while (tArrayFillPointer < &sNoinitData.Arrays.PowerArray[POWER_ARRAY_SIZE]) {
        *tArrayFillPointer++ = 80;
    }
#endif
}

/*
 * Array is in section .noinit
 * Therefore we can check the checksum and keep existing data before initializing it after reboot
 */
void initializePowerArray() {
    uint16_t tPowerArrayChecksum = 0;

    for (uint16_t i = 0; i < POWER_ARRAY_SIZE; ++i) {
        tPowerArrayChecksum += sNoinitData.Arrays.PowerArray[i];
    }
    if (tPowerArrayChecksum == sNoinitData.PowerArrayValuesChecksum) {
#if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.println(F("Checksum match -> keep array"));
#else
        BlueDisplay1.debug(PSTR("Checksum match -> keep array"));
#endif
    } else {

        // checksum does not match -> initialize array
#if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.print(F("Computed checksum "));
        Serial.print(tPowerArrayChecksum);
        Serial.print(F(" does not match "));
        Serial.print(sNoinitData.PowerArrayValuesChecksum);
        Serial.println(F(", assume power up -> initialize array"));
#else
        BlueDisplay1.debug(PSTR("Checksum mismatch"));
#endif

        // Fill the noinit data array with zero
        memset(&sNoinitData, 0x00, sizeof(sNoinitData));

#if defined(ENABLE_STACK_ANALYSIS)
        printRAMAndStackInfo(&Serial);
#  if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.flush();
#  endif
#endif
    }
}

/*
 * aPowerValue is AverageWatt / 10 (POWER_COMPRESSION_FACTOR)
 */
void writeToPowerArray(uint16_t aPowerValue) {
    uint8_t tPowerValue = aPowerValue / POWER_COMPRESSION_FACTOR;

    sNoinitData.PowerArrayValuesChecksum -= sNoinitData.Arrays.PowerArray[0]; // adjust checksum with removed element
    // Shift array to front and get maximum
    for (uint16_t i = 0; i < POWER_ARRAY_SIZE - 1; ++i) {
        sNoinitData.Arrays.PowerArray[i] = sNoinitData.Arrays.PowerArray[i + 1];
    }

    sNoinitData.Arrays.PowerArray[POWER_ARRAY_SIZE - 1] = tPowerValue;
    sNoinitData.PowerArrayValuesChecksum += tPowerValue; // adjust checksum with new element

    if (BlueDisplay1.isConnectionEstablished()) {
        drawPowerChart();
    }

#if defined(LOCAL_DEBUG)
    Serial.print(F("Write "));
    Serial.print(aPowerValue);
#  if defined(LOCAL_TRACE)
    Serial.print(F(" Checksum="));
    Serial.print(sNoinitData.PowerArrayValuesChecksum);
#  endif
    Serial.println();
#endif
}

/**************************************
 * BlueDisplay GUI related functions
 **************************************/
/*
 * This function is not called by event callback, it is called from setup or main loop
 * signalInitDisplay() is called by event callback, which only sets a flag for the main loop.
 * This helps reducing stack usage,
 * !!!but BlueDisplay1.isConnectionEstablished() cannot be used as indicator for initialized BD data,
 * without further handling, because initialized BD data may be delayed!!!
 */
void initDisplay(void) {
#if defined(LOCAL_DEBUG)
    Serial.println(F("InitDisplay"));
#endif

    uint16_t tDisplayHeight = BlueDisplay1.getHostDisplayHeight();
    uint16_t tDisplayWidth = BlueDisplay1.getHostDisplayWidth();
    if (tDisplayHeight > tDisplayWidth) {
        // her we have portrait -> swap to landscape
        tDisplayHeight = tDisplayWidth;
        tDisplayWidth = BlueDisplay1.getHostDisplayHeight();
    }
    tDisplayHeight = (DISPLAY_WIDTH * tDisplayHeight) / tDisplayWidth;
    if (tDisplayHeight < ((float) DISPLAY_WIDTH / 1.75)) {
        // Clip ratio to 4 : 7 otherwise the layout is to broad
        tDisplayHeight = (float) DISPLAY_WIDTH / 1.75;
    }
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH, tDisplayHeight);

// set layout variables;
    int tDisplayHeightEighth = tDisplayHeight / 8;

// here we have received a new local timestamp
    initLocalTimeHandling();

    BDButton::BDButtonPGMTextParameterStruct tBDButtonPGMParameterStruct; // Saves 480 Bytes for all 5 buttons

    BDButton::setInitParameters(&tBDButtonPGMParameterStruct, BUTTONS_START_X, BASE_TEXT_SIZE, SMALL_BUTTON_WIDTH,
            tDisplayHeightEighth, DAY_BUTTONS_COLOR, F("4"), BASE_TEXT_SIZE_2, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 4 * HOURS_IN_ONE_DAY,
            &doDays);
//    TouchButton4days.init(&tBDButtonPGMParameterStruct);

    /*
     * Draw 2 buttons right
     */
    tBDButtonPGMParameterStruct.aPositionX += (BASE_TEXT_SIZE * 4) + BASE_TEXT_SIZE_HALF; // gap of (BASE_TEXT_SIZE / 2)
    tBDButtonPGMParameterStruct.aValue = 2 * HOURS_IN_ONE_DAY; // 48
#if defined(__AVR__)
    tBDButtonPGMParameterStruct.aPGMText = F("2");
#else
    tBDButtonParameterStruct.aText = "2";
#endif
    TouchButton2days.init(&tBDButtonPGMParameterStruct);

    tBDButtonPGMParameterStruct.aPositionY += (tDisplayHeightEighth + BASE_TEXT_SIZE_2 + BASE_TEXT_SIZE_HALF);
    tBDButtonPGMParameterStruct.aValue = HOURS_IN_ONE_DAY / 2; // 12
    tBDButtonPGMParameterStruct.aPGMText = F("1/2");
    TouchButton12Hour.init(&tBDButtonPGMParameterStruct); // 14 bytes

    tBDButtonPGMParameterStruct.aPositionX = BUTTONS_START_X;
    tBDButtonPGMParameterStruct.aValue = HOURS_IN_ONE_DAY; // 24
    tBDButtonPGMParameterStruct.aPGMText = F("1");
    TouchButton1day.init(&tBDButtonPGMParameterStruct); // 14 bytes

    int tButtonYSpacing = tDisplayHeightEighth + BASE_TEXT_SIZE_HALF;

    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH;
    tBDButtonPGMParameterStruct.aTextSize = BASE_TEXT_SIZE;

    tBDButtonPGMParameterStruct.aButtonColor = COLOR16_YELLOW;
    tBDButtonPGMParameterStruct.aPositionY += tButtonYSpacing;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doShowRawData;
    tBDButtonPGMParameterStruct.aPGMText = F("Show RAW\ndata");
    TouchButtonShowRawData.init(&tBDButtonPGMParameterStruct);

    tBDButtonPGMParameterStruct.aButtonColor = COLOR16_LIGHT_GREY;
    tBDButtonPGMParameterStruct.aPositionY += tButtonYSpacing;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doSignalChangeBrightness;
    tBDButtonPGMParameterStruct.aPGMText = F("Refresh/\nBrightness");
    TouchButtonBrightness.init(&tBDButtonPGMParameterStruct);
    TouchButtonBrightness.setButtonTextColor(COLOR16_WHITE);

    /*
     * Button as invisible overlay to the time text.
     * Shows next time to storage as toast
     */
    tBDButtonPGMParameterStruct.aPositionY = TIME_INFO_START_Y;
    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH - (2 * BASE_TEXT_SIZE);
    tBDButtonPGMParameterStruct.aHeightY = 2 * BASE_TEXT_SIZE;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doShowTimeToNextStorage;
    tBDButtonPGMParameterStruct.aPGMText = F("Next"); // not shown, just for debugging
    TouchButtonShowTimeToNextStorage.init(&tBDButtonPGMParameterStruct);

    // Timing correction buttons
    tBDButtonPGMParameterStruct.aPositionX = BUTTONS_START_X + BUTTON_WIDTH - BASE_TEXT_SIZE;
    tBDButtonPGMParameterStruct.aPositionY = TIME_INFO_START_Y - 1;
    tBDButtonPGMParameterStruct.aValue = 1;
    tBDButtonPGMParameterStruct.aWidthX = BASE_TEXT_SIZE;
    tBDButtonPGMParameterStruct.aHeightY = BASE_TEXT_SIZE + 1;
    tBDButtonPGMParameterStruct.aTextSize = BASE_TEXT_SIZE / 2;
    tBDButtonPGMParameterStruct.aButtonColor = CHART_BACKGROUND_COLOR;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doClockCorrection;
    tBDButtonPGMParameterStruct.aPGMText = F("+");
    TouchButtonClockCorrectionPlus.init(&tBDButtonPGMParameterStruct);

    tBDButtonPGMParameterStruct.aPositionY = TIME_INFO_START_Y + BASE_TEXT_SIZE + 2;
    tBDButtonPGMParameterStruct.aValue = -1;
    tBDButtonPGMParameterStruct.aPGMText = F("-");
    TouchButtonClockCorrectionMinus.init(&tBDButtonPGMParameterStruct);

    sCurrentBrightness = BRIGHTNESS_LOW;
    changeBrightness(); // from low to high :-)
}

void BD_printPowerCorrection() {
    snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings),
            PSTR("%4u%%"), sPowerCorrectionPercentage);
    BlueDisplay1.drawText(DISPLAY_WIDTH - (2 * BASE_TEXT_SIZE_2), BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE,
            sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, BASE_TEXT_SIZE, CHART_TEXT_COLOR, sBackgroundColor);
}

void printClockCorrection(char *aBuffer) {
    int16_t tSecondsPerDay = (((long) SECONDS_IN_ONE_DAY) * sOneSecondTimer64usCorrectionCount) / 15625;
    char tSignOfCorrection;
    if (tSecondsPerDay >= 0) {
        tSignOfCorrection = '+';
    } else {
        tSignOfCorrection = '-';
        tSecondsPerDay = -tSecondsPerDay;
    }
    sprintf_P(aBuffer, PSTR("%c%2u:%02u/day"), tSignOfCorrection, tSecondsPerDay / SECONDS_IN_ONE_MINUTE,
            tSecondsPerDay % SECONDS_IN_ONE_MINUTE);
}

void BD_printClockCorrection() {
    printClockCorrection(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings);
    BlueDisplay1.drawText(BUTTONS_START_X, BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE,
            sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, (BASE_TEXT_SIZE * 3) / 4, CHART_TEXT_COLOR, sBackgroundColor);
}

void drawDisplay() {
    BlueDisplay1.clearDisplay(sBackgroundColor);
    PowerChart.drawYAxisTitle(-(int) BASE_TEXT_SIZE_2, CHART_START_X);
    sLastDisplayedPowerAverageWatt = __UINT16_MAX__; // force display of Power

    drawPowerChart(); // also prints date with printTimeAtTwoLines

//    TouchButton4days.drawButton();
    TouchButton2days.drawButton();

    // Draw text between buttons
    BlueDisplay1.drawText(BUTTONS_START_X + BASE_TEXT_SIZE_2, (BlueDisplay1.getRequestedDisplayHeight() / 4) - BASE_TEXT_SIZE,
            F("Day(s)"), BASE_TEXT_SIZE_1_5, DAY_BUTTONS_COLOR, sBackgroundColor);
    TouchButton1day.drawButton();
    TouchButton12Hour.drawButton();

    TouchButtonShowRawData.drawButton();
    TouchButtonBrightness.drawButton();
    TouchButtonShowTimeToNextStorage.activate(); // Just listen for touches on the date / time area

    BD_printPowerCorrection();
    TouchButtonClockCorrectionPlus.drawButton();
    TouchButtonClockCorrectionMinus.drawButton();
    BD_printClockCorrection();
}

void setChartHoursToDisplay(int16_t aChartHoursToDisplay) {
    sChartHoursToDisplay = aChartHoursToDisplay;

    // defaults for every scale except 1/2 day
    uint8_t tXPixelSpacing = CHART_WIDTH / 8;  // 8 grid lines per chart
    uint8_t tXLabelDistance = 2; // draw label at every 2. grid line

    int8_t tXLabelScaleFactor;
    int8_t tDataFactor;
    if (aChartHoursToDisplay == (2 * HOURS_IN_ONE_DAY)) {
        // Data compressed by 2
        sPowerArrayDisplayStart = &sNoinitData.Arrays.PowerArray[0];
        tDataFactor = CHART_X_AXIS_SCALE_FACTOR_COMPRESSION_2;
        /*
         * 48 hours | 2 days -> with 8 grids at X axis => 1 grid line each 6 hours, label each 12 hours => X scale expanded by 2
         */
        tXLabelScaleFactor = CHART_X_AXIS_SCALE_FACTOR_EXPANSION_2;
    } else if (aChartHoursToDisplay == HOURS_IN_ONE_DAY) {
        // Data direct
        sPowerArrayDisplayStart = &sNoinitData.Arrays.PowerArray[POWER_ARRAY_SIZE / 2];
        tDataFactor = CHART_X_AXIS_SCALE_FACTOR_1;
        /*
         * 24 hours |  day -> with 8 grids at X axis => 1 grid line each 3 hours, label each 6 hours => X scale expanded by 4
         */
        tXLabelScaleFactor = CHART_X_AXIS_SCALE_FACTOR_EXPANSION_4;
    } else {
        // Data expanded by 2
        sPowerArrayDisplayStart = &sNoinitData.Arrays.PowerArray[(POWER_ARRAY_SIZE / 2) + (POWER_ARRAY_SIZE / 4)];
        tDataFactor = CHART_X_AXIS_SCALE_FACTOR_EXPANSION_2;
        /*
         * 12 hour. Here the regular expansion would be 8, which leads to 1.5 hour per grid
         * So change the grid to 1 hour per grid and draw labels every 3 hours
         */
        tXPixelSpacing = CHART_WIDTH / 12; // 12 grid lines per chart
        tXLabelDistance = 3; // draw label at every 3. grid line, but draw big label still each 2x12 hours by keeping mXBigLabelDistance at 2
        tXLabelScaleFactor = CHART_X_AXIS_SCALE_FACTOR_EXPANSION_12; // 1 hour per grid
    }
    PowerChart.setXLabelDistance(tXLabelDistance);
    PowerChart.setXGridOrLabelPixelSpacing(tXPixelSpacing);
    PowerChart.setXLabelScaleFactor(tXLabelScaleFactor);
    PowerChart.setXDataScaleFactor(tDataFactor);
}

void doDays(BDButton *aTheTouchedButton, int16_t aChartHoursToDisplay) {
    (void) aTheTouchedButton;
    setChartHoursToDisplay(aChartHoursToDisplay);
    drawPowerChart(); // this sets XLabelAndGridOffset
}

void doClockCorrection(BDButton *aTheTouchedButton, int16_t aCorrectionValue) {
    (void) aTheTouchedButton;
    sOneSecondTimer64usCorrectionCount = sOneSecondTimer64usCorrectionCount + aCorrectionValue;
    eeprom_update_byte((uint8_t*) &sOneSecondTimer64usCorrectionCountEeprom, sOneSecondTimer64usCorrectionCount);
    set1SecondTimer1Correction();
    BD_printClockCorrection();
}

/*
 * Current time is at pixel position CHART_START_X + CHART_WIDTH
 * Touched time is at current time - (pixel_difference * X_data_scale * 5 min)
 */
void doShowTimeAtTouchPosition(struct TouchEvent *const aTouchPosition) {
    static struct XYPosition sLastPosition = { 0, 0 };
    uint16_t tPositionX = aTouchPosition->TouchPosition.PositionX;
    if (tPositionX >= CHART_START_X && tPositionX <= (CHART_START_X + CHART_WIDTH)) {
        uint16_t tPixelDifference = PowerChart.reduceLongWithIntegerScaleFactor((CHART_START_X + CHART_WIDTH) - tPositionX,
                PowerChart.getXDataScaleFactor());
        time_float_union tTimeOfTouchPosition;

#if defined(USE_C_TIME)
        tTimeOfTouchPosition.TimeValue = (BlueDisplay1.getHostUnixTimestamp()
                - (BlueDisplay1.getHostUnixTimestamp() % STORAGE_INTERVAL_SECONDS)) - (tPixelDifference * STORAGE_INTERVAL_SECONDS);
#else
        tTimeOfTouchPosition.TimeValue = (now() - (now() % STORAGE_INTERVAL_SECONDS))
                - (tPixelDifference * STORAGE_INTERVAL_SECONDS);
#endif

        char tTimeString[10];
        convertUnixTimestampToHourAndMinuteString(tTimeString, tTimeOfTouchPosition);
        // append "    " to clear "/day"
        tTimeString[5] = ' ';
        tTimeString[6] = ' ';
        tTimeString[7] = ' ';
        tTimeString[8] = ' ';
        tTimeString[9] = '\0';
        BlueDisplay1.drawText(BUTTONS_START_X, TIME_MARKER_START_Y, tTimeString, BASE_TEXT_SIZE, CHART_DATA_COLOR,
                sBackgroundColor);
        // Clear last indicator
        if (sLastPosition.PositionX != 0) {
            BlueDisplay1.drawLineRel(sLastPosition.PositionX, sLastPosition.PositionY - 48, 0, 32, sBackgroundColor);
        }
        // Draw an indicator
        if (aTouchPosition->TouchPosition.PositionY > 48) {
            BlueDisplay1.drawLineRel(tPositionX, aTouchPosition->TouchPosition.PositionY - 48, 0, 32, CHART_DATA_COLOR);
            sLastPosition = aTouchPosition->TouchPosition;
        }
    }
}

/*
 * Base is:
 * X label increment is 1/2 day at X label scale factor 1
 * big labels are every day / 2 labels
 * Skip every 2. label text
 * With 2 days (576 values) in buffer we have no data compression at 1 day, because the chart width is half the buffer size.
 * For 1 day have one big (day) label per chart => with 8 X labels per chart we need a label expansion of 4
 * For 2 day have two big (day) label per chart => with 8 X labels per chart we need a label expansion of 2 and data compression of 2
 *
 * 8 (12 for 1/2 day) grid lines per chart is set by doDays()
 */
void initPowerChart() {
    sPowerArrayDisplayStart = sNoinitData.Arrays.PowerArray;
    PowerChart.initXLabelTimestampForLabelScaleIdentity(0, SECONDS_IN_ONE_DAY / 2, 5); // Increment is 1/2 day at X label scale factor 1
    // PowerChart.setXLabelDistance(2); // Draw label at every 2. grid line except for 1/2 day => Actual label distance is set by doDays()
    PowerChart.setXBigLabelDistance(2); // Show day date label bigger
    PowerChart.setLabelStringFunction(convertUnixTimestampToHourString);

    uint16_t tChartHeight = BlueDisplay1.getRequestedDisplayHeight() - (BlueDisplay1.getRequestedDisplayHeight() / 4); // 3/4 display height
    PowerChart.initChart(CHART_START_X, CHART_START_Y, CHART_WIDTH, tChartHeight, CHART_AXES_SIZE, BASE_TEXT_SIZE,
    CHART_DISPLAY_GRID, 0, BlueDisplay1.getRequestedDisplayHeight() / 14); // GridOrLabelXPixelSpacing is set by doDays()

    PowerChart.initChartColors(CHART_DATA_COLOR, CHART_AXES_COLOR, CHART_GRID_COLOR, sTextColor, sTextColor, sBackgroundColor);
    PowerChart.setYTitleTextAndSize("Watt", BASE_TEXT_SIZE_1_5);

    /*
     * Set values initially for whole buffer
     */
    setChartHoursToDisplay(NUMBER_OF_DAYS_IN_BUFFER * HOURS_IN_ONE_DAY);

    registerTouchDownCallback(doShowTimeAtTouchPosition);
    registerTouchMoveCallback(doShowTimeAtTouchPosition);
}

void drawPowerChart() {
    if (sShowRawDataDisplay == DO_NOT_SHOW_RAW_DATA) {
        BD_printChangedPowerValue();
        BD_printDaylyEnergyValue();
        BD_printDayHistoricEnergyValues();

        PowerChart.clear();

        /*
         * Offset to origin for first big (date / midnight) label in seconds
         */
#if defined(USE_C_TIME)
        long tDifferenceToLastMidnightInSeconds = sTimeInfo->tm_sec + ((sTimeInfo->tm_min + (MINUTES_IN_ONE_HOUR * sTimeInfo->tm_hour)) * (long)SECONDS_IN_ONE_MINUTE);
#else
        long tDifferenceToLastMidnightInSeconds = second()
                + ((minute() + (MINUTES_IN_ONE_HOUR * hour())) * (long) SECONDS_IN_ONE_MINUTE);
#endif
        /*
         * We must always start with a midnight value to mark the start of the big labels
         * (+ SECONDS_PER_STORAGE) if we have midnight, the offset of the first label must be -1,
         * such that the midnight label is drawn at rightmost X axis position.
         * Because for data compression 2 one chart step is 2 SECONDS_PER_STORAGE,
         * we must expand it to 2 * SECONDS_PER_STORAGE for the effect.
         */
        long tCorrectedSecondsForStorage = PowerChart.reduceLongWithIntegerScaleFactor(SECONDS_PER_STORAGE,
                PowerChart.mXDataScaleFactor);
        if (sChartHoursToDisplay == (HOURS_IN_ONE_DAY / 2)) {
            /*
             * We want to see the labels of the last half day here, so midnight grid line is shifted left by 12 hours
             */
            PowerChart.setXLabelAndGridOffset(
                    tDifferenceToLastMidnightInSeconds + (sChartHoursToDisplay * SECONDS_IN_ONE_HOUR)
                            + tCorrectedSecondsForStorage);
        } else {
            PowerChart.setXLabelAndGridOffset(tDifferenceToLastMidnightInSeconds + tCorrectedSecondsForStorage);
        }

        // Use 1,2, or 4 days back as start value
        // Timestamp of the midnight, which is left by tDifferenceToLastMidnightInSeconds of Y axis.
        uint8_t tFullDays =
                ((sChartHoursToDisplay - 1) / HOURS_IN_ONE_DAY) + 1; // Results in 1 for values <= 24
#if defined(USE_C_TIME)
    PowerChart.drawXAxisAndDateLabels(
            BlueDisplay1.getHostUnixTimestamp() - ((tFullDays * SECONDS_IN_ONE_DAY) + tDifferenceToLastMidnightInSeconds),
            convertUnixTimestampToHourString);
#else
        PowerChart.drawXAxisAndDateLabels(now() - ((tFullDays * SECONDS_IN_ONE_DAY) + tDifferenceToLastMidnightInSeconds),
                convertUnixTimestampToDateString);
#endif

#if defined(LOCAL_TRACE)
        Serial.println();
        Serial.print(F(" Offset="));
        Serial.print(tDifferenceToLastMidnightInSeconds);
        Serial.print(F(" FullDays="));
        Serial.println(tFullDays);
#endif

        /*
         * Do auto scale
         */
        // Get maximum of displayed range
        uint8_t tPowerArrayMaximumValue10Watt = 0;
        auto tPowerArrayDisplayStart = sPowerArrayDisplayStart;
        for (; tPowerArrayDisplayStart < &sNoinitData.Arrays.PowerArray[POWER_ARRAY_SIZE]; ++tPowerArrayDisplayStart) {
            if (tPowerArrayMaximumValue10Watt < *tPowerArrayDisplayStart) {
                tPowerArrayMaximumValue10Watt = *tPowerArrayDisplayStart;
            }
        }
        // Scale
        uint16_t tYLabelIncrementValue;
        if (tPowerArrayMaximumValue10Watt <= 10) {
            // 0 to 100 W
            tYLabelIncrementValue = 10; // 10 W per grid
        } else if (tPowerArrayMaximumValue10Watt <= 20) {
            // 0 to 200 W
            tYLabelIncrementValue = 20; // 20 W per grid
        } else if (tPowerArrayMaximumValue10Watt <= 50) {
            tYLabelIncrementValue = 50;
        } else if (tPowerArrayMaximumValue10Watt <= 100) {
            tYLabelIncrementValue = 100;
        } else if (tPowerArrayMaximumValue10Watt <= 200) {
            // 0 to 2 kW
            tYLabelIncrementValue = 200;
        } else {
            // over 2 kW
            tYLabelIncrementValue = 500; // 500 W per grid
        }
        PowerChart.initYLabel(0, tYLabelIncrementValue, POWER_COMPRESSION_FACTOR, 4, 0); // 10 for input 1 , 4 is minimum label string width

        PowerChart.drawYAxisAndLabels(); // this will restore the overwritten ??? label
        PowerChart.drawGrid();
        PowerChart.drawChartDataWithYOffset(sPowerArrayDisplayStart, POWER_ARRAY_SIZE, CHART_MODE_LINE);

        /*
         * Print time and clear time marker area see doShowTimeAtTouchPosition()
         * Print it after chart, because the sign of day correction value is cleared by chart drawing
         */
        printTimeAtTwoLines(BUTTONS_START_X, TIME_INFO_START_Y, BASE_TEXT_SIZE, sTextColor, sBackgroundColor); // gets now()
        BD_printClockCorrection(); // this clears the time marker area
    }
}

void drawRawDataChart() {
    BlueDisplay1.clearDisplay(CHART_BACKGROUND_COLOR);
    TouchButtonShowRawData.drawButton();
    TouchButtonBrightness.drawButton();

    uint16_t tMaxVoltage = 0;
    uint32_t tVoltageAccumulator = 0;

    /*
     * Compute maximum and average of voltage
     */
    for (uint16_t i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; ++i) {
        tVoltageAccumulator += sNoinitData.Buffer.VoltageArray[i];
        if (tMaxVoltage < sNoinitData.Buffer.VoltageArray[i]) {
            tMaxVoltage = sNoinitData.Buffer.VoltageArray[i];
        }
    }
    /*
     * Assume VOLTAGE full 1.1 V range at 400 V with divider 3.6263 MOhm to 10 kOhm.
     * 620 kOhm is a standard value and with 3.62 MOhm we have 0.17% deviation.
     * => voltage LSB is 391 mV. (Raw * 400) / 1023
     */
    tMaxVoltage = (tMaxVoltage * 400L) / 1023;
    uint16_t tAverageVoltage = (tVoltageAccumulator * 400L) / (1023L * NUMBER_OF_SAMPLES_FOR_10_MILLIS);

    /*
     * Convert Raw voltage for display to 8 bit values
     * This overwrites first half of 16 bit voltage buffer
     */
    uint8_t *t8BitVoltagePointer = (uint8_t*) sNoinitData.Buffer.VoltageArray;
    for (uint16_t i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; ++i) {
        *t8BitVoltagePointer = 0xFF - (sNoinitData.Buffer.VoltageArray[i] >> 2);
        t8BitVoltagePointer++;
    }
#if defined(SEARCH_LOWEST_STACKPOINTER_MODE)
        startTracing();
#endif
    BlueDisplay1.drawChartByteBuffer(0, 0, COLOR16_RED, COLOR16_WHITE, DIRECT_VOLTAGE_CHART_INDEX, false,
            (uint8_t*) sNoinitData.Buffer.VoltageArray, NUMBER_OF_SAMPLES_FOR_10_MILLIS);
#if defined(SEARCH_LOWEST_STACKPOINTER_MODE)
        stopTracing();
        Serial.println();
        printLowestStackpointerAndProgramAddress();
#endif
    BlueDisplay1.drawChartByteBuffer(0, 0, COLOR16_BLUE, COLOR16_WHITE, DIRECT_CURRENT_CHART_INDEX, true,
            (uint8_t*) sNoinitData.Arrays.ArrayForDisplayOfRawCurrent, NUMBER_OF_SAMPLES_FOR_10_MILLIS);
    // Reset values for later display of power
    for (uint16_t i = 0; i < NUMBER_OF_SAMPLES_FOR_10_MILLIS; ++i) {
        sNoinitData.Arrays.PowerArray[i] = 0;
    }

    /*
     * With ZMCT103C and 50 Ohm we get 0.5V at 10 A and full 1.1 V range at 22 A. (22A is peak value for 15.5 A effective value)
     * => current LSB is 21.5 mA  (Raw * 22000) / 1023 => Power LSB is 8.4 mW.
     * Maximum power value for 15 A and 230 V is: 3.45 kW
     * Use VoltageArray as string buffer
     */
    uint32_t tMaxCurrentMilliampere = (float) sMaxCurrent * (22000.0 / 1023.0);
    uint32_t tAverageCurrentMilliampere = (float) sCurrentAccumulator * (22000.0 / (1023.0 * NUMBER_OF_SAMPLES_FOR_10_MILLIS));

    snprintf_P((char*) sNoinitData.Buffer.VoltageArray, NUMBER_OF_SAMPLES_FOR_10_MILLIS,
            PSTR("%4uW Max:%3uV %2u.%03uA|0x%04X  Avg:%3uV %2u.%03uA|0x%04X"), sPowerForShowRawData, tMaxVoltage,
            (uint16_t) tMaxCurrentMilliampere / 1000, (uint16_t) tMaxCurrentMilliampere % 1000, sMaxCurrent, tAverageVoltage,
            (uint16_t) tAverageCurrentMilliampere / 1000, (uint16_t) tAverageCurrentMilliampere % 1000,
            sCurrentAccumulator / NUMBER_OF_SAMPLES_FOR_10_MILLIS);
    BlueDisplay1.drawText(BASE_TEXT_SIZE, BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE,
            (char*) sNoinitData.Buffer.VoltageArray, BASE_TEXT_SIZE, CHART_TEXT_COLOR, sBackgroundColor);
}

void checkAndHandlePowerLoggerEventFlags() {
    if (sDoInitDisplay) {
        sDoInitDisplay = false; // do it once
        setTime(BlueDisplay1.mHostUnixTimestamp); // set it before drawDisplay() -> drawPowerChart() -> printTimeAtTwoLines()
        initDisplay();
        initPowerChart();
        drawDisplay();
    }
    if (sDoRefreshOrChangeBrightness) {
        sDoRefreshOrChangeBrightness = false; // do it once
        if (millis() - sMillisOfLastRefreshOrChangeBrightness < TIMEOUT_FOR_BRIGHTNESS_MILLIS) {
            changeBrightness();
        }
        sShowRawDataDisplay = DO_NOT_SHOW_RAW_DATA; // Reset display of only raw data
        drawDisplay();
        sMillisOfLastRefreshOrChangeBrightness = millis();
    }
}

/*
 * To be called by setup or main loop
 * Calls delayed initDisplay() and changeBrightness() functions to save stack
 * Checks for BlueDisplay events and returns true if event happened,
 * which may introduce an unknown delay the program might not be able to handle
 */
bool handleEventAndFlags() {
    sBDEventJustReceived = false;
    checkAndHandleEvents(); // BlueDisplay function
    checkAndHandlePowerLoggerEventFlags();
    return sBDEventJustReceived;
}

/*
 * GUI event handler section
 */

//This handler is called after boot or reconnect
void signalInitDisplay(void) {
    sDoInitDisplay = true;
}

void doSignalChangeBrightness(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    sDoRefreshOrChangeBrightness = true;
}

void doShowRawData(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    //Reset old max current value which leads to non shifting the current values and set state accordingly
    sShowRawDataDisplay = ACQUIRE_RAW_DATA_NO_SHIFT;
    sMaxCurrent = 0;
}

void doShowTimeToNextStorage(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
#if defined(USE_C_TIME)
    BlueDisplay1.getInfo(SUBFUNCTION_GET_INFO_LOCAL_TIME, getTimeEventCallbackForLogger); // set
#else
    now(); // update sysTime and prevMillis to current time
#endif

    printTimeToNextStorage();
    printTimeAtTwoLines(BUTTONS_START_X, TIME_INFO_START_Y, BASE_TEXT_SIZE, sTextColor, sBackgroundColor);
}

/*
 * Print power and estimated energy if not already done
 * 3 ms
 * @return  true if printing was done, false otherwise
 */
bool BD_checkAndPrintChangedPowerValue() {
    if (BlueDisplay1.isConnectionEstablished() && sShowRawDataDisplay == DO_NOT_SHOW_RAW_DATA) {
        return BD_printChangedPowerValue();
    }
    return false;
}

/*
 * Apply EMA lowpass alpha = 0.03125 | 1/32 to the power values. Cutoff frequency is 0.127 Hz @25Hz.
 * https://github.com/ArminJo/Arduino-Utils?tab=readme-ov-file#simpleemafilters
 * Skip lowpass if delta is more than 25 % (/4) or 12 % (/8), to avoid to have a slow increasing value at a fast power jump.
 * @return  true if printing was done, false otherwise
 */
#define LOOPS_FOR_5_SECONDS  (5 * MILLIS_IN_ONE_SECOND / DURATION_OF_ONE_LOOP_MILLIS) // 125
bool BD_printChangedPowerValue() {
    static uint8_t sNoDisplayCount;

    if (uintDifferenceAbs(sPowerAverageWattLowpass, sPowerAverageWatt) > (sPowerAverageWattLowpass / 4)) { // %25 percent is a reasonable heuristic value
        sPowerAverageWattLowpass = sPowerAverageWatt; // Fast response, reinitialize lowpass.
    } else {
        sPowerAverageWattLowpass += ((sPowerAverageWatt - sPowerAverageWattLowpass) + (1 << 4)) >> 5; // 2.5 us
    }

    if (sLastDisplayedPowerAverageWatt == sPowerAverageWattLowpass) {
        sNoDisplayCount = 0;
    } else {
        sNoDisplayCount++;
    }

    // Force print value if difference is > 1/32 or it is different for more than 5 seconds
    if ((uintDifferenceAbs(sLastDisplayedPowerAverageWatt, sPowerAverageWattLowpass) > (sLastDisplayedPowerAverageWatt / 32))
            || (sNoDisplayCount > LOOPS_FOR_5_SECONDS)) {
        // Power value has changed more than 3% -> print it
        snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings,
                sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("%4dW"), sPowerAverageWattLowpass);
        BlueDisplay1.drawText(CHART_START_X + BASE_TEXT_SIZE, BASE_TEXT_SIZE_HALF,
                sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, 3 * BASE_TEXT_SIZE, CHART_DATA_COLOR, sBackgroundColor); // 5 + 17 = 22 bytes are sent -> around 2 ms
        sLastDisplayedPowerAverageWatt = sPowerAverageWattLowpass;
        sNoDisplayCount = 0;
        return true;
    }

    return false;
}

void BD_printFloatValueAndUnit(uint16_t aPositionX, uint16_t aPositionY, float aValue, const char *aUnitString, uint16_t aFontSize,
        color16_t aTextColor, color16_t aBackgroundColor) {
    dtostrf(aValue, 4, 2, &sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings[10]); // 4 character + NL
    snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings),
            PSTR("%s%s"), &sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings[10], aUnitString);
    BlueDisplay1.drawText(aPositionX, aPositionY, sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings, aFontSize, aTextColor,
            aBackgroundColor); // 7 + 17 = 24 bytes are sent -> around 2.5 ms
}

void BD_printDaylyEnergyValue() {
    BD_printFloatValueAndUnit(CHART_START_X + (13 * BASE_TEXT_SIZE), BASE_TEXT_SIZE_HALF, sPowerAverageOfLast24HoursWatt * 0.024,
            "kWh", 2 * BASE_TEXT_SIZE, CHART_DATA_COLOR, sBackgroundColor); // 7 + 17 = 24 bytes are sent -> around 2.5 ms
}

void BD_printDayHistoricEnergyValues() {
    BD_printFloatValueAndUnit(CHART_START_X + (13 * BASE_TEXT_SIZE), BASE_TEXT_SIZE_HALF + 2 * BASE_TEXT_SIZE,
            sNoinitData.sEnergyOfLastDaysWattHourPerDay[1] / 1000.0, "kWh", BASE_TEXT_SIZE, CHART_DATA_COLOR, sBackgroundColor); // 7 + 17 = 24 bytes are sent -> around 2.5 ms
    BD_printFloatValueAndUnit(CHART_START_X + (18 * BASE_TEXT_SIZE), BASE_TEXT_SIZE_HALF + 2 * BASE_TEXT_SIZE,
            sNoinitData.sEnergyOfLastDaysWattHourPerDay[0] / 1000.0, "kWh", BASE_TEXT_SIZE, CHART_DATA_COLOR, sBackgroundColor); // 7 + 17 = 24 bytes are sent -> around 2.5 ms

}

void changeBrightness() {
    if (sCurrentBrightness == BRIGHTNESS_HIGH) {
        // Set to dimmed background
        BlueDisplay1.setScreenBrightness(BD_SCREEN_BRIGHTNESS_MIN);
        sCurrentBrightness = BRIGHTNESS_MIDDLE;
    } else if (sCurrentBrightness == BRIGHTNESS_MIDDLE) {
        // Set to dark mode
        sBackgroundColor = COLOR16_LIGHT_GREY;
        sTextColor = COLOR16_WHITE;
        PowerChart.setLabelColor(COLOR16_WHITE);
        PowerChart.setBackgroundColor(COLOR16_LIGHT_GREY);
        sCurrentBrightness = BRIGHTNESS_LOW;
    } else {
        // Back to user brightness
        sBackgroundColor = COLOR16_WHITE;
        sTextColor = COLOR16_BLACK;
        BlueDisplay1.setScreenBrightness(BD_SCREEN_BRIGHTNESS_USER);
        PowerChart.setLabelColor(COLOR16_BLACK);
        PowerChart.setBackgroundColor(COLOR16_WHITE);
        sCurrentBrightness = BRIGHTNESS_HIGH;
    }
}

/*
 * Time handling
 */
/*
 * Print difference between next storage time and now(), which is set by caller :-)
 * Serial output is displayed as an Android toast
 */
void printTimeToNextStorage() {
#if defined(USE_C_TIME)
#warning printTimeToNextStorage() is not yet supported for usage of plain C time library
#else
    int16_t tSecondsToNext = sNextStorageSeconds - sysTime;

    // give 10 seconds for internal delay / waiting for Power value
    if (tSecondsToNext < -10) {
        BlueDisplay1.debug(F("Timeout")); // signal, that seconds to next has an unexpected high negative value
    } else {
        if (tSecondsToNext < 0) {
            tSecondsToNext = 0;
        }

        uint8_t tMinutesToGo = tSecondsToNext / SECS_PER_MIN;
        uint8_t tSecondsToGo = tSecondsToNext % SECS_PER_MIN;
        snprintf_P(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings,
                sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("Next in %d min %02d sec"), tMinutesToGo,
                tSecondsToGo);
// !!!!! What a bug! This gives x min 00 sec with 00 constant because of wrong type of SECS_PER_MIN !!!!!
//        snprintf_P(tStringBuffer, sizeof(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings), PSTR("Next in %d min %02d sec"), tSecondsToNext / SECS_PER_MIN,
//                tSecondsToNext % SECS_PER_MIN);
        BlueDisplay1.debug(sNoinitData.Buffer.sStringBufferForLCDRowAndBDStrings); // Show as toast
//        Serial.print(F("Next in "));
//        if (tSecondsToNext / SECS_PER_MIN > 0) {
//            Serial.print(tSecondsToNext / SECS_PER_MIN);
//            Serial.print(F(" min "));
//        }
//        Serial.print(tSecondsToNext % SECS_PER_MIN);
//        Serial.println(F(" sec"));
    }
#endif
}

/*
 * It is called at startup and then every 24 hours.
 */
void getTimeEventCallbackForLogger(uint8_t aSubcommand, uint8_t aByteInfo, uint16_t aShortInfo, ByteShortLongFloatUnion aLongInfo) {
    (void) aSubcommand;
    (void) aByteInfo;
    (void) aShortInfo;

#if defined(USE_C_TIME)
        BlueDisplay1.setHostUnixTimestamp(aLongInfo.uint32Value);
        sTimeInfoWasJustUpdated = true;
        sTimeInfo = localtime((const time_t*) &aLongInfo.uint32Value); // Compute minutes, hours, day, month etc. for later use in printTime
#else

    setTime(aLongInfo.uint32Value);
    time_t tTimestamp = now(); // use this timestamp for display etc.

    // tweak sNextStorageMillis, so that our next storage is at the next full 5 minute
    uint16_t tSecondsUntilNextFull5Minutes = (STORAGE_INTERVAL_SECONDS
            - ((minute(tTimestamp) * SECS_PER_MIN) + second(tTimestamp)) % STORAGE_INTERVAL_SECONDS);
    sNextStorageSeconds = tTimestamp + tSecondsUntilNextFull5Minutes;
#endif

    delay(400); // wait for the scale factor toast to vanish
    printTimeToNextStorage();
#if defined(LOCAL_TRACE)
    Serial.print(sTimestamp);
    Serial.print('|');
    Serial.println(sNextStorageSeconds);
#endif
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _POWER_LOGGER_AND_CHART_HPP
