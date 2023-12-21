<div align = center>

# [Arduino Nano substitute for a DTSU666-H power meter](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter)

Substitutes the RS485 modbus service of DTSU-H coil based 3 phase power meter for usage with a Deye hybrid inverter.<br/>
On phase L1 it reads every 4. current waves, on phase L2 and L3 it reads only every 8. half wave.<br/>

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp;
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/Arduino-DTSU666H_PowerMeter?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/releases/latest)
 &nbsp; &nbsp;
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Arduino-DTSU666H_PowerMeter/latest?color=yellow)](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/commits/main)
 &nbsp; &nbsp;
[![Badge Build Status](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/actions)
 &nbsp; &nbsp;
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Arduino-DTSU666H_PowerMeter)
<br/>

</div>

#### If you find this program useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter)

<br/>

# Features
- Astonishing accuracy of 1% at my site.
- Every watt-hour, the build-in LED flashes for 80 ms.
- 2 buttons to set a correction percentage other than 100% - which was not required at my site :-).
- Display of Power, Energy, Time for energy and correction percentage on a locally attached 1602 LCD.
- Page button for switching 4 LCD display pages.
- Debug button/switch for realtime monitoring analog current and voltage of 4 different half waves with Arduino Plotter.
- Serial.print() function is still available for monitoring and debugging.

<br/>

# Principle of operation
**We assume, that voltage waveform of the 3 phases are equal and negative and positive values are symmetric, so we take only one half wave as voltage reference.**

**Phase A** is reference (the one, which supplies the voltage and which can be negative), **phase B** has a delay of 6.6 ms / 120 &deg; and **C** a delay of 13.3 ms / 240 &deg;.

1. Read "positive" part voltage values of phase A for 10 ms and store 385 values in RAM to be used as reference for all 3 phases. This starts 16 Bit timer 1 to keep track of the phase for the next current measurements.
2. Wait 3.3 ms.
3. After 10 + 3.333 ms do 10 ms phase C current measurement. Multiply values with voltage.
4. Wait 3.3 ms.
5. After 20 + 6.666 ms do 10 ms phase B current measurement. Multiply values with voltage.
6. Wait 3.3 ms
7. After 40 ms do 20 ms phase A current measurement, which will also cover negative current. Multiply values with voltage.
8. 20 ms for math and reply to RS485 or output to LCD until starting again at 80 ms - x ms.

The Deye inverter sends a 9600 baud modbus request 01 03  15 1E  00 06  A1 C2 every 100 ms to 120 ms.<br/>
We send the reply at pin 2 with software serial at a 80 ms raster.<br/>
This means, around every 400 ms we have one loop where we do not need to reply and can update the LCD instead.<br/>
Sometimes the Deye sends the request, while we do a reply.

<br/>

# Pictures

| My installation | Display and buttons |
| :-: | :-: |
| ![My installation](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/Overview.jpg) | ![Display and buttons](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/DisplayAndButtons.jpg) |
| Breadboard detail | Automatic brightness |
| ![Breadboard detail](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/Breadboard.jpg) | ![Automatic brightness](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/AutomaticBrightness.jpg) |
| Split-core current transformers used | Deye settings |
| ![Split-coil](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/SplitCoil.jpg) | ![Deye settings](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/DeyeSettingsForDTDSU666.jpg) |

<br/>

# Screenshots
## LCD pages
| Power page | Energy page |
| :-: | :-: |
| ![Power page](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/PowerPage.jpg) | ![Energy page](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/EnergyPage.jpg) |
| Time for Energy page | Period page |
| ![Time for Energy page](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/EnergyTimePage.jpg) | ![Period page](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/PeriodPage.jpg) |

## Arduino plotter output of measured currents
| Reference phase - positive half wave | Reference phase - negative half wave |
| :-: | :-: |
| ![Reference positive](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/Reference.png) | ![Reference negative](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/Reference_negative.png) |
| 6.6 ms / 120 &deg; delayed phase | 13.2 ms / 240 &deg; delayed phase |
| ![7 ms](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/7ms.png) | ![13 ms](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/13ms.png) |
| 6.6 ms / 120 &deg; without low-pass | 13.2 ms / 240 &deg; without low-pass |
| ![7 ms without low-pass](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/7ms_WithoutLowpass.png) | ![13 ms without low-pass](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/blob/main/pictures/13ms_WithoutLowpass.png) |

<br/>


```
 Implemented with 3 x 30A split-core current transformers.
 2. order low pass 1600 Hz, to suppressthe high frequency modulation of current as seen above.
               ____         ____         _____
          o---|____|---o---|____|---o---|_____|---o-----> A1 (A2, A3)
          |     1k     |     1k     |     10k     |
 Split-   >            |            |             |
 core    <            ---          ---            | Schottky diode
 coil     >           --- 100 nF   --- 100 nF     o-|<|-o
         <             |            |                   |
        __|____________|____________|___________________|__ GND




Grid voltage divider for full range (1.1 V) at 400 V.

HIGH VOLTAGE - DANGER - use heat-shrink tubing
             ____     ____     ____     _____
230V~ L1 >--|____|---|____|---|____|---|_____|---o-----o-----> A0
              1M       1M       1M      630 k    |     |
                                                 _     |
                                                | |    | Schottky diode
                                                | |    o-|<|-o
                                                |_|          |
                                                 |           |
                                               __|___________|__ N (GND)




  # Automatic brightness control for 1602 LCD
   5V O------o------o
             |      |
             _      |
            | |     |
        LDR | |     |
            |_|     |
             |    |/
             o----|
                  |>
                    |
                    O To anode of LCD backlight

```

<br/>

# Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the Arduino-DTSU666H_PowerMeter folder.<br/>
All libraries, especially the modified ones, are included in this project.

<br/>

# Libraries used
This program uses the following libraries, which are already included in this repository:

- [SoftwareSerialTX](https://reference.arduino.cc/reference/en/libraries/liquidcrystal-i2c/) for sending Serial to JK-BMS.
- Modified [LiquidCrystal](http://www.arduino.cc/en/Reference/LiquidCrystal) for 1602 LCD.
- [EasyButtonAtInt01](https://github.com/ArminJo/EasyButtonAtInt01) for LCD page switching button.

 <br/>

# BOM

- Breadboard.
- Jumper wire and cables.
- Pin header to connect cables to breadboard.
- Arduino Nano.
- TTL to RS485 converter module with 74HC04 and MAX485.
- 10 nF capacitor to increase the C2 capacitor on the module, preventing the module (using 74HC04 instead of 74HC06) from ringing.
- 4 Schottky diodes for the 4 analog pins used e.g. BAT 42, BAT 43, BAT 85.
- 6 x 100 nF capacitors and 6 x 1 kOhm resistors for the 3 low-passes.
- 3 x 1 MOhm + 630 (680) kOhm + 10 kOhm for the 230 V voltage divider.
- Power supply.
- 1602 LCD.
- LDR for automatic LCD brightness control.
- BC 549 or any NPN type for automatic LCD brightness control. The effect varies, depending on the LDR and the hFE of the transistor.

<br/>

# Tested Inverter
- SUN-5K-SG05LP1-EU

# Useful Links
- [Modbus registers map](https://www.aggsoft.com/serial-data-logger/tutorials/modbus-data-logging/chint-instrument-dsu666-dtsu666.htm)

# Revision History

### Version 1.1
- Added reply to 0x2014 (Power in 0.1 W units) and 0x0006 (Current transformer rate IrAt).

### Version 1.0
- Initial version.

# Sample Serial output
See also [here](https://github.com/ArminJo/Arduino-DTSU666H_PowerMeter/tree/main/extras).

```
START ../src/Arduino-DTSU666H_PowerMeter.cpp
Version 1.0 from Sep 19 2023_
If you connect periodically print pin A4 to ground, Serial plotter data is printed every 2000 ms
Power correction + pin is 5
Power correction - pin is 4
LCD data is printed every (8 * 80) ms
Reference line, which can be negative is 2
Line, with 6.6 ms delay is 1
Line, with 13.3 ms delay is 3
Waiting for requests to ID MODBUS_SLAVE_ADDR with _9600_ baud at pin RX. Reply / TX is on pin 2
First waiting for voltage at line2
Power correction is 100 %
```
