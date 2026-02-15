# STM32 Temperature Warning System

## Overview

Real-time temperature monitoring system built on STM32 using mbed OS.
The system reads temperature from an I2C sensor, compares it to user-defined thresholds, and triggers audio/visual alarms.
An LCD displays the current temperature and min/max values from the last hour.

## Features

* I2C temperature sensor (DS1631)
* User-adjustable upper and lower thresholds
* Buzzer alarm with different sound patterns
* LED indicators for high and low alarms
* LCD display (SPI shift register interface)
* RTOS-based architecture:

  * Sensor thread
  * Alarm thread
  * LCD thread
* Rolling 1-hour min/max temperature tracking

## Hardware

* STM32 development board
* DS1631 temperature sensor (I2C)
* 16x2 or 20x4 LCD with shift register (SPI)
* Buzzer (PWM)
* 2 LEDs
* 4 push buttons

## System Architecture

The system uses three RTOS threads:

1. **Sensor Thread**

   * Reads temperature every second
   * Updates min/max values
   * Sends data to LCD thread
   * Triggers alarm flags

2. **Alarm Thread**

   * Waits for alarm flags
   * Activates buzzer and LEDs
   * Uses different patterns for high/low alarms

3. **LCD Thread**

   * Receives temperature updates
   * Displays current, max, and min values

## Folder Structure

```
Core/      → main application code
Drivers/   → STM32 or mbed drivers
docs/      → diagrams
images/    → hardware photos
```

## How to Build

1. Open project in mbed or STM32 environment.
2. Compile and flash to the STM32 board.
3. Connect hardware as described.
4. Power the board.

## Demo
### Hardware Setup
<img width="2000" height="1125" alt="image" src="https://github.com/user-attachments/assets/4bbc9dc2-574a-49d0-9134-fb4a4bcaef9b" />

STM32-based temperature monitoring system with DS1631 sensor,
LCD display, buzzer, LEDs, and user input buttons.

