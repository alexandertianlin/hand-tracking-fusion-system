# Glove Sensor System

STM32 firmware for wearable IMU + tactile sensor glove system.

## Projects

- **Agilereach_STM32H523_I2C_V2_IMU-Glove/** - STM32H523-based IMU glove firmware (I2C V2)
- **Agilereach_STM32H70B_WIFI_6IT-Glove/** - STM32H70B-based glove firmware with WiFi communication

## Overview

Firmware for a sensor glove system that captures hand motion via IMUs and tactile sensors. Data is transmitted to a host computer (Unity-based visualization) via serial (I2C) or WiFi depending on the variant.

## Hardware

- STM32H523 / STM32H70B microcontrollers
- Multiple IMU sensors for finger tracking
- Tactile/force sensors
- WiFi module (H70B variant) for wireless data transmission
- I2C communication bus (H523 variant)
