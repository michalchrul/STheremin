# Theremin for STM32F407VG (Discovery)

# Overview
It is a digital imitation of a theremin instrument using ultrasonic sensors to control the frequency and the amplitude.

# Additional hardware used:
	2x HC-SR04 Ultrasonic sensors

# Additional connections used:
	Sensor #1: Trigger <---> PE5 | Echo <---> PD0  (Distance_1 for the frequency)
	Sensor #2: Trigger <---> PE7 | Echo <---> PB12 (Distance_2 for the amplitude)

# How to run?
After connecting the pins properly simply download the project and compile it with the CooCox CoIDE.

# To do
- Fix the CODEC_I2S function usage for the amplitude modulation
# Possible future Improvements

- Waveform selection
- Simple delay effect

# Notes
 This program contains code written by Andreas Finkelmeyer which was shared here:
 http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio
 <br />
 
Contact: michalchrul@gmail.com
