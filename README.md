# Line Follower Robot with Obstacle Detection and Variable Speed Control

## Project Overview

This repository contains the code and documentation for a Line Follower Robot developed for the CSE2425 course at TU Delft. The robot is designed to follow a line with the ability to detect obstacles and adjust its speed dynamically. It showcases the integration and application of various sensors and motor control techniques using the STM32F401 microcontroller.

## Robot components

- 1x STM32F401 microcontroller
- 2x HC-SR04 ultrasound ranging sensor
- 1x L9110 H-Bridge
- 2x TCRT5000 Infrared reflective sensor
- 2x B83609 speed encoder
- 2x Motors with gearbox
- 2x Slotted wheels
- 2x Wheels
- 1x Caster wheel
- 1x 5000mAh Mi Power bank
- 1x USB Power Splitter
- 1x Micro USB cable
- 2 Red LEDs
- Wooden frames
- 1x Breadboard
- 1x Power Module 5V/3.3V
- 1x bag of screws and nuts
- Jumper wires
- ST LINK Stlink ST-Link V2 Mini

## Features

1. **Variable Speed Control:** Implements initial constant speed, increased speed phase, and gradual deceleration.
2. **Obstacle Detection:** Utilizes ultrasound sensors to detect obstacles within a 15cm range and stops the robot.
3. **Line Following:** Uses infrared sensors to follow a 2cm line on the ground.
4. **Enhanced Obstacle Avoidance:** Combines infrared and ultrasound sensors for navigating around obstacles and returning to the line.
5. **Fixed Distance Line Tracking:** Follows a line for a predetermined distance using encoder feedback for precise movement.