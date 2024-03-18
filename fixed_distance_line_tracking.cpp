#include "utils.hpp"
#include <Arduino.h>
#include <cmath>

// Define pins for the IR sensors
#define IR_SENSOR_LEFT PA6      // Left IR sensor pin
#define IR_SENSOR_RIGHT PA7     // Right IR sensor pin

#define MOTOR_A_SPEED_PIN PB1    // Motor A Speed Control
#define MOTOR_A_DIR_PIN   PB0   // Motor A Direction Control
#define MOTOR_B_SPEED_PIN PA5   // Motor B Speed Control
#define MOTOR_B_DIR_PIN   PB3   // Motor B Direction Control

#define ENCODER_LEFT PA2
#define ENCODER_RIGHT PA4

#define PIN_LEFT PB7            // LED for left turn
#define PIN_RIGHT PB6           // LED for right turn
#define PIN_STOP PB15

#define THRESHOLD 400

int leftSensorValue, rightSensorValue;
const int PPR = 40;
const float wheelRadius = 3.5; // Wheel radius in cm
const float wheelCircumference = 2 * M_PI * wheelRadius;
int pulseCountLeft = 0;
int pulseCountRight = 0;
float targetRotations;

// Variables to keep track of the last interrupt time
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
const unsigned long debounceDelay = 10; // Debounce delay in milliseconds

void countLeftPulse() {
    unsigned long currentTime = millis();
    if (currentTime - lastLeftPulseTime > debounceDelay) {
        pulseCountLeft++;
        lastLeftPulseTime = currentTime;
    }
}

void countRightPulse() {
    unsigned long currentTime = millis();
    if (currentTime - lastRightPulseTime > debounceDelay) {
        pulseCountRight++;
        lastRightPulseTime = currentTime;
    }
}

float calculateTargetRotations(float distanceMeters) {
    float distanceCm = distanceMeters * 100; // Convert meters to cm
    return distanceCm / wheelCircumference;
}

void setup() {
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_DIR_PIN, OUTPUT);

    pinMode(PIN_LEFT, OUTPUT);
    pinMode(PIN_RIGHT, OUTPUT);
    pinMode(PIN_STOP, OUTPUT);

    pinMode(ENCODER_LEFT, INPUT);
    pinMode(ENCODER_RIGHT, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countLeftPulse, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countRightPulse, RISING);


    // Set the target distance here, for example:
    float targetDistance = 1.9; // This value will be given during the demo
    targetRotations = calculateTargetRotations(targetDistance);

    digitalWrite(PIN_STOP, LOW);
}


void loop() {
    digitalWrite(PIN_LEFT, HIGH);
    digitalWrite(PIN_RIGHT, HIGH);
    leftSensorValue = analogRead(IR_SENSOR_LEFT);
    rightSensorValue = analogRead(IR_SENSOR_RIGHT);

    if (leftSensorValue < THRESHOLD && rightSensorValue < THRESHOLD) {
        digitalWrite(PIN_LEFT, HIGH);
        digitalWrite(PIN_RIGHT, HIGH);

        analogWrite(MOTOR_A_SPEED_PIN, 80);
        analogWrite(MOTOR_A_DIR_PIN, 255);
        analogWrite(MOTOR_B_SPEED_PIN, 80);
        analogWrite(MOTOR_B_DIR_PIN, 255);
    } else if (leftSensorValue < THRESHOLD && rightSensorValue > THRESHOLD) {
        digitalWrite(PIN_LEFT, LOW);
        digitalWrite(PIN_RIGHT, HIGH);

        analogWrite(MOTOR_A_SPEED_PIN, 80);
        analogWrite(MOTOR_B_SPEED_PIN, 255);
        analogWrite(MOTOR_A_DIR_PIN, 255);
        analogWrite(MOTOR_B_DIR_PIN, 80);
        
    } else if (leftSensorValue > THRESHOLD && rightSensorValue < THRESHOLD) {
        digitalWrite(PIN_RIGHT, LOW);
        digitalWrite(PIN_LEFT, HIGH);

        analogWrite(MOTOR_A_DIR_PIN, 80);
        analogWrite(MOTOR_A_SPEED_PIN, 255);
        analogWrite(MOTOR_B_DIR_PIN, 255);
        analogWrite(MOTOR_B_SPEED_PIN, 80);   
    }

    float rotationsLeft = pulseCountLeft / (float)PPR;
    float rotationsRight = pulseCountRight / (float)PPR;
    float averageRotations = (rotationsLeft + rotationsRight) / 2.0;

    if (averageRotations >= targetRotations) {
        // Stop the motors
        digitalWrite(PIN_STOP, HIGH);

        analogWrite(MOTOR_A_SPEED_PIN, 255);
        analogWrite(MOTOR_A_DIR_PIN, 255);
        analogWrite(MOTOR_B_SPEED_PIN, 255);
        analogWrite(MOTOR_B_DIR_PIN, 255);
        return;
    }
}
