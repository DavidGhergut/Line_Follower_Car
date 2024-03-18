#include "utils.hpp"
#include <Arduino.h>

// Define pins for the IR sensors
#define IR_SENSOR_LEFT PA6      // Left IR sensor pin
#define IR_SENSOR_RIGHT PA7     // Right IR sensor pin

#define MOTOR_A_SPEED_PIN PB1   // Motor A Speed Control
#define MOTOR_A_BACK_PIN   PB0   // Motor A Direction Control
#define MOTOR_B_SPEED_PIN PA5   // Motor B Speed Control
#define MOTOR_B_BACK_PIN   PB3   // Motor B Direction Control

#define PIN_LEFT PB7            // LED for left turn
#define PIN_RIGHT PB6           // LED for right turn

#define THRESHOLD 400

int leftSensorValue, rightSensorValue;

void setup() {
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_A_BACK_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_BACK_PIN, OUTPUT);

    pinMode(PIN_LEFT, OUTPUT);
    pinMode(PIN_RIGHT, OUTPUT);
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
        analogWrite(MOTOR_A_BACK_PIN, 255);
        analogWrite(MOTOR_B_SPEED_PIN, 80);
        analogWrite(MOTOR_B_BACK_PIN, 255);
    } else if (leftSensorValue < THRESHOLD && rightSensorValue > THRESHOLD) {
        digitalWrite(PIN_LEFT, LOW);
        digitalWrite(PIN_RIGHT, HIGH);

        analogWrite(MOTOR_A_SPEED_PIN, 80);
        analogWrite(MOTOR_B_SPEED_PIN, 255);
        analogWrite(MOTOR_A_BACK_PIN, 255);
        analogWrite(MOTOR_B_BACK_PIN, 80);
        
    } else if (leftSensorValue > THRESHOLD && rightSensorValue < THRESHOLD) {
        digitalWrite(PIN_RIGHT, LOW);
        digitalWrite(PIN_LEFT, HIGH);

        analogWrite(MOTOR_A_BACK_PIN, 80);
        analogWrite(MOTOR_A_SPEED_PIN, 255);
        analogWrite(MOTOR_B_BACK_PIN, 255);
        analogWrite(MOTOR_B_SPEED_PIN, 80);   
    }
}
