#include "utils.hpp"
#include <Arduino.h>

// Define pin assignments
#define MOTOR1_PIN PB1
#define MOTOR2_PIN PA5
#define ULTRASOUND_TRIGGER_PIN PA0
#define ULTRASOUND_ECHO_PIN PB10

// Define the threshold distance for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD_DISTANCE 15

// Define initial motor speeds
#define INITIAL_SPEED_MOTOR1 90
#define INITIAL_SPEED_MOTOR2 90

// Variables to store ultrasonic sensor data
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile boolean triggered = false;

// Interrupt service routine for handling echo pin changes
void handleEcho() {
    if (digitalRead(ULTRASOUND_ECHO_PIN) == HIGH) {
        // Rising edge, record start time
        startTime = micros();
    } else {
        // Falling edge, record end time and set triggered flag
        endTime = micros();
        triggered = true;
    }
}

void setup() {
    // Initialize ultrasound sensor pins
    pinMode(ULTRASOUND_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASOUND_ECHO_PIN, INPUT);

    // Initialize motor pins
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);

    // Attach an interrupt to the echo pin to measure pulse duration
    attachInterrupt(digitalPinToInterrupt(ULTRASOUND_ECHO_PIN), handleEcho, CHANGE);
}

void loop() {
    // Trigger the ultrasound sensor
    digitalWrite(ULTRASOUND_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASOUND_TRIGGER_PIN, LOW);

    // Wait for the triggered flag to be set
    while (!triggered);

    // Calculate the duration of the echo pulse
    unsigned long duration = endTime - startTime;

    // Calculate the distance in centimeters
    long distance = (duration * 0.034) / 2;

    // Reset the triggered flag
    triggered = false;

    // Check if an obstacle is closer than the threshold distance
    if (distance < OBSTACLE_THRESHOLD_DISTANCE) {
        // Stop the motors when an obstacle is detected
        analogWrite(MOTOR1_PIN, 255); // Stop motor 1
        analogWrite(MOTOR2_PIN, 255); // Stop motor 2
    } else {
        // Set the motors to initial speeds for normal operation
        analogWrite(MOTOR1_PIN, INITIAL_SPEED_MOTOR1);
        analogWrite(MOTOR2_PIN, INITIAL_SPEED_MOTOR2);
    }

    delay(50); // Add a small delay to reduce processing load
}
