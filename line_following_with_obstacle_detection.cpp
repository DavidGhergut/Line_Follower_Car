#include "utils.hpp"
#include <Arduino.h>

// Define pins for the IR sensors
#define IR_SENSOR_LEFT PA6      // Left IR sensor pin
#define IR_SENSOR_RIGHT PA7     // Right IR sensor pin

#define MOTOR_A_SPEED_PIN PB1   // Motor A Speed Control
#define MOTOR_A_BACK_PIN   PB0   // Motor A Direction Control
#define MOTOR_B_SPEED_PIN PA5   // Motor B Speed Control
#define MOTOR_B_BACK_PIN   PB3   // Motor B Direction Control

#define ENCODER_LEFT PA2
#define ENCODER_RIGHT PA4

#define ULTRASOUND_TRIGGER_PIN PA0
#define ULTRASOUND_ECHO_PIN PB10

// Define the threshold distance for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD_DISTANCE 15

// Define initial motor speeds
#define INITIAL_SPEED_MOTOR1 90
#define INITIAL_SPEED_MOTOR2 90

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

// Variables to store ultrasonic sensor data
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile boolean triggered = false;


enum CarState {
    FOLLOW_LINE,
    DETECT_OBSTACLE,
    TURN_RIGHT,
    MOVE_FORWARD_20CM_FIRST,
    MOVE_FORWARD_20CM_SECOND,
    TURN_LEFT_FIRST,
    TURN_LEFT_SECOND,
    SEARCH_LINE,
    STOP
};

CarState currentState = FOLLOW_LINE;

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

void moveForward() {
    analogWrite(MOTOR_A_SPEED_PIN, 50);
    analogWrite(MOTOR_A_BACK_PIN, 255);
    analogWrite(MOTOR_B_SPEED_PIN, 50);
    analogWrite(MOTOR_B_BACK_PIN, 255);
}

void turnRight() {
    analogWrite(MOTOR_A_SPEED_PIN, 30);
    analogWrite(MOTOR_B_SPEED_PIN, 255);
    analogWrite(MOTOR_A_BACK_PIN, 255);
    analogWrite(MOTOR_B_BACK_PIN, 90);
}

void turnLeft() {
    analogWrite(MOTOR_A_BACK_PIN, 90);
    analogWrite(MOTOR_A_SPEED_PIN, 255);
    analogWrite(MOTOR_B_BACK_PIN, 255);
    analogWrite(MOTOR_B_SPEED_PIN, 30);
}

void stop() {
    // Stop motor 1
    analogWrite(MOTOR_A_SPEED_PIN, 255); 
    analogWrite(MOTOR_A_BACK_PIN, 255);
    // Stop motor 2
    analogWrite(MOTOR_B_SPEED_PIN, 255);
    analogWrite(MOTOR_B_BACK_PIN, 255);
}

void turnRight90Degrees() {
    analogWrite(MOTOR_A_SPEED_PIN, 40);
    analogWrite(MOTOR_B_SPEED_PIN, 255);
    analogWrite(MOTOR_A_BACK_PIN, 255);
    analogWrite(MOTOR_B_BACK_PIN, 90);

    unsigned long turnDuration = 900; // Calibrate this duration
    unsigned long startTime = millis();

    while (millis() - startTime < turnDuration) {
        // Just waiting for the duration to pass
    }

    stop(); // Stop the motors after turning
}

void turnLeft90Degress() {
    analogWrite(MOTOR_A_BACK_PIN, 90);
    analogWrite(MOTOR_A_SPEED_PIN, 255);
    analogWrite(MOTOR_B_BACK_PIN, 255);
    analogWrite(MOTOR_B_SPEED_PIN, 30);

    unsigned long turnDuration = 900; // Calibrate this duration
    unsigned long startTime = millis();

    while (millis() - startTime < turnDuration) {
        // Just waiting for the duration to pass
    }

    stop(); // Stop the motors after turning
}


void moveForwardDistance(float distance) {
    // Calculate target rotations based on the distance
    float targetRotations = calculateTargetRotations(distance / 100.0); // Convert cm to meters
    
    // Reset encoder counts
    pulseCountLeft = 0;
    pulseCountRight = 0;
    
    // Move forward and continuously check if the target distance is reached
    while (true) {
        float rotationsLeft = pulseCountLeft / (float)PPR;
        float rotationsRight = pulseCountRight / (float)PPR;
        float averageRotations = (rotationsLeft + rotationsRight) / 2.0;

        if (averageRotations >= targetRotations) {
            break;
        }
        else {
            moveForward();
        }
    }

    // Stop the motors
    stop();
}

void setup() {
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    // Initialize ultrasound sensor pins
    pinMode(ULTRASOUND_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASOUND_ECHO_PIN, INPUT);

    // Initialize motor pins
    pinMode(MOTOR_A_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_A_BACK_PIN, OUTPUT);
    pinMode(MOTOR_B_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_B_BACK_PIN, OUTPUT);

    // Attach an interrupt to the echo pin to measure pulse duration
    attachInterrupt(digitalPinToInterrupt(ULTRASOUND_ECHO_PIN), handleEcho, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countLeftPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countRightPulse, CHANGE);

    pinMode(PIN_LEFT, OUTPUT);
    pinMode(PIN_RIGHT, OUTPUT);
    pinMode(PIN_STOP, OUTPUT);

    digitalWrite(PIN_STOP, LOW);
}

void loop () {
    // Read IR sensor values for line following 
    leftSensorValue = analogRead(IR_SENSOR_LEFT);
    rightSensorValue = analogRead(IR_SENSOR_RIGHT);

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

    switch (currentState) {
        case FOLLOW_LINE:
            if (leftSensorValue < THRESHOLD && rightSensorValue < THRESHOLD) {
                moveForward();
            } else if (leftSensorValue < THRESHOLD) {
                turnRight();
            } else if (rightSensorValue < THRESHOLD) {
                turnLeft();
            }

            if (distance < OBSTACLE_THRESHOLD_DISTANCE) {
                currentState = TURN_RIGHT;
            }
            break;
  
        case TURN_RIGHT:
            turnRight90Degrees();
            currentState = MOVE_FORWARD_20CM_FIRST;
            break;

        case MOVE_FORWARD_20CM_FIRST:
            moveForwardDistance(12.0);
            currentState = TURN_LEFT_FIRST;
            break;

        case TURN_LEFT_FIRST:
            turnLeft90Degress();
            currentState = MOVE_FORWARD_20CM_SECOND;
            break;

        case MOVE_FORWARD_20CM_SECOND:
            moveForwardDistance(15.0);
            currentState = TURN_LEFT_SECOND;
            break;

        case TURN_LEFT_SECOND:
            turnLeft90Degress();
            currentState = SEARCH_LINE;
            break;

        case SEARCH_LINE:
            // Move forward until the line is found
            moveForward();
            if (leftSensorValue < THRESHOLD || rightSensorValue < THRESHOLD) {
                currentState = FOLLOW_LINE;
            }
            break;

        case STOP:
            stop();
            break;
    }
}