#include "utils.h"
#include <Arduino.h>

#define MOTOR1_PIN PB1
#define MOTOR2_PIN PA5
#define INITIAL_SPEED1 100
#define INITIAL_SPEED2 100
#define HIGHER_SPEED1 60
#define HIGHER_SPEED2 60
#define MAX_SPEED 255
#define MIN_SPEED 0

unsigned long startTime;
const long phaseDuration = 4000; // 4 seconds for each phase
int phase = 0; // Variable used for stopping the loop 

void setup() {
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    startTime = millis(); // Record the start time
}

void loop() {
    if (phase == 0){
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - startTime;

        analogWrite(MOTOR1_PIN, INITIAL_SPEED1);
        analogWrite(MOTOR2_PIN, INITIAL_SPEED2);
        delay(4000);
        // Increase speed
        analogWrite(MOTOR1_PIN, HIGHER_SPEED1);
        analogWrite(MOTOR2_PIN, HIGHER_SPEED2);
        delay(4000); 
        // Decrease speed
        for (int i = 0; i <= 7; i++){
            analogWrite(MOTOR1_PIN, 60 + i * 30);
            analogWrite(MOTOR2_PIN, 60 + i * 30);
            delay(300);
        }
        phase = 1;
    }
}
