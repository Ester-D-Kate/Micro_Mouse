#include <Arduino.h>

#define LEFT_MOTOR_FWD 8
#define LEFT_MOTOR_BWD 9
#define RIGHT_MOTOR_FWD 6
#define RIGHT_MOTOR_BWD 7
#define PWM_LEFT 10
#define PWM_RIGHT 5

String inputData = "";
int rightSpeedValue = 0; 
int leftSpeedValue = 0; 
char direction = 'S';
char turnDir = 'N';

void controlMotors() {
  int leftSpeed = leftSpeedValue;
  int rightSpeed = rightSpeedValue;

switch (direction) {
    case 'R': // Right
        digitalWrite(LEFT_MOTOR_FWD, HIGH);
        digitalWrite(LEFT_MOTOR_BWD, LOW);
        digitalWrite(RIGHT_MOTOR_FWD, HIGH);
        digitalWrite(RIGHT_MOTOR_BWD, LOW);
        break;
    case 'L': // Left
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BWD, HIGH);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BWD, HIGH);
        break;
    case 'B': // Backward
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BWD, HIGH);
        digitalWrite(RIGHT_MOTOR_FWD, HIGH);
        digitalWrite(RIGHT_MOTOR_BWD, LOW);
        break;
    case 'F': // Forward
        digitalWrite(LEFT_MOTOR_FWD, HIGH);
        digitalWrite(LEFT_MOTOR_BWD, LOW);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BWD, HIGH);
        break;
    case 'S': // Stop
        digitalWrite(LEFT_MOTOR_FWD, LOW);
        digitalWrite(LEFT_MOTOR_BWD, LOW);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
        digitalWrite(RIGHT_MOTOR_BWD, LOW);
        break;
}

  // Apply PWM speed control
  analogWrite(PWM_LEFT, leftSpeed);
  analogWrite(PWM_RIGHT, rightSpeed);
}

void setup() {
    Serial.begin(115200);

    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BWD, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BWD, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        inputData = Serial.readStringUntil('\n'); // Read command from ESP8266

        if (inputData.length() > 5) {
            // Parse values from the command format: "<dir,rightSpeed,leftSpeed,N,0>"
            direction = inputData.charAt(1);
            int firstComma = inputData.indexOf(',');
            int secondComma = inputData.indexOf(',', firstComma + 1);
            int thirdComma = inputData.indexOf(',', secondComma + 1);

            leftSpeedValue = inputData.substring(firstComma + 1, secondComma).toInt();
            rightSpeedValue = inputData.substring(secondComma + 1, thirdComma).toInt();
            turnDir = inputData.charAt(thirdComma + 1);
        }
    }

    controlMotors();
}

