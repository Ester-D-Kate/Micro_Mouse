#include <Arduino.h>
#define LEFT_MOTOR_FWD 8
#define LEFT_MOTOR_BWD 9
#define RIGHT_MOTOR_FWD 6
#define RIGHT_MOTOR_BWD 7
#define PWM_LEFT 10
#define PWM_RIGHT 5

String inputData = "";
int speedValue = 0;
int turnIntensity = 0;
char direction = 'S';
char turnDir = 'N';

void controlMotors() {
  int leftSpeed = speedValue;
  int rightSpeed = speedValue;

  if (turnDir == 'L') rightSpeed -= turnIntensity;
  if (turnDir == 'R') leftSpeed -= turnIntensity;

  if (leftSpeed < 30) leftSpeed = 30;
  if (rightSpeed < 30) rightSpeed = 30;

  switch (direction) {
      case 'F':
          digitalWrite(LEFT_MOTOR_FWD, HIGH);
          digitalWrite(LEFT_MOTOR_BWD, LOW);
          digitalWrite(RIGHT_MOTOR_FWD, HIGH);
          digitalWrite(RIGHT_MOTOR_BWD, LOW);
          break;
      case 'B':
          digitalWrite(LEFT_MOTOR_FWD, LOW);
          digitalWrite(LEFT_MOTOR_BWD, HIGH);
          digitalWrite(RIGHT_MOTOR_FWD, LOW);
          digitalWrite(RIGHT_MOTOR_BWD, HIGH);
          break;
      case 'L':
          digitalWrite(LEFT_MOTOR_FWD, LOW);
          digitalWrite(LEFT_MOTOR_BWD, HIGH);
          digitalWrite(RIGHT_MOTOR_FWD, HIGH);
          digitalWrite(RIGHT_MOTOR_BWD, LOW);
          break;
      case 'R':
          digitalWrite(LEFT_MOTOR_FWD, HIGH);
          digitalWrite(LEFT_MOTOR_BWD, LOW);
          digitalWrite(RIGHT_MOTOR_FWD, LOW);
          digitalWrite(RIGHT_MOTOR_BWD, HIGH);
          break;
      case 'S':
          digitalWrite(LEFT_MOTOR_FWD, LOW);
          digitalWrite(LEFT_MOTOR_BWD, LOW);
          digitalWrite(RIGHT_MOTOR_FWD, LOW);
          digitalWrite(RIGHT_MOTOR_BWD, LOW);
          break;
  }

  analogWrite(PWM_LEFT, leftSpeed * 2.55);
  analogWrite(PWM_RIGHT, rightSpeed * 2.55);
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
        inputData = Serial.readStringUntil('\n');

        if (inputData.length() > 5) {
            direction = inputData.charAt(1);
            speedValue = inputData.substring(3, inputData.indexOf(',', 3)).toInt();
            turnDir = inputData.charAt(inputData.indexOf(',', 3) + 1);
            turnIntensity = inputData.substring(inputData.lastIndexOf(',') + 1).toInt();
        }
    }

    controlMotors();
}

