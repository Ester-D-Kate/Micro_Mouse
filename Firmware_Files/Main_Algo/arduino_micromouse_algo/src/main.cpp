#include <Arduino.h>
#include <NewPing.h>

#define trigPinLeft 14
#define echoPinLeft 15
#define trigPinFront 16
#define echoPinFront 17
#define trigPinRight 18
#define echoPinRight 19

#define IR_LEFT A6 
#define IR_RIGHT A7

#define RIGHT_HALL 12  
#define LEFT_HALL 11  

#define LEFT_MOTOR_FWD 8
#define LEFT_MOTOR_BWD 9
#define RIGHT_MOTOR_FWD 6
#define RIGHT_MOTOR_BWD 7
#define PWM_LEFT 10
#define PWM_RIGHT 5

#define MAX_DISTANCE 200 // Max distance in cm

int rightCount = 0;  
int leftCount = 0; 
bool rightLastState = HIGH; 
bool leftLastState = HIGH;

int irLeftValue = 0;
int irRightValue = 0;

String command = "<0,0,0,0,0,0,0>";
String inputData = "";
int rightSpeedValue = 0; 
int leftSpeedValue = 0; 
char direction = 'S';
char turnDir = 'N';

NewPing SonarFront(trigPinFront, echoPinFront, MAX_DISTANCE);
NewPing SonarRight(trigPinRight, echoPinRight, MAX_DISTANCE);
NewPing SonarLeft(trigPinLeft, echoPinLeft, MAX_DISTANCE);

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
    Serial.begin(57600);
    
    
    pinMode(RIGHT_HALL, INPUT_PULLUP); 
    pinMode(LEFT_HALL, INPUT_PULLUP);
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BWD, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BWD, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);
}

void loop() {
    
    int DisFront = SonarFront.ping_cm();
    int DisRightDiagnal = SonarRight.ping_cm();
    int DisLeftDiagnal = SonarLeft.ping_cm();
    if (DisFront == 0) DisFront = MAX_DISTANCE;
    if (DisRightDiagnal == 0) DisRightDiagnal = MAX_DISTANCE;
    if (DisLeftDiagnal == 0) DisLeftDiagnal = MAX_DISTANCE;
    
    bool rightCurrentState = digitalRead(RIGHT_HALL);
    bool leftCurrentState = digitalRead(LEFT_HALL);
    if (rightLastState == LOW && rightCurrentState == HIGH) {
        rightCount++; 
    }  
    if (leftLastState == LOW && leftCurrentState == HIGH) {
        leftCount++; // Increment left count
    }
    rightLastState = rightCurrentState;
    leftLastState = leftCurrentState;

    irLeftValue = analogRead(IR_LEFT); 
    irRightValue = analogRead(IR_RIGHT);
    
    command = "<" + String(DisFront) + "," + String(DisLeftDiagnal) + "," + String(DisRightDiagnal) + "," + 
          String(leftCount) + "," + String(rightCount) + "," + 
          String(irLeftValue) + "," + String(irRightValue) + ">";
    Serial.println(command);
    
    
    char inputBuffer[30]; // enough length for command

    if (Serial.available()) {
        int len = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer));
        inputBuffer[len] = '\0'; // null-terminate the string

        if (len > 5) {
            direction = inputBuffer[1];
            sscanf(inputBuffer, "<%*c,%d,%d,%c", &leftSpeedValue, &rightSpeedValue, &turnDir);
        }
    }

    controlMotors();
}