#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

char inputBuffer[50];
float DisFront, DisLeftHorizontal, DisRightHorizontal, 
      leftCount, rightCount, irLeftValue, irRightValue;
const float correctionFactor = 0.707; 
int currentHorizontalDistance = DisLeftHorizontal;
float referenceWallDistance = 0.0;
float currentWallDistance = 0.0;
int RotationalAngle = 0;
char referenceWallSide = 'N'; 
bool isCalibrated = false;
bool isRotating = false; 
int count = 0;

String wall_Detection = "XXXX";

float angleZ = 0.0;
float angleZ1 = 0.0;
float gyroZOffset = 0.0;
float DegreeAngle = 0;
float DegreeAngle1 = 0;
unsigned long prevTime = 0;

float Kp = 1.5, Ki = 0.01, Kd = 0.5;
float Kpg = 1, Kig = 0.01, Kdg = 0.2;
float integral = 0.0, gyroIntegral=0.0, previous_error = 0.0 , previous_gyro_error = 0.0;
float initialAngle = 0.11111;
float desired_angle = 0;  
float dt = 0.0089;  

String command = "<S,0,0,N,0>"; 
char direction = 'S';  
float rightSpeedValue = 120;
float leftSpeedValue = 120;
float RightSpeed = 120;
float LeftSpeed = 120;
float RightSpeed1 = 120;
float LeftSpeed1 = 120;
char turnDir = 'N';
char BotDir = 'F';
char BotDir1 = 'F';

void resetPID() {
  integral = 0;
  previous_error = 0;
}

void resetGyroPID() {
  gyroIntegral = 0;
  previous_gyro_error = 0;
}

float computePID(float setpoint, float current_value, float dt) {
  float error = (((setpoint - current_value)/1023)*150);
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previous_error = error;
  return output;
}

float computeGyroPID(float setpoint, float current_value, float dt) {
  float error = (((setpoint - current_value)/1023)*150);
  //gyroIntegral += error * dt;
  float derivative = (error - previous_error) / dt;
  float output = (Kpg * error) + (Kig * integral) + (Kdg * derivative);
  previous_gyro_error = error;
  return output;
}

void alignWithBackWall() {
  // 1. Send command to move backward slowly for alignment
  String alignCommand = "<B,150,150,N,0>";  // Backward with moderate speed
  Serial.println(alignCommand);  // Send to Arduino

  delay(2500);  // Move backward for 700 milliseconds (adjust based on testing)

  // 2. Stop the bot after moving back
  String stopCommand = "<S,0,0,N,0>";  // Stop command
  Serial.println(stopCommand);  // Send to Arduino

  // 3. Reset Gyroscope readings
  angleZ = 0.0;
  DegreeAngle = 0;
  integral = 0;  // Reset PID integral (optional but good for fresh start)
  previous_error = 0.0;
}

void readFromArduino() {
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer) - 1);
    inputBuffer[len] = '\0'; // Null-terminate properly
    // Strip newline, carriage returns if any (in case they are present)
    while (len > 0 && (inputBuffer[len - 1] == '\r' || inputBuffer[len - 1] == '\n')) {
      inputBuffer[--len] = '\0';
    }

    // Now parse
    if (inputBuffer[0] == '<' && inputBuffer[len - 1] == '>') {
      float tempDisFront, tempDisLeftHorizontal, tempDisRightHorizontal;
      float tempLeftCount, tempRightCount;
      float tempIrLeftValue, tempIrRightValue;

      int extracted = sscanf(inputBuffer, "<%f,%f,%f,%f,%f,%f,%f>",
                             &tempDisFront, &tempDisLeftHorizontal, &tempDisRightHorizontal,
                             &tempLeftCount, &tempRightCount,
                             &tempIrLeftValue, &tempIrRightValue);

      if (extracted == 7) {
        // Update all values except DisFront if isRotating is true
        if (!isRotating) {
          DisFront = tempDisFront;
        }
        DisLeftHorizontal = tempDisLeftHorizontal;
        DisRightHorizontal = tempDisRightHorizontal;
        leftCount = tempLeftCount;
        rightCount = tempRightCount;
        irLeftValue = tempIrLeftValue;
        irRightValue = tempIrRightValue;
      }
    }
  }
}

void sendStartCommandToArduino() {
  command = "<" + String(direction) + "," + String(leftSpeedValue) + "," + String(rightSpeedValue) + "," + String(DisRightHorizontal) + "," +String(DisLeftHorizontal)+">";
  Serial.println(command); // Send command to Arduino
}

void sendCommandToArduino() {
  command = "<" + String(direction) + "," + String(leftSpeedValue) + "," + String(rightSpeedValue) + 
  "," + String(isRotating) + "," + String(DisFront) + ">";
  Serial.println(command); 
}

void updateGyro() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroZ = g.gyro.z - gyroZOffset;
  angleZ += gyroZ * dt;
  angleZ1 += gyroZ * dt;
  // No reset, keep angle continuous
  if(angleZ1 > 6.19) {
    angleZ1 = 0;
  }else if(angleZ1 < -6.20){
    angleZ1 = 0;
  }

  DegreeAngle = (angleZ / 6.20) * 360;
  
  DegreeAngle1 = (angleZ1 / 6.20) * 360;

  RotationalAngle = DegreeAngle1;

  if(DegreeAngle1>179 ) {
      RotationalAngle = DegreeAngle1-360;
  }else if(DegreeAngle1<-179){
      RotationalAngle = DegreeAngle1+360;
  }
}

void botDirection(){
  if(RotationalAngle >= -45 && RotationalAngle < 44){
    BotDir = 'F';
  }else if(RotationalAngle >= 44 && RotationalAngle < 134){
    BotDir = 'L';
  }else if(RotationalAngle >= 134 && RotationalAngle < 179 || RotationalAngle < -134 && RotationalAngle >= -180){
    BotDir = 'B';
  }else if(RotationalAngle <= -45 && RotationalAngle >= -134){
    BotDir = 'R';
  }
}

void calibrateWallDistance() {

  if (count < 3) {
    if (irLeftValue < 750) {
      referenceWallDistance = irLeftValue;
      referenceWallSide = 'L';
    } else if (irRightValue < 750) {
      referenceWallDistance = DisRightHorizontal;
      referenceWallSide = 'R';
    } else if (irLeftValue < 750 && irRightValue < 750) { 
      referenceWallDistance = irLeftValue;
      referenceWallSide = 'L';
    } else {
      referenceWallDistance = 888;
      referenceWallSide = 'N';
    }
  }
}

void makeDecision() {
  if (DisFront > 16 && irLeftValue < 750 && irRightValue < 750) {
    wall_Detection = "XLRX";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;  
    currentWallDistance = irLeftValue;
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed + correction,0,255);
    rightSpeedValue = constrain(RightSpeed - correction,0,255);
  } 
  
  else if (DisFront > 16 && irLeftValue < 750 && irRightValue > 1000) {
    wall_Detection = "XLXX";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;  
    currentWallDistance = irLeftValue;
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed + correction,0,255);
    rightSpeedValue = constrain(RightSpeed - correction,0,255);
  } 
  
  else if (DisFront > 16 && irRightValue < 750 && irLeftValue > 1000) {
    wall_Detection = "XXRX";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120; 
    currentWallDistance = irRightValue;
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed - correction,0,255);
    rightSpeedValue = constrain(RightSpeed + correction,0,255);
  } 
  
  else if(DisFront > 16 && irLeftValue > 1000 && irRightValue > 1000){
    wall_Detection = "XXXX";
    float RotationalAngle1 = RotationalAngle;
    if(BotDir=='F'){
      desired_angle = 0;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='L'){
      desired_angle = 90;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='R'){
      desired_angle = -90;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='B'){
      desired_angle = -180;
      if(RotationalAngle > 0){
        RotationalAngle1 = RotationalAngle-360;
      }else if(RotationalAngle < 0){
        RotationalAngle1 = RotationalAngle;
      }
    }
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed= 120;
    float correction = computeGyroPID(desired_angle,RotationalAngle1, dt);
    leftSpeedValue = constrain(LeftSpeed - correction,0,255);
    rightSpeedValue = constrain(RightSpeed + correction,0,255);
  } 
  
  else if (DisFront < 16 && DisFront > 4 && irLeftValue < 750 && irRightValue < 750) {
    wall_Detection = "XLRN";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;
    currentWallDistance = irLeftValue;
    leftSpeedValue = LeftSpeed-(LeftSpeed-((DisFront/16)*LeftSpeed));
    rightSpeedValue = RightSpeed-(RightSpeed-((DisFront/16)*RightSpeed));
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed + correction,0,255);
    rightSpeedValue = constrain(RightSpeed - correction,0,255);
  } 
  
  else if (DisFront < 16 && DisFront > 4 && irLeftValue < 750 && irRightValue > 1000) {
    wall_Detection = "XLXN";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;
    currentWallDistance = irLeftValue;
    leftSpeedValue = LeftSpeed-(LeftSpeed-((DisFront/16)*LeftSpeed));
    rightSpeedValue = RightSpeed-(RightSpeed-((DisFront/16)*RightSpeed));
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed + correction,0,255);
    rightSpeedValue = constrain(RightSpeed - correction,0,255);
  } 
  
  else if (DisFront < 16 && DisFront > 4 && irRightValue < 750 && irLeftValue > 1000) {
    wall_Detection = "XXRN";
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;
    currentWallDistance = irRightValue;
    leftSpeedValue = LeftSpeed-(LeftSpeed-((DisFront/16)*LeftSpeed));
    rightSpeedValue = RightSpeed-(RightSpeed-((DisFront/16)*RightSpeed));
    float correction = computePID(referenceWallDistance,currentWallDistance , dt);
    leftSpeedValue = constrain(LeftSpeed - correction,0,255);
    rightSpeedValue = constrain(RightSpeed + correction,0,255);
  } 
  
  else if (DisFront < 16 && DisFront > 4 && irLeftValue > 1000 && irRightValue > 1000) {
    wall_Detection = "XXXN";
    float RotationalAngle1 = RotationalAngle;
    if(BotDir=='F'){
      desired_angle = 0;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='L'){
      desired_angle = 90;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='R'){
      desired_angle = -90;
      RotationalAngle1 = RotationalAngle;
    }else if(BotDir=='B'){
      desired_angle = -180;
      if(RotationalAngle > 0){
        RotationalAngle1 = RotationalAngle-360;
      }else if(RotationalAngle < 0){
        RotationalAngle1 = RotationalAngle;
      }
    }
    direction = 'F';
    LeftSpeed = 120;
    RightSpeed = 120;
    LeftSpeed1 = LeftSpeed-(LeftSpeed-((DisFront/16)*LeftSpeed));
    RightSpeed1 = RightSpeed-(RightSpeed-((DisFront/16)*RightSpeed));
    float correction = computeGyroPID(desired_angle,RotationalAngle1, dt);
    leftSpeedValue = constrain(LeftSpeed1 - correction,0,255);  
    rightSpeedValue = constrain(RightSpeed1 + correction,0,255);
  } 
  
  else if (DisFront < 4 && irLeftValue > 1000 && irRightValue > 1000) {
    wall_Detection = "FXXN";  
    direction = 'F';           
    if (initialAngle == 0.11111) {
        initialAngle = DegreeAngle; 
      }
    desired_angle = initialAngle - 90;
    LeftSpeed = 0;  
    RightSpeed = 0;  
    float correction = computeGyroPID(desired_angle, DegreeAngle, dt);
    leftSpeedValue = constrain(LeftSpeed - correction, 0, 255); 
    rightSpeedValue = constrain(RightSpeed + correction, 0, 255); 
    if (!isRotating) {
        isRotating = true; 
      }
    if (abs(DegreeAngle - desired_angle) < 5.0) {  
      isRotating = false;
      initialAngle = 0.11111; 
      }
    }    
  
  else if (DisFront < 4 && irLeftValue > 1000 && irRightValue < 750){
    wall_Detection = "FXRN";
    direction = 'F';
    if (initialAngle == 0.11111) {
      initialAngle = DegreeAngle; 
    }
    desired_angle = initialAngle + 90;
    LeftSpeed = 0;
    RightSpeed = 0;
    float correction = computeGyroPID(desired_angle, DegreeAngle, dt);
    leftSpeedValue = constrain(LeftSpeed - correction, 0, 255); 
    rightSpeedValue = constrain(RightSpeed + correction,0,255);
    if (!isRotating) {
      isRotating = true; 
    }

  if (abs(DegreeAngle - desired_angle) < 5.0) {  
    isRotating = false;
    initialAngle = 0.11111; 
    }
  } 
  
  else if (DisFront < 4 && irRightValue > 1000 && irLeftValue < 750){
    wall_Detection = "FLXN";
    if (initialAngle == 0.11111) {
      initialAngle = DegreeAngle; 
    }
    desired_angle = initialAngle - 90;
    direction = 'F';
    LeftSpeed = 0;
    RightSpeed = 0;
    float correction = computeGyroPID(desired_angle, DegreeAngle, dt);
    leftSpeedValue = constrain(LeftSpeed - correction,0,255);
    rightSpeedValue = constrain(RightSpeed + correction,0,255);
    if (!isRotating) {
      isRotating = true; 
    }

    if (abs(DegreeAngle - desired_angle) < 5.0) {  
      isRotating = false;
      initialAngle = 0.11111; 
    }
  } 
  
  else if (DisFront < 4 && irLeftValue < 750 && irRightValue < 750){
    wall_Detection = "FLRN";
    if (initialAngle == 0.11111) {
      initialAngle = DegreeAngle; 
    }

    desired_angle = initialAngle - 180;
    direction = 'R';
    LeftSpeed = 10; 
    RightSpeed = 10;
    float correction = computeGyroPID(desired_angle, DegreeAngle, dt);
    leftSpeedValue = constrain(LeftSpeed - correction,0,255);
    rightSpeedValue = constrain(RightSpeed - correction,0,255);
    if (!isRotating) {
      isRotating = true; 
    }

    if (abs(DegreeAngle - desired_angle) < 5.0) {  
      isRotating = false;
      initialAngle = 0.11111; 
    }
  }
}

void setup() {
  Serial.begin(57600); 
  delay(3000);
  Wire.begin(4, 5); 

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  Serial.println("Calibrating Gyro...");
  int numSamples = 500;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
    delay(5);
  }
  gyroZOffset /= numSamples;
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZOffset, 6);

  prevTime = millis();

  alignWithBackWall(); 
 
  delay(500);
}

void loop() {
  updateGyro();
  
  botDirection();

  readFromArduino();
 
  calibrateWallDistance();
  
  if (count > 3){
    
    makeDecision();

    sendCommandToArduino();
  
  }

  count++;
}


