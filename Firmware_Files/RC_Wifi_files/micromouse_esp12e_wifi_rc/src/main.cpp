#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "Airtel_arun_7500";
const char* password = "7087523885s";

Adafruit_MPU6050 mpu;
AsyncWebServer server(80);
WiFiClient client;

String command = "<S,0,N,0>"; // Default command (Stopped)

// Stores speed and turn intensity
String dir = "S";
String speed = "0";
String turn = "N";


bool isFDesiredAngleSet = false;
bool isBDesiredAngleSet = false;
bool isLDesiredAngleSet = false;
bool isRDesiredAngleSet = false;
bool isSDesiredAngleSet = false;

int DegreeAngle = 0;
int RightSpeedValue = 50;
int LeftSpeedValue = 50;
int speedValue = 50;
int turnIntensity = 0;
float angleZ = 0.0;  // Total rotated angle
float gyroZOffset = 0.0;  // Will be calculated
unsigned long prevTime = 0;  // Previous time

float Kp = 1.5;  // Proportional Gain
float Ki = 0.01; // Integral Gain
float Kd = 0.5;  // Derivative Gain

float desired_angle = 0;  // Setpoint (desired angle)
float integral = 0.0;
float previous_error = 0.0;

// Webpage (HTML + JS)
const char webpage[] PROGMEM = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <title>ESP Bot Control</title>
        <style>
            body { font-family: Arial; text-align: center; }
            .btn { padding: 15px; margin: 10px; font-size: 20px; }
            .slider-container { margin: 10px; }
            .slider { width: 80%; }
        </style>
    </head>
    <body>
        <h2>ESP Remote Control</h2>
        
        <div class="slider-container">
            <label for="speedSlider">Speed:</label>
            <input type="range" min="0" max="100" value="50" class="slider" id="speedSlider">
            <span id="speedValue">50</span>%
        </div>
    
            <button class="btn" onpointerdown="sendCommand('F')" onpointerup="sendCommand('S')">  F  </button><br>
            <button class="btn" onpointerdown="sendCommand('L')" onpointerup="sendCommand('S')">  L  </button>
            <button class="btn" onpointerdown="sendCommand('S')" onpointerup="sendCommand('S')">  S  </button>
            <button class="btn" onpointerdown="sendCommand('R')" onpointerup="sendCommand('S')">  R  </button><br>
            <button class="btn" onpointerdown="sendCommand('B')" onpointerup="sendCommand('S')">  B  </button>

        <h3>PID Tuning</h3>
        <div class="slider-container">
            <label for="kpSlider">Kp:</label>
            <input type="range" min="0" max="10" step="0.1" value="1.5" class="slider" id="kpSlider">
            <span id="kpValue">1.5</span>
        </div>
    
        <div class="slider-container">
            <label for="kiSlider">Ki:</label>
            <input type="range" min="0" max="1" step="0.01" value="0.01" class="slider" id="kiSlider">
            <span id="kiValue">0.01</span>
        </div>
    
        <div class="slider-container">
            <label for="kdSlider">Kd:</label>
            <input type="range" min="0" max="5" step="0.1" value="0.5" class="slider" id="kdSlider">
            <span id="kdValue">0.5</span>
        </div>
    
        <button class="btn" onclick="updatePID()">Update PID</button>
    
        <script>
            let turnIntensity = 0;
    
            function sendCommand(direction) {
                let speed = document.getElementById('speedSlider').value;
                if (direction === 'L' || direction === 'R') {
                    turnIntensity += 10;
                    if (turnIntensity > 70) turnIntensity = 70;
                } else {
                    turnIntensity = 0;
                }
                fetch(`/move?dir=${direction}&speed=${speed}&turn=${turnIntensity}`);
            }
    
            function updatePID() {
                let kp = document.getElementById('kpSlider').value;
                let ki = document.getElementById('kiSlider').value;
                let kd = document.getElementById('kdSlider').value;
    
                fetch(`/updatePID?kp=${kp}&ki=${ki}&kd=${kd}`);
            }
    
            document.getElementById('speedSlider').oninput = function() {
                document.getElementById('speedValue').innerText = this.value;
            };
    
            document.getElementById('kpSlider').oninput = function() {
                document.getElementById('kpValue').innerText = this.value;
            };
    
            document.getElementById('kiSlider').oninput = function() {
                document.getElementById('kiValue').innerText = this.value;
            };
    
            document.getElementById('kdSlider').oninput = function() {
                document.getElementById('kdValue').innerText = this.value;
            };
        </script>
    </body>
    </html>
    )rawliteral";
    

void setup() {
  Serial.begin(115200); // Start Serial Monitor
  while (!Serial);  // Wait for Serial Monitor to open
  WiFi.begin(ssid, password);
  Serial.println(" ");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) { 
      delay(1000); 
      Serial.print(".");
  }
  
  Serial.println("\nConnected to WiFi!");
  Serial.print("ESP IP Address: ");
  Serial.println(WiFi.localIP());  // Print IP address to Serial Monitor

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", webpage);
  });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request){
      String dir = request->getParam("dir")->value();
      String speed = request->getParam("speed")->value();
      String turn = request->getParam("turn")->value();

      command = "<" + dir + "," + speed + "," + (dir == "L" || dir == "R" ? dir : "N") + "," + speed + ">";
      Serial.println(command);
      request->send(200, "text/plain", "OK");
  });
  server.on("/updatePID", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();

    Serial.print("Updated PID: Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.println(Kd);

    request->send(200, "text/plain", "PID Updated");
});

  server.begin();

  Wire.begin(4, 5);  // SDA = GPIO4 (D2), SCL = GPIO5 (D1)

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
      Serial.println("MPU6050 not detected!");
      while (1);
  }

  Serial.println("MPU6050 Connected!");
  
  //  Calculate Gyro Drift Offset
  Serial.println("Calculating Gyro Drift...");
  int numSamples = 500;
  for (int i = 0; i < numSamples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gyroZOffset += g.gyro.z;
      delay(5);  // Small delay for stability
  }
  gyroZOffset /= numSamples;  // Average drift calculation
  Serial.print("Gyro Z Offset (Drift Correction): ");
  Serial.println(gyroZOffset, 6);

  prevTime = millis();  // Initialize time
}




void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float gyroZ = g.gyro.z - gyroZOffset;  
    angleZ += gyroZ * dt;  // Integration of gyro value to get angle
    
    if(angleZ > 6.20) {
        angleZ = 0;
    }else if(angleZ < -6.20){
        angleZ = 0;
    } 

    DegreeAngle=(angleZ/6.20)*360;

    int RotationalAngle = DegreeAngle;
    
    if(DegreeAngle>179 ) {
        RotationalAngle = DegreeAngle-360;
    }else if(DegreeAngle<-179){
        RotationalAngle = DegreeAngle+360;
    }
    
    // Calculate PID terms
    float error = desired_angle - RotationalAngle;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    
    // PID output for controlling car
    float PidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    //int outputPWM = map(PidOutput, -maxVal, maxVal, 0, 255);
    //outputPWM = constrain(outputPWM, 0, 255);

    previous_error = error;
    if (command[1] == 'F' && !isFDesiredAngleSet) {
        desired_angle = RotationalAngle;  // Store the current angle as the reference
        isFDesiredAngleSet = true;  // Prevent updating until another command appears
        
        Serial.print("Desired Angle Locked F: "); Serial.println(desired_angle);
    }
    if (command[1] == 'B' && !isBDesiredAngleSet) {
        desired_angle = RotationalAngle;  // Store the current angle as the reference
        isBDesiredAngleSet = true;  // Prevent updating until another command appears
        Serial.print("Desired Angle Locked B: "); Serial.println(desired_angle);
    }
    if (command[1] == 'L' && !isLDesiredAngleSet) {
        desired_angle = RotationalAngle;  // Store the current angle as the reference
        isLDesiredAngleSet = true;  // Prevent updating until another command appears
        Serial.print("Desired Angle Locked L: "); Serial.println(desired_angle);
    }
    if (command[1] == 'R' && !isRDesiredAngleSet) {
        desired_angle = RotationalAngle;  // Store the current angle as the reference
        isRDesiredAngleSet = true;  // Prevent updating until another command appears
        Serial.print("Desired Angle Locked R: "); Serial.println(desired_angle);
    }
    if (command[1] == 'S' && !isSDesiredAngleSet) {
        desired_angle = RotationalAngle;  // Store the current angle as the reference
        isSDesiredAngleSet = true;  // Prevent updating until another command appears
        Serial.print("Desired Angle Locked S: "); Serial.println(desired_angle);
    }
    Serial.print("AngleZ: "); Serial.print(angleZ); Serial.print("  ");
    Serial.print("Angle: "); Serial.print(RotationalAngle); Serial.print("  ");
    Serial.print("Error: "); Serial.println(error);
    // Reset the flag when a new direction (B, L, S, R) is received
    if (command[1] != 'F') {
        isFDesiredAngleSet = false;
    }
    if (command[1] != 'B') {
        isBDesiredAngleSet = false;
    }
    if (command[1] != 'L') {
        isLDesiredAngleSet = false;
    }
    if (command[1] != 'R') {
        isRDesiredAngleSet = false;
    }
    if (command[1] != 'S') {
        isSDesiredAngleSet = false;
    }

    delay(50);
}