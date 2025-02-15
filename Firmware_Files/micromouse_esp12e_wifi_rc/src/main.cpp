#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "Airtel_arun_7500";
const char* password = "7087523885s";

AsyncWebServer server(80);
WiFiClient client;

String command = "<S,0,N,0>"; // Default command (Stopped)

// Stores speed and turn intensity
int speedValue = 50;
int turnIntensity = 0;

// Webpage (HTML + JS)
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP Bot Control</title>
    <style>
        body { font-family: Arial; text-align: center; }
        .btn { padding: 15px; margin: 10px; font-size: 20px; }
        .speed-slider { width: 80%; }
    </style>
</head>
<body>
    <h2>ESP Remote Control</h2>
    <input type="range" min="0" max="100" value="50" class="speed-slider" id="speedSlider">
    <p>Speed: <span id="speedValue">50</span>%</p>

    <button class="btn" onclick="sendCommand('F')">Forward</button><br>
    <button class="btn" onclick="sendCommand('L')">Left</button>
    <button class="btn" onclick="sendCommand('S')">Stop</button>
    <button class="btn" onclick="sendCommand('R')">Right</button><br>
    <button class="btn" onclick="sendCommand('B')">Backward</button>

    <script>
        let turnIntensity = 0;
        function sendCommand(direction) {
            let speed = document.getElementById('speedSlider').value;
            if (direction === 'L' || direction === 'R') {
                turnIntensity += 10; // Increase turn reduction
                if (turnIntensity > 70) turnIntensity = 70;
            } else {
                turnIntensity = 0; // Reset on new direction
            }
            fetch(`/move?dir=${direction}&speed=${speed}&turn=${turnIntensity}`);
        }

        document.getElementById('speedSlider').oninput = function() {
            document.getElementById('speedValue').innerText = this.value;
        };
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200); // Start Serial Monitor
  WiFi.begin(ssid, password);
  
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

      command = "<" + dir + "," + speed + "," + (dir == "L" || dir == "R" ? dir : "N") + "," + turn + ">";
      Serial.println(command);
      request->send(200, "text/plain", "OK");
  });

  server.begin();
}

void loop() {
    // Nothing needed here, handled via server events
}
