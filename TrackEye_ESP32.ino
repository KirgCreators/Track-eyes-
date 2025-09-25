#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>

const char* ssid = "Sailormoon";         // WiFi SSID
const char* password = "TrackEye01";     // WiFi Password

const char* serverName = "http://172.20.10.4/sensor-tracker/save_data.php";
const char* controlUrl = "http://172.20.10.4/sensor-tracker/control.php";

const int hallPin = 34;      // Analog input sensor
const int ppsPin = 26;       // PPS signal
const int ledPin = 2;        // Indicator LED
const float setpoint = 1.8;  // Voltage threshold

HardwareSerial GPS(2);
TinyGPSPlus gps;

volatile bool ppsFlag = false;
bool loggingEnabled = false;

float latitude = 0.0;
float longitude = 0.0;
float voltage = 0.0;
String status = "OK";

// PPS interrupt
void IRAM_ATTR onPPS() {
  ppsFlag = true;
}

// Check log control status from server
void checkLoggingStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(controlUrl);
    int code = http.GET();
    if (code == 200) {
      String cmd = http.getString();
      cmd.trim();
      loggingEnabled = (cmd == "start");
      Serial.println("[CONTROL] Logging: " + cmd);
    } else {
      Serial.println("[ERROR] Control status HTTP error: " + String(code));
    }
    http.end();
  } else {
    Serial.println("[WARNING] WiFi not connected for control check.");
  }
}

// WiFi connection function
void connectToWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Failed to connect.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(hallPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(ppsPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppsPin), onPPS, RISING);

  GPS.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  connectToWiFi();
}

void loop() {
  // GPS Data Parsing
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  // Periodic check every 5 seconds for control status
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    checkLoggingStatus();
    lastCheck = millis();
  }

  // PPS-triggered data logging
  if (ppsFlag) {
    ppsFlag = false;

    // Use last valid GPS location if current is invalid
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }

    // Read analog voltage
    int raw = analogRead(hallPin);
    voltage = raw * (3.3 / 4095.0);
    status = (voltage < setpoint) ? "DEFECT" : "OK";
    digitalWrite(ledPin, (status == "DEFECT") ? HIGH : LOW);

    Serial.printf("[DATA] V: %.2fV | %s | Lat: %.6f | Lon: %.6f\n",
                  voltage, status.c_str(), latitude, longitude);

    // Send data if logging enabled
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi(); // auto reconnect
    }

    if (WiFi.status() == WL_CONNECTED && loggingEnabled) {
      HTTPClient http;
      http.begin(serverName);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      // Get current timestamp
      struct tm timeinfo;
      if(!getLocalTime(&timeinfo)) {
        // fallback to millis if NTP not configured
        sprintf((char*) &timeinfo, "1970-01-01 00:00:00");
      }
      char timestamp[20];
      strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);

      String postData = "timestamp=" + String(timestamp) +
                        "&latitude=" + String(latitude, 6) +
                        "&longitude=" + String(longitude, 6) +
                        "&voltage=" + String(voltage, 2) +
                        "&status=" + status;

      int httpCode = http.POST(postData);
      String response = http.getString();

      Serial.println("[HTTP] Code: " + String(httpCode));
      Serial.println("[HTTP] Response: " + response);
      http.end();
    }
  }
}
