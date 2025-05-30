/*******************************************************
    SMART GARDEN + NPK MODBUS WITH WEB SERVER (ESP32)
    - DHT22, soil moisture, N-P-K via RS485 Modbus
    - Web Interface with CSS & Background Image
    - OTA Updates, Google Sheets Integration
    - Plant Health Indicator
    - Fixed for ESP32 compatibility
*******************************************************/

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // Use ESP32 compatible version
#include <DHT.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <ModbusMaster.h>

// LCD Configuration (ESP32 compatible)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16 columns, 2 rows

// Button Pins
const int BTN_MENU  = 4;
const int BTN_UP    = 12;
const int BTN_DOWN  = 32;
const int BTN_ENTER = 33;

// DHT22 Configuration
#define DHTPIN 13
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Soil Moisture
const int SOIL_MOISTURE_PIN = 34;
#define SOIL_SAMPLES 10
uint16_t soilBuffer[SOIL_SAMPLES] = {0};
int soilIndex = 0;

// Relays & LEDs
const int RELAY_PUMP = 26;
const int RELAY_FAN  = 25;
const int LED_PUMP   = 27;
const int LED_FAN    = 14;

// RS485/Modbus for NPK
#define RS485_DE_RE_PIN 16
#define RS485_RX_PIN    15
#define RS485_TX_PIN    17
const uint8_t SENSOR_ID  = 1;
const uint32_t BAUD_RATE = 4800;
ModbusMaster node;

// EEPROM Addresses
#define SOIL_THRESHOLD_ADDR    0
#define TEMP_THRESHOLD_ADDR    1
#define HYST_SOIL_LOWER_ADDR   2
#define HYST_SOIL_UPPER_ADDR   3
#define HYST_TEMP_LOWER_ADDR   6
#define HYST_TEMP_UPPER_ADDR   7
#define CAL_SOIL_DRY_ADDR      8
#define CAL_SOIL_WET_ADDR     10
#define EEPROM_SIZE           12

// Web Server
WebServer server(80);

// Menu Items
const char* menuItems[] = {
  "Set Soil", "Set Temp", "Hyst Soil", "Hyst Temp",
  "Set WiFi", "Auto Mode", "Cal Soil",
  "Backup Data", "Restore Data", "Shutdown", "Exit"
};
const int totalMenus = sizeof(menuItems) / sizeof(menuItems[0]);

// Global Variables
bool pumpStatus = false;
bool fanStatus  = false;
bool autoMode = true, manualMode = false, inMenu = false, updateDisplay = true;
bool wifiConnected = false, systemOn = true;
unsigned long lastMenuPress = 0, lastUpdate = 0;

// Threshold & Hysteresis
int soilThreshold = 35, tempThreshold = 30;
int hystSoilLower = 3, hystSoilUpper = 3;
int hystTempLower = 2, hystTempUpper = 2;

// Soil Calibration
uint16_t calSoilDry = 4095, calSoilWet = 0;

// Manual Durations
int pumpManualTime = 0, fanManualTime = 0;
unsigned long pumpEndMillis = 0, fanEndMillis = 0;

// Sensor Values
int lastTemp = -1, lastHum = -1, lastMoist = -1;
uint16_t lastN = 0, lastP = 0, lastK = 0;
String plantHealthStatus = "good"; // Default health status

// Google Sheets URL
const char* googleScriptURL = "https://script.google.com/macros/s/YOUR_SCRIPT_ID/exec";

// Function Prototypes
void preTransmission();
void postTransmission();
bool isPressed(int pin);
uint16_t readSoilAverage();
int computeMoisture(uint16_t raw);
void sendToGoogleSheet(float t, float h, int m);
void readNPK();
void evaluatePlantHealth();
void displayHomeScreen();
void checkWiFiConnection();
void checkSensorStatus();
void calibrateSoilSensor();
void backupCalibrationData();
void restoreCalibrationData();
void inputNumber(int *v, int mn, int mx);
void setThreshold(String label, int *val, int addr);
void setHysteresis(String type, int addrLow, int addrHigh);
void setWiFiManagerPortal();
void confirmAutoMode();
void confirmCalSoil();
void setManualPumpTime();
void setManualFanTime();
void startManualMode();
void setPump(bool s);
void setFan(bool s);
void navigateMenu();
void executeMenu(int menuIndex);
void updateMenuDisplay(int menuIndex);
void shutdownSystem();
void startupSystem();
void autoIrrigationControl();
void runManualMode();

// Web Server Functions
void handleRoot();
void handleCSS();
void handleBackgroundImage();
void handleSensorData();
void handleSetMode();
void handleSetPump();
void handleSetFan();
void handleSetPumpTime();
void handleSetFanTime();
void handleNotFound();
void initWebServer();

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  dht.begin();
  startupSystem();

  // Read EEPROM values
  int v;
  if ((v = EEPROM.read(SOIL_THRESHOLD_ADDR)) != 255) soilThreshold = v;
  if ((v = EEPROM.read(TEMP_THRESHOLD_ADDR)) != 255) tempThreshold = v;
  if ((v = EEPROM.read(HYST_SOIL_LOWER_ADDR)) != 255) hystSoilLower = v;
  if ((v = EEPROM.read(HYST_SOIL_UPPER_ADDR)) != 255) hystSoilUpper = v;
  if ((v = EEPROM.read(HYST_TEMP_LOWER_ADDR)) != 255) hystTempLower = v;
  if ((v = EEPROM.read(HYST_TEMP_UPPER_ADDR)) != 255) hystTempUpper = v;
  calSoilDry = EEPROM.read(CAL_SOIL_DRY_ADDR) | (EEPROM.read(CAL_SOIL_DRY_ADDR + 1) << 8);
  calSoilWet = EEPROM.read(CAL_SOIL_WET_ADDR) | (EEPROM.read(CAL_SOIL_WET_ADDR + 1) << 8);

  // Initialize LCD and buttons
  Wire.begin(21, 22);  // SDA, SCL pins for ESP32
  lcd.init();
  lcd.backlight();
  
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);

  // Initialize relays and LEDs
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  pinMode(LED_FAN, OUTPUT);
  digitalWrite(RELAY_PUMP, LOW);
  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(LED_PUMP, LOW);
  digitalWrite(LED_FAN, LOW);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    while (1) delay(10);
  }

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  bool res = wm.autoConnect("SmartGardenAP", "12345678");
  if (!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  } else {
    wifiConnected = true;
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // Initialize Web Server
  initWebServer();

  // Initialize OTA
  ArduinoOTA.setHostname("SmartGarden");
  ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();

  // Initialize Modbus
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(SENSOR_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  displayHomeScreen();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  
  if (systemOn) {
    checkWiFiConnection();
    checkSensorStatus();

    static unsigned long t0 = 0;
    if (millis() - t0 >= 5000) {
      t0 = millis();
      float T = dht.readTemperature();
      float H = dht.readHumidity();
      int M = computeMoisture(readSoilAverage());
      
      if (!isnan(T) && !isnan(H) && M >= 0) {
        lastTemp = T;
        lastHum = H;
        lastMoist = M;
        sendToGoogleSheet(T, H, M);
        readNPK();
        evaluatePlantHealth();
      }
    }

    if (!inMenu && updateDisplay && millis() - lastUpdate >= 500) {
      lastUpdate = millis();
      displayHomeScreen();
    }

    if (isPressed(BTN_MENU)) {
      inMenu = true;
      updateDisplay = false;
      lcd.clear();
      lastMenuPress = millis();
      navigateMenu();
    }
    
    if (inMenu) {
      navigateMenu();
    } else if (manualMode) {
      runManualMode();
    } else {
      autoIrrigationControl();
    }
  } else {
    // Shutdown wake logic
    if (digitalRead(BTN_MENU) == LOW && digitalRead(BTN_ENTER) == LOW) {
      unsigned long s = millis();
      while (digitalRead(BTN_MENU) == LOW && digitalRead(BTN_ENTER) == LOW) {
        if (millis() - s >= 3000) {
          startupSystem();
          systemOn = true;
          return;
        }
      }
    }
    delay(100);
  }
}

// RS485 Functions
void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// Button Functions
bool isPressed(int pin) {
  if (digitalRead(pin) == LOW) {  // Button aktif LOW karena PULLUP
    delay(50);  // Debounce
    if (digitalRead(pin) == LOW) {
      return true;
    }
  }
  return false;
}

// Sensor Functions
uint16_t readSoilAverage() {
  soilBuffer[soilIndex] = analogRead(SOIL_MOISTURE_PIN);
  soilIndex = (soilIndex + 1) % SOIL_SAMPLES;
  uint32_t sum = 0;
  for (int i = 0; i < SOIL_SAMPLES; i++) sum += soilBuffer[i];
  return sum / SOIL_SAMPLES;
}

int computeMoisture(uint16_t raw) {
  if (raw <= 5 || raw >= 4090 || calSoilDry == calSoilWet) return -1;
  float f = float(calSoilDry - raw) / float(calSoilDry - calSoilWet);
  return constrain(int(f * 100), 0, 100);
}

void sendToGoogleSheet(float temp, float humid, int moist) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(googleScriptURL);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"temperature\":" + String(temp) +
                   ",\"humidity\":" + String(humid) +
                   ",\"soil_moisture\":" + String(moist) + 
                   ",\"plant_health\":\"" + plantHealthStatus + "\"}";
    int httpCode = http.POST(payload);
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
}

void readNPK() {
  uint8_t result = node.readHoldingRegisters(0x001E, 3);
  if (result == node.ku8MBSuccess) {
    lastN = node.getResponseBuffer(0);
    lastP = node.getResponseBuffer(1);
    lastK = node.getResponseBuffer(2);
  } else {
    Serial.printf("Modbus error: %d\n", result);
  }
}

void evaluatePlantHealth() {
  // Check for sensor errors first
  if (isnan(lastTemp) || isnan(lastHum) || lastMoist < 0 || lastN == 0 || lastP == 0 || lastK == 0) {
    plantHealthStatus = "unknown";
    return;
  }

  // Score each parameter (0-100)
  int tempScore = constrain(map(lastTemp, 10, 40, 0, 100), 0, 100);
  int moistScore = lastMoist;
  int nScore = constrain(map(lastN, 0, 200, 0, 100), 0, 100);
  int pScore = constrain(map(lastP, 0, 150, 0, 100), 0, 100);
  int kScore = constrain(map(lastK, 0, 300, 0, 100), 0, 100);

  // Calculate overall health score (weighted average)
  float overallScore = (tempScore * 0.2) + (moistScore * 0.3) + 
                      (nScore * 0.15) + (pScore * 0.15) + (kScore * 0.2);

  // Determine health status
  if (overallScore >= 80) {
    plantHealthStatus = "excellent";
  } else if (overallScore >= 60) {
    plantHealthStatus = "good";
  } else if (overallScore >= 40) {
    plantHealthStatus = "warning";
  } else {
    plantHealthStatus = "bad";
  }
}

// Display Functions
void displayHomeScreen() {
  static uint8_t state = 0, lastState = 255;
  static unsigned long tSlide = 0, tValue = 0;
  unsigned long now = millis();

  if (now - tSlide >= 5000) {
    tSlide = now;
    state = (state + 1) % 2;
    lastState = 255;
  }

  if (state != lastState) {
    lastState = state;
    lcd.clear();
    if (state == 0) {
      lcd.setCursor(0, 0);
      lcd.print(wifiConnected ? "---CONNECTED---" : "--SMART GARDEN--");
      lcd.setCursor(0, 1);
      lcd.print("T:  C H:  % S:  ");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("    NPK(ppm)    ");
      lcd.setCursor(0, 1);
      lcd.print("N:   P:    K:   ");
    }
  }

  if (state == 0 && now - tValue >= 1000) {
    tValue = now;
    // Temperature
    char bufT[3];
    if (isnan(lastTemp)) strcpy(bufT, "--");
    else sprintf(bufT, "%2d", lastTemp);
    lcd.setCursor(2, 1);
    lcd.print(bufT);

    // Humidity
    char bufH[3];
    if (isnan(lastHum)) strcpy(bufH, "--");
    else sprintf(bufH, "%2d", lastHum);
    lcd.setCursor(8, 1);
    lcd.print(bufH);

    // Soil Moisture
    char bufS[3];
    if (lastMoist < 0) strcpy(bufS, "--");
    else sprintf(bufS, "%2d", lastMoist);
    lcd.setCursor(14, 1);
    lcd.print(bufS);
  }

  if (state == 1) {
    lcd.setCursor(2, 1); lcd.print(lastN);
    lcd.setCursor(7, 1); lcd.print(lastP);
    lcd.setCursor(13, 1); lcd.print(lastK);
  }
}

// Web Server Handlers
void handleRoot() {
  File file = SPIFFS.open("/Olivia.html", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to load HTML file");
    return;
  }
  
  server.streamFile(file, "text/html");
  file.close();
}

void handleCSS() {
  File file = SPIFFS.open("/style.css", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to load CSS file");
    return;
  }
  
  server.sendHeader("Cache-Control", "max-age=604800");
  server.streamFile(file, "text/css");
  file.close();
}

void handleBackgroundImage() {
  File file = SPIFFS.open("/latar.png", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to load background image");
    return;
  }
  
  server.sendHeader("Cache-Control", "max-age=604800");
  server.streamFile(file, "image/png");
  file.close();
}

void handleSensorData() {
  String json = "{";
  json += "\"temperature\":" + String(lastTemp) + ",";
  json += "\"humidity\":" + String(lastHum) + ",";
  json += "\"moisture\":" + String(lastMoist) + ",";
  json += "\"nitrogen\":" + String(lastN) + ",";
  json += "\"phosphorus\":" + String(lastP) + ",";
  json += "\"potassium\":" + String(lastK) + ",";
  json += "\"plantHealth\":\"" + plantHealthStatus + "\",";
  json += "\"pumpStatus\":" + String(pumpStatus ? "true" : "false") + ",";
  json += "\"fanStatus\":" + String(fanStatus ? "true" : "false") + ",";
  json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"tempHumidError\":" + String(isnan(dht.readTemperature()) || isnan(dht.readHumidity()) ? "true" : "false") + ",";
  json += "\"soilError\":" + String((analogRead(SOIL_MOISTURE_PIN) <= 5 || analogRead(SOIL_MOISTURE_PIN) >= 4090) ? "true" : "false") + ",";
  json += "\"npkError\":false";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    autoMode = server.arg("mode") == "auto";
    manualMode = !autoMode;
    
    if (autoMode) {
      setPump(false);
      setFan(false);
    }
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetPump() {
  if (server.hasArg("state")) {
    bool status = server.arg("state") == "on";
    setPump(status);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetFan() {
  if (server.hasArg("state")) {
    bool status = server.arg("state") == "on";
    setFan(status);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetPumpTime() {
  if (server.hasArg("time")) {
    pumpManualTime = server.arg("time").toInt();
    if (!autoMode && pumpManualTime > 0) {
      pumpEndMillis = millis() + pumpManualTime * 60000UL;
      setPump(true);
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetFanTime() {
  if (server.hasArg("time")) {
    fanManualTime = server.arg("time").toInt();
    if (!autoMode && fanManualTime > 0) {
      fanEndMillis = millis() + fanManualTime * 60000UL;
      setFan(true);
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void initWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/style.css", HTTP_GET, handleCSS);
  server.on("/latar.png", HTTP_GET, handleBackgroundImage);
  server.on("/api/sensor", HTTP_GET, handleSensorData);
  server.on("/api/setmode", HTTP_POST, handleSetMode);
  server.on("/api/setpump", HTTP_POST, handleSetPump);
  server.on("/api/setfan", HTTP_POST, handleSetFan);
  server.on("/api/setpumptime", HTTP_POST, handleSetPumpTime);
  server.on("/api/setfantime", HTTP_POST, handleSetFanTime);
  server.onNotFound(handleNotFound);
  server.begin();
}

// System Functions
void shutdownSystem() {
  digitalWrite(RELAY_PUMP, LOW);
  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(LED_PUMP, LOW);
  digitalWrite(LED_FAN, LOW);
  lcd.clear();
  lcd.print("SYSTEM  SHUTDOWN");
  delay(1000);
  lcd.noBacklight();
  server.stop();
  WiFi.disconnect();
  systemOn = false;
}

void startupSystem() {
  lcd.backlight();
  lcd.clear();
  lcd.print("SYSTEM STARTING");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
    delay(100);
  }
  
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  initWebServer();
  displayHomeScreen();
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 5000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      WiFi.reconnect();
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
        delay(100);
      }
      wifiConnected = (WiFi.status() == WL_CONNECTED);
    } else {
      wifiConnected = true;
    }
    updateDisplay = true;
  }
}

void checkSensorStatus() {
  if (isnan(dht.readTemperature()) || isnan(dht.readHumidity())) {
    // Sensor error handling
    Serial.println("DHT sensor error");
  }
  uint16_t soilValue = readSoilAverage();
  if (soilValue <= 5 || soilValue >= 4090) {
    // Sensor error handling
    Serial.println("Soil sensor error");
  }
}

// Threshold Setting Functions
void setThreshold(String label, int *val, int addr) {
  int v = *val;
  lcd.clear();
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.print(v);
  delay(2000);
  
  lcd.clear();
  lcd.print("Set ");
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print(">");
  lcd.print(v);
  
  while (!isPressed(BTN_MENU)) {
    if (isPressed(BTN_UP)) {
      v++;
      if (v > 100) v = 0;
      lcd.setCursor(1, 1);
      lcd.print("   ");
      lcd.setCursor(1, 1);
      lcd.print(v);
      delay(200);
    }
    if (isPressed(BTN_DOWN)) {
      v--;
      if (v < 0) v = 100;
      lcd.setCursor(1, 1);
      lcd.print("   ");
      lcd.setCursor(1, 1);
      lcd.print(v);
      delay(200);
    }
    delay(50);
  }
  
  *val = v;
  EEPROM.write(addr, v);
  EEPROM.commit();
  lcd.clear();
  lcd.print("Saved!");
  delay(1000);
}

void setHysteresis(String type, int addrLow, int addrHigh) {
  int *lower, *upper;
  String label;
  
  if (type == "Soil") {
    lower = &hystSoilLower;
    upper = &hystSoilUpper;
    label = "Soil Hyst";
  } else {
    lower = &hystTempLower;
    upper = &hystTempUpper;
    label = "Temp Hyst";
  }
  
  lcd.clear();
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(*lower);
  lcd.print(" H:");
  lcd.print(*upper);
  delay(2000);
  
  bool settingLower = true;
  int v = *lower;
  
  lcd.clear();
  lcd.print("Set ");
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print(settingLower ? ">L:" : " L:");
  lcd.print(*lower);
  lcd.print(" H:");
  lcd.print(*upper);
  
  while (!isPressed(BTN_MENU)) {
    if (isPressed(BTN_ENTER)) {
      settingLower = !settingLower;
      v = settingLower ? *lower : *upper;
      lcd.setCursor(0, 1);
      lcd.print(settingLower ? ">L:" : " L:");
      lcd.print(*lower);
      lcd.print(" H:");
      lcd.print(*upper);
      delay(200);
    }
    
    if (isPressed(BTN_UP)) {
      v++;
      if (v > 10) v = 0;
      if (settingLower) *lower = v;
      else *upper = v;
      lcd.setCursor(0, 1);
      lcd.print(settingLower ? ">L:" : " L:");
      lcd.print(*lower);
      lcd.print(" H:");
      lcd.print(*upper);
      delay(200);
    }
    
    if (isPressed(BTN_DOWN)) {
      v--;
      if (v < 0) v = 10;
      if (settingLower) *lower = v;
      else *upper = v;
      lcd.setCursor(0, 1);
      lcd.print(settingLower ? ">L:" : " L:");
      lcd.print(*lower);
      lcd.print(" H:");
      lcd.print(*upper);
      delay(200);
    }
    
    delay(50);
  }
  
  EEPROM.write(addrLow, *lower);
  EEPROM.write(addrHigh, *upper);
  EEPROM.commit();
  lcd.clear();
  lcd.print("Saved!");
  delay(1000);
}

// Other Menu Functions
void confirmAutoMode() {
  lcd.clear();
  lcd.print("Set Auto Mode?");
  lcd.setCursor(0, 1);
  lcd.print("ENTER:Yes MENU:No");
  
  while (true) {
    if (isPressed(BTN_ENTER)) {
      autoMode = true;
      manualMode = false;
      lcd.clear();
      lcd.print("Auto Mode ON");
      delay(1000);
      return;
    }
    if (isPressed(BTN_MENU)) {
      lcd.clear();
      lcd.print("Cancelled");
      delay(1000);
      return;
    }
    delay(50);
  }
}

void confirmCalSoil() {
  lcd.clear();
  lcd.print("Calibrate Soil?");
  lcd.setCursor(0, 1);
  lcd.print("ENTER:Yes MENU:No");
  
  while (true) {
    if (isPressed(BTN_ENTER)) {
      calibrateSoilSensor();
      return;
    }
    if (isPressed(BTN_MENU)) {
      lcd.clear();
      lcd.print("Cancelled");
      delay(1000);
      return;
    }
    delay(50);
  }
}

void calibrateSoilSensor() {
  lcd.clear();
  lcd.print("Dry Sensor...");
  lcd.setCursor(0, 1);
  lcd.print("Press ENTER");
  
  while (!isPressed(BTN_ENTER)) delay(50);
  
  calSoilDry = readSoilAverage();
  lcd.clear();
  lcd.print("Dry Value:");
  lcd.setCursor(0, 1);
  lcd.print(calSoilDry);
  delay(2000);
  
  lcd.clear();
  lcd.print("Wet Sensor...");
  lcd.setCursor(0, 1);
  lcd.print("Press ENTER");
  
  while (!isPressed(BTN_ENTER)) delay(50);
  
  calSoilWet = readSoilAverage();
  lcd.clear();
  lcd.print("Wet Value:");
  lcd.setCursor(0, 1);
  lcd.print(calSoilWet);
  delay(2000);
  
  EEPROM.write(CAL_SOIL_DRY_ADDR, calSoilDry & 0xFF);
  EEPROM.write(CAL_SOIL_DRY_ADDR + 1, (calSoilDry >> 8) & 0xFF);
  EEPROM.write(CAL_SOIL_WET_ADDR, calSoilWet & 0xFF);
  EEPROM.write(CAL_SOIL_WET_ADDR + 1, (calSoilWet >> 8) & 0xFF);
  EEPROM.commit();
  
  lcd.clear();
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Saved!");
  delay(1000);
}

void backupCalibrationData() {
  // Implement your backup logic here
  lcd.clear();
  lcd.print("Backup feature");
  lcd.setCursor(0, 1);
  lcd.print("not implemented");
  delay(2000);
}

void restoreCalibrationData() {
  // Implement your restore logic here
  lcd.clear();
  lcd.print("Restore feature");
  lcd.setCursor(0, 1);
  lcd.print("not implemented");
  delay(2000);
}

void setWiFiManagerPortal() {
  lcd.clear();
  lcd.print("Starting WiFi");
  lcd.setCursor(0, 1);
  lcd.print("Manager...");
  
  WiFiManager wm;
  wm.startConfigPortal("SmartGardenAP");
  
  lcd.clear();
  lcd.print("WiFi Connected");
  delay(1000);
}
