
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#else
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>           // Adafruit sensor
#include <Adafruit_BME280.h>           // For BME280 support
Adafruit_BME280 sensor;                // I2C mode
#define SensorAddress   0x76           // Use 0x77 for an Adafruit variant

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid     = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Default Threshold Temperature Value
int    Humidity;
String inputMessage        = "55.0";
String lastHumidity        = "";
String enableSwitchChecked = "checked";
String inputMessage2       = "true";
String switchSTATE         = "OFF";
bool   simulating          = true;
float  hysteresis          = 1;

// HTML web page to handle 2 input fields (threshold_input, enable_arm_input)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Temperature Threshold Output Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  </head><body>
  <h2>Humidity Value: %HUMIDITY% </h2> 
  <h2>Switch Value: %THRESHOLD% </h2>
  <h2>Switch State: %STATE% </h2>
  <h3>Enter Humidistat Trigger Value:</h3>
  <form action="/get">
    Humidity Threshold <input type="number" step="1" name="threshold_input" value="%THRESHOLD%" required><br>
    Enable Switching   <input type="checkbox" name="enable_switch_input" value="true" %ENABLE_SWITCH_INPUT%><br><br>
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

// Replaces placeholder with BME280 values
String processor(const String& var) {
  //Serial.println(var);
  if (var == "HUMIDITY") {
    return lastHumidity;
  }
  else if (var == "THRESHOLD") {
    return inputMessage;
  }
  else if (var == "ENABLE_SWITCH_INPUT") {
    return enableSwitchChecked;
  }
  else if (var == "STATE") {
    return switchSTATE;
  }
  return String();
}

// Flag variable to keep track if triggers was activated or not
bool triggerActive = false;

const char* PARAM_INPUT_1 = "threshold_input";
const char* PARAM_INPUT_2 = "enable_switch_input";

unsigned long previousMillis = 0;
const long    interval       = 5000;

// GPIO where the FAN Switch is connected to
const int output = 2;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());

  pinMode(output, OUTPUT);
  digitalWrite(output, LOW);

  // Set the variable simulating to true for the actual sensor, otherwise false
  simulating = true;

  // Start and read the BME280 sensor
  StartSensor();
  ReadSensor();

  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Receive an HTTP GET request at <ESP_IP>/get?threshold_input=<inputMessage>&enable_arm_input=<inputMessage2>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
    // GET threshold_input value on <ESP_IP>/get?threshold_input=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      // GET enable_Switch_input value on <ESP_IP>/get?enable_switch_input=<inputMessage2>
      if (request->hasParam(PARAM_INPUT_2)) {
        inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
        enableSwitchChecked = "checked";
      }
      else {
        inputMessage2 = "false";
        enableSwitchChecked = "";
      }
    }
    Serial.println(inputMessage);
    Serial.println(inputMessage2);
    request->send_P(200, "text/html", index_html, processor);
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ReadSensor();
    Serial.println(String(Humidity) + "%");
    lastHumidity = String(Humidity);
    // Check if Humidity is above threshold and if it needs to trigger output
    if (Humidity > (inputMessage.toFloat() + hysteresis) && inputMessage2 == "true" && !triggerActive) {
      String message = String("Humidity above threshold. Current Humidity: ") + String(Humidity) + "%";
      Serial.println(message);
      triggerActive = true;
      digitalWrite(output, HIGH);
      switchSTATE = "ON";
    }
    // Check if Humidity is below threshold and if it needs to trigger output
    if ((Humidity < (inputMessage.toFloat()) - hysteresis) && inputMessage2 == "true" && triggerActive) {
      String message = String("Humidity below threshold. Current Humidity: ") + String(Humidity) + "%";
      Serial.println(message);
      triggerActive = false;
      digitalWrite(output, LOW);
      switchSTATE = "OFF";
    }
    if (triggerActive == false) switchSTATE = "OFF";
  }
}

//#########################################################################################
void StartSensor() {
  Wire.setClock(100000);                           // Slow down the SPI bus for some BME280 devices
  if (!simulating) {                               // If not sensor simulating, then start the real one
    bool status = sensor.begin(SensorAddress);     // You can also pass a Wire library object like &Wire2, e.g. status = bme.begin(0x76, &Wire2);
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address and sensor ID!");
      Serial.print("SensorID is: 0x"); Serial.println(sensor.sensorID(), 16);
      Serial.print("       ID of 0xFF probably means a bad address, or a BMP 180 or BMP 085 or BMP280\n");
      Serial.print("  ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("       ID of 0x60 represents a BME 280.\n");
    }
    else {
      Serial.println("Sensor started...");
    }
    delay(1000);                                // Wait for sensor to start
  }
}
//#########################################################################################
void ReadSensor() {
  if (simulating) {
    Humidity = random(45, 65);                // Generate a random humidity value between 45% and 65%
  }
  else
  {
    while (isnan(sensor.readHumidity())) { }     // Make sure there are no reading errors
    Humidity = sensor.readHumidity();
    Serial.println("Humidity = " + String(Humidity, 0));
  }
}
