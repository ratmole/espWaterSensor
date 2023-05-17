#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <AsyncTCP.h>
#endif

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// Let's find out what resistor values we will need for our RVD:
// https://www.esp8266.com/viewtopic.php?f=5&t=5556&start=5

// Vinmax = 5V
// Voutmax = VADCin_max = 1V
// Vout=Vin*R2/(R1+R2)

// From the resistor ratio calculation values are:
// R1=40K and R2=10k.
// You can choose also other values as long as you keep accurate
// the ratio between, better go upper that that.
// 400k and 100k should give the same result for example but be
// careful with the input impedance! As a general practice a Op Amp Buffer will be a good add-on.

/************************************************************
  Water Sensor Calibration

  The output voltage offset of the sensor is 0.5V (norminal).
  However, due to the zero-drifting of the internal circuit, the
  no-load output voltage is not exactly 0.5V. Calibration needs to
  be carried out as follow.

  Calibration: connect the 3 pin wire to the Arduio UNO (VCC, GND and Signal)
  without connecting the sensor to the water pipe and run the program
  for once. Mark down the LOWEST voltage value through the serial
  monitor and revise the "OffSet" value to complete the calibration.

  After the calibration the sensor is ready for measuring!
**************************************************************/

const char *ssid = "ii";
const char *password = "123qweasdzxc123qweasdzxc";
String serverName = "http://ratmole.ddns.net/emoncms/input/post";

String apiKey = "388a981e8f2a02d4d644abde0dec5146";
String inputName = "waterLevel";
String inputNameSub1 = "waterPressure";
String inputNameSub2 = "Voltage";
String inputNameSub3 = "Percentage";

String Name = "Water Level";

int analogPin = A0;

float minV, minP, minWP = 1024;

float V, P, WP, maxV, maxP, maxWP, lastTime = 0;
unsigned long timerDelay = 2000;
const float OffSet = 0.149;
const float Max_ADCvoltage = 1.000;
const float pressureMultiplier = 5.0;

const char *html = "<html>"
                   "<meta http-equiv='refresh' content='30'>"
                   "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                   "<link rel='stylesheet' href='https://www.w3schools.com/w3css/4/w3.css'><body><center>"
                   "<p><b>Voltage: %PLACEHOLDER_VOLTAGE% V</b></p>"
                   "<p>Voltage Min: %PLACEHOLDER_VMIN% V ,"
                   "Voltage Max: %PLACEHOLDER_VMAX% V</p>"
                   "<p><b>Pressure: %PLACEHOLDER_PRESSURE% kPa</b></p>"
                   "<p>Pressure Min: %PLACEHOLDER_PMIN% kPa ,"
                   "Pressure Max: %PLACEHOLDER_PMAX% kPa</p>"
                   "<p><b>Percentage: %PLACEHOLDER_PERCENTAGE%</b></p>"
                   "<p>Percentage Min: %PLACEHOLDER_WPMIN% ,"
                   "Percentage Max: %PLACEHOLDER_WPMAX% </p></center>"
                   "<div class='w3-container'>"
                   "<div class='w3-border'>"
                   "<div class='w3-green' style='height:24px;width:%PLACEHOLDER_PERCENTAGE%%'></div>"
                   "</div>"
                   "</div>"
                   "</body></html>";

void blink(int times)
{
  int count = 0;

  while (count <= times)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    count++;
  }
}

void storeMinMax(float V, float P, float WP)
{
  if (V < minV)
  {
    minV = V;
  }
  if (V > maxV)
  {
    maxV = V;
  }
  if (P < minP)
  {
    minP = P;
  }
  if (P > maxP)
  {
    maxP = P;
  }
  if (WP < minWP)
  {
    minWP = WP;
  }
  if (WP > maxWP)
  {
    maxWP = WP;
  }
}

float average(int analogPin, int samples, int delaymSec)
{
  float sum = 0.000;

  if (samples == 1)
  {
    return (float)analogRead(analogPin);
  }
  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(analogPin);
    delay(delaymSec);
  }
  return float(sum) / samples;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

String processor(const String &var)
{

  if (var == "PLACEHOLDER_VOLTAGE")
  {
    return String(V);
  }
  else if (var == "PLACEHOLDER_PRESSURE")
  {
    return String(P);
  }
  else if (var == "PLACEHOLDER_PERCENTAGE")
  {
    return String(WP);
  }
  else if (var == "PLACEHOLDER_VMIN")
  {
    return String(minV);
  }
  else if (var == "PLACEHOLDER_VMAX")
  {
    return String(maxV);
  }
  else if (var == "PLACEHOLDER_PMIN")
  {
    return String(minP);
  }
  else if (var == "PLACEHOLDER_PMAX")
  {
    return String(maxP);
  }
  else if (var == "PLACEHOLDER_WPMIN")
  {
    return String(minWP);
  }
  else if (var == "PLACEHOLDER_WPMAX")
  {
    return String(maxWP);
  }
  return String();
}

AsyncWebServer server(80);

void setup()
{
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  Serial.println("/** ESP8266 Water pressure sensor with OTA Enabled **/");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" ");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  blink(1);
  Serial.println("");
  Serial.print("Timer set to ");
  Serial.print(timerDelay / 1000);
  Serial.print(" seconds, it will take ");
  Serial.print(timerDelay / 1000);
  Serial.println(" seconds before publishing the first reading.");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", Name); });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", html, processor); });

  AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}
void loop()
{
  // Send an HTTP POST request depending on timerDelay
  if ((millis() - lastTime) > timerDelay)
  {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client;
      HTTPClient http;

      // Connect sensor to Analog ADC
      V = average(analogPin, 1, 0) * 1 / 1023; // Sensor output voltage

      // P = (V - OffSet) * 250;             //Calculate water pressure for 5V ACD + 1 MPa pressure sensor
      P = mapfloat((V - OffSet), 0.000, Max_ADCvoltage, 0.0, 10.0); // Calculate water pressure
      WP = mapfloat((V - OffSet), 0.000, Max_ADCvoltage, 0.0, 100.0);
      storeMinMax(V, P, WP);

      Serial.print("Voltage:");
      Serial.print(V, 3);
      Serial.print("V");
      Serial.print(", Pressure:");
      Serial.print(P, 3);
      Serial.print(" kPa");
      Serial.print(", Percentage:");
      Serial.print(WP, 3);
      Serial.print(" %");
      Serial.print(", V MIN:");
      Serial.print(minV, 3);
      Serial.print("V");
      Serial.print(" V MAX:");
      Serial.print(maxV, 3);
      Serial.print("V");
      Serial.print(", P MIN:");
      Serial.print(minP, 3);
      Serial.print("kPa");
      Serial.print(" P MAX:");
      Serial.print(maxP, 3);
      Serial.print("kPa");
      Serial.print(", WP MIN:");
      Serial.print(minWP, 3);
      Serial.print("%");
      Serial.print(" WP MAX:");
      Serial.print(maxWP, 3);
      Serial.print("%");
      Serial.println();

      String serverPath = serverName + "/" + inputName + "?fulljson={%22" + inputNameSub1 + "%22:" + P + ",%22" + inputNameSub2 + "%22:" + V + ",%22" + inputNameSub3 + "%22:" + WP + "}&apikey=" + apiKey;

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0)
      {
        // Serial.print("HTTP Response code: ");
        // Serial.println(httpResponseCode);
        // String payload = http.getString();
        // Serial.println(payload);
        blink(1);
      }
      else
      {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
        blink(10);
      }
      // Free resources
      http.end();
    }
    else
    {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}