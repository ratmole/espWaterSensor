#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Smoothed.h>

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

const char *ssid = "xxx";
const char *password = "xxx";
String serverName = "http://xxx/emoncms/input/post";

String apiKey = "xxx";
String inputName = "waterLevel";
String inputNameSub1 = "waterPressure";
String inputNameSub2 = "Voltage";
String inputNameSub3 = "Percentage";

String Name = "Water Level";

int analogPin = A0;
int countStall = 0;

float V, Vreal, P, WP;
float minV = 1024.000;
float minP = 1024.000;
float minVreal = 1024.000;

float minWP = 1024.000;

float Vpre = 0;
float maxV = 0.000;
float maxVreal = 0.000;
float maxP = 0.000;
float maxWP = 0.000;
float lastTime = 0;

unsigned long timerDelay = 10000;
float OffSet = 0.002;
float Max_ADCvoltage = 0.740;

bool boolTimeDate = false;
String bootTimeDate;
String currentDate;

bool shouldReboot = false;

Smoothed <float> mySensor; 

// Week Days
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

const char *html = "<html>"
                   "<meta http-equiv='refresh' content='5'>"
                   "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                   "<style>"
                   "p {font-size:10px;line-height:12px;}"
                   ".tank {width: 150px;height:300px;border: 2px solid black;background: white;position: relative;display: inline-block;margin: 10px;}"
                   ".tank .water {position:absolute;background: #4CAF50;width:100%%;bottom: 0;}"
                   ".tank .value {position:absolute;width:100%%;vertical-align:middle;bottom:0;}"
                   ".tank .min {position:absolute;width:100%%;border-top: 1px solid red;bottom:0;}"
                   ".tank .max {position:absolute;width:100%%;border-top: 1px solid blue;bottom:0;}"
                   ".tank .scaleValue {position:absolute;right:175px !important;;font-size:10px;;border-top:none !important;}"
                   ".tank ._0  {position:absolute;width:10px;border-top: 1px solid black;bottom:0;right:155px;}"
                   ".tank ._5  {position:absolute;width:5px;border-top: 1px solid black;bottom:15;right:155px;}"
                   ".tank ._10 {position:absolute;width:10px;border-top: 1px solid black;bottom:30;right:155px;}"
                   ".tank ._15  {position:absolute;width:5px;border-top: 1px solid black;bottom:45;right:155px;}"
                   ".tank ._20 {position:absolute;width:10px;border-top: 1px solid black;bottom:60;right:155px;}"
                   ".tank ._25  {position:absolute;width:5px;border-top: 1px solid black;bottom:75;right:155px;}"
                   ".tank ._30 {position:absolute;width:10px;border-top: 1px solid black;bottom:90;right:155px;}"
                   ".tank ._35  {position:absolute;width:5px;border-top: 1px solid black;bottom:105;right:155px;}"
                   ".tank ._40 {position:absolute;width:10px;border-top: 1px solid black;bottom:120;right:155px;}"
                   ".tank ._45  {position:absolute;width:5px;border-top: 1px solid black;bottom:135;right:155px;}"
                   ".tank ._50 {position:absolute;width:10px;border-top: 1px solid black;bottom:150;right:155px;}"
                   ".tank ._55  {position:absolute;width:5px;border-top: 1px solid black;bottom:165;right:155px;}"
                   ".tank ._60 {position:absolute;width:10px;border-top: 1px solid black;bottom:180;right:155px;}"
                   ".tank ._65  {position:absolute;width:5px;border-top: 1px solid black;bottom:195;right:155px;}"
                   ".tank ._70 {position:absolute;width:10px;border-top: 1px solid black;bottom:210;right:155px;}"
                   ".tank ._75  {position:absolute;width:5px;border-top: 1px solid black;bottom:225;right:155px;}"
                   ".tank ._80 {position:absolute;width:10px;border-top: 1px solid black;bottom:240;right:155px;}"
                   ".tank ._85  {position:absolute;width:5px;border-top: 1px solid black;bottom:255;right:155px;}"
                   ".tank ._90 {position:absolute;width:10px;border-top: 1px solid black;bottom:270;right:155px;}"
                   ".tank ._95  {position:absolute;width:5px;border-top: 1px solid black;bottom:285;right:155px;}"
                   ".tank ._100 {position:absolute;width:10px;border-top: 1px solid black;bottom:300;right:155px;}"
                   "</style>"
                   "<body><center>"
                   "<p><b>%PLACEHOLDER_NAME%</b></p>"
                   "<p><b>%PLACEHOLDER_DATE%</b></p>"
                   "<p>%PLACEHOLDER_BOOTDATE%</p>"
                   "<p><b>Voltage: %PLACEHOLDER_VOLTAGE% V</b></p>"
                   "<p>Voltage Min: %PLACEHOLDER_VMIN% V ,"
                   "Voltage Max: %PLACEHOLDER_VMAX% V</p>"
                   "<p><b>Voltage Real: %PLACEHOLDER_VOLTAGEREAL% V</b></p>"
                   "<p>Voltage Min Real: %PLACEHOLDER_VMINREAL% V ,"
                   "Voltage Max Real: %PLACEHOLDER_VMAXREAL% V</p>"
                   "<p><b>Pressure: %PLACEHOLDER_PRESSURE% kPa</b></p>"
                   "<p>Pressure Min: %PLACEHOLDER_PMIN% kPa ,"
                   "Pressure Max: %PLACEHOLDER_PMAX% kPa</p>"
                   "<p><b>Percentage: %PLACEHOLDER_PERCENTAGE%</b></p>"
                   "<p>Percentage Min: %PLACEHOLDER_WPMIN% ,"
                   "Percentage Max: %PLACEHOLDER_WPMAX% </p>"
                   "<div class='tank'>"
                   "<div class='water' style='height:%PLACEHOLDER_PERCENTAGE%%%'></div>"
                   "<div class='value' style='bottom:%PLACEHOLDER_PERCENTAGE%%%'>%PLACEHOLDER_PERCENTAGE%%%</div>"
                   "<div class='min' style='bottom:%PLACEHOLDER_WPMIN%%%'></div>"
                   "<div class='max' style='bottom:%PLACEHOLDER_WPMAX%%%'></div>"
                   "<div class='scaleValue _0'>0</div><div class='_0'></div>"
                   "<div class='_5'></div>"
                   "<div class='scaleValue _10'>10</div><div class='_10'></div>"
                   "<div class='_15'></div>"
                   "<div class='scaleValue _20'>20</div><div class='_20'></div>"
                   "<div class='_25'></div>"
                   "<div class='scaleValue _30'>30</div><div class='_30'></div>"
                   "<div class='_35'></div>"
                   "<div class='scaleValue _40'>40</div><div class='_40'></div>"
                   "<div class='_45'></div>"
                   "<div class='scaleValue _50'>50</div><div class='_50'></div>"
                   "<div class='_55'></div>"
                   "<div class='scaleValue _60'>60</div><div class='_60'></div>"
                   "<div class='_65'></div>"
                   "<div class='scaleValue _70'>70</div><div class='_70'></div>"
                   "<div class='_75'></div>"
                   "<div class='scaleValue _80'>80</div><div class='_80'></div>"
                   "<div class='_85'></div>"
                   "<div class='scaleValue _90'>90</div><div class='_90'></div>"
                   "<div class='_95'></div>"
                   "<div class='scaleValue _100'>100</div><div class='_100'></div>"
                   "</div>"
                   "<p><a href='/reset'>reset</a></p>"
                   "<p><a href='/restart'>restart</a></p>"
                   "</center></body></html>";

const char *htmlMinimal = "<html>"
                          "<meta http-equiv='refresh' content='5'>"
                          "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                          "<style>"
                          "p {font-size:10px;line-height:12px;}"
                          ".tank {width: 150px;height: 300px;border: 2px solid black;background: white;position: relative;display: inline-block;margin: 10px;}"
                          ".tank .water {position:absolute;background: #4CAF50;width:100%%;bottom: 0;}"
                          ".tank .value {position:absolute;width:100%%;vertical-align:middle;bottom:0;}"
                          ".tank .min {position:absolute;width:100%%;border-top: 1px solid red;bottom:0;}"
                          ".tank .max {position:absolute;width:100%%;border-top: 1px solid blue;bottom:0;}"
                          ".tank .scaleValue {position:absolute;right:175px !important;;font-size:10px;;border-top:none !important;}"
                          ".tank ._0  {position:absolute;width:10px;border-top: 1px solid black;bottom:0;right:155px;}"
                          ".tank ._5  {position:absolute;width:5px;border-top: 1px solid black;bottom:15;right:155px;}"
                          ".tank ._10 {position:absolute;width:10px;border-top: 1px solid black;bottom:30;right:155px;}"
                          ".tank ._15  {position:absolute;width:5px;border-top: 1px solid black;bottom:45;right:155px;}"
                          ".tank ._20 {position:absolute;width:10px;border-top: 1px solid black;bottom:60;right:155px;}"
                          ".tank ._25  {position:absolute;width:5px;border-top: 1px solid black;bottom:75;right:155px;}"
                          ".tank ._30 {position:absolute;width:10px;border-top: 1px solid black;bottom:90;right:155px;}"
                          ".tank ._35  {position:absolute;width:5px;border-top: 1px solid black;bottom:105;right:155px;}"
                          ".tank ._40 {position:absolute;width:10px;border-top: 1px solid black;bottom:120;right:155px;}"
                          ".tank ._45  {position:absolute;width:5px;border-top: 1px solid black;bottom:135;right:155px;}"
                          ".tank ._50 {position:absolute;width:10px;border-top: 1px solid black;bottom:150;right:155px;}"
                          ".tank ._55  {position:absolute;width:5px;border-top: 1px solid black;bottom:165;right:155px;}"
                          ".tank ._60 {position:absolute;width:10px;border-top: 1px solid black;bottom:180;right:155px;}"
                          ".tank ._65  {position:absolute;width:5px;border-top: 1px solid black;bottom:195;right:155px;}"
                          ".tank ._70 {position:absolute;width:10px;border-top: 1px solid black;bottom:210;right:155px;}"
                          ".tank ._75  {position:absolute;width:5px;border-top: 1px solid black;bottom:225;right:155px;}"
                          ".tank ._80 {position:absolute;width:10px;border-top: 1px solid black;bottom:240;right:155px;}"
                          ".tank ._85  {position:absolute;width:5px;border-top: 1px solid black;bottom:255;right:155px;}"
                          ".tank ._90 {position:absolute;width:10px;border-top: 1px solid black;bottom:270;right:155px;}"
                          ".tank ._95  {position:absolute;width:5px;border-top: 1px solid black;bottom:285;right:155px;}"
                          ".tank ._100 {position:absolute;width:10px;border-top: 1px solid black;bottom:300;right:155px;}"
                          "</style>"
                          "<body><center>"
                          "<p><b>%PLACEHOLDER_NAME%</b></p>"
                          "<p><b>%PLACEHOLDER_DATE%</b></p>"
                          "<p>%PLACEHOLDER_BOOTDATE%</p>"
                          "<div class='tank'>"
                          "<div class='water' style='height:%PLACEHOLDER_PERCENTAGE%%%'></div>"
                          "<div class='value' style='bottom:%PLACEHOLDER_PERCENTAGE%%%'>%PLACEHOLDER_PERCENTAGE%%%</div>"
                          "<div class='min' style='bottom:%PLACEHOLDER_WPMIN%%%'></div>"
                          "<div class='max' style='bottom:%PLACEHOLDER_WPMAX%%%'></div>"
                          "<div class='scaleValue _0'>0</div><div class='_0'></div>"
                          "<div class='_5'></div>"
                          "<div class='scaleValue _10'>10</div><div class='_10'></div>"
                          "<div class='_15'></div>"
                          "<div class='scaleValue _20'>20</div><div class='_20'></div>"
                          "<div class='_25'></div>"
                          "<div class='scaleValue _30'>30</div><div class='_30'></div>"
                          "<div class='_35'></div>"
                          "<div class='scaleValue _40'>40</div><div class='_40'></div>"
                          "<div class='_45'></div>"
                          "<div class='scaleValue _50'>50</div><div class='_50'></div>"
                          "<div class='_55'></div>"
                          "<div class='scaleValue _60'>60</div><div class='_60'></div>"
                          "<div class='_65'></div>"
                          "<div class='scaleValue _70'>70</div><div class='_70'></div>"
                          "<div class='_75'></div>"
                          "<div class='scaleValue _80'>80</div><div class='_80'></div>"
                          "<div class='_85'></div>"
                          "<div class='scaleValue _90'>90</div><div class='_90'></div>"
                          "<div class='_95'></div>"
                          "<div class='scaleValue _100'>100</div><div class='_100'></div>"
                          "</div>"
                          "</center></body></html>";
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

void storeMinMax(float V, float Vreal, float P, float WP)
{

  if (V < minV)
  {
    minV = V;
  }
  if (V > maxV)
  {
    maxV = V;
  }
  if (Vreal < minVreal)
  {
    minVreal = Vreal;
  }
  if (Vreal > maxVreal)
  {
    maxVreal = Vreal;
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
  if (var == "PLACEHOLDER_NAME")
  {
    return String(Name);
  }
  else if (var == "PLACEHOLDER_VOLTAGE")
  {
    return String((float)V, 3);
  }
  else if (var == "PLACEHOLDER_VOLTAGEREAL")
  {
    return String((float)Vreal, 3);
  }
  else if (var == "PLACEHOLDER_PRESSURE")
  {
    return String((float)P, 3);
  }
  else if (var == "PLACEHOLDER_PERCENTAGE")
  {
    return String((float)WP, 1);
  }
  else if (var == "PLACEHOLDER_VMIN")
  {
    return String((float)minV, 3);
  }
  else if (var == "PLACEHOLDER_VMAX")
  {
    return String((float)maxV, 3);
  }
  else if (var == "PLACEHOLDER_VMINREAL")
  {
    return String((float)minVreal, 3);
  }
  else if (var == "PLACEHOLDER_VMAXREAL")
  {
    return String((float)maxVreal, 3);
  }
  else if (var == "PLACEHOLDER_PMIN")
  {
    return String((float)minP, 3);
  }
  else if (var == "PLACEHOLDER_PMAX")
  {
    return String((float)maxP, 3);
  }
  else if (var == "PLACEHOLDER_WPMIN")
  {
    return String((float)minWP, 3);
  }
  else if (var == "PLACEHOLDER_WPMAX")
  {
    return String((float)maxWP, 3);
  }
  else if (var == "PLACEHOLDER_DATE")
  {
    return String(currentDate);
  }
  else if (var == "PLACEHOLDER_BOOTDATE")
  {
    return String(bootTimeDate);
  }
  return String();
}

AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void RESTART()
{
  Serial.println("RESTART");
  server.end();
  delay(2000);
  ESP.restart();
}

void setup()
{

  int smooth = (1800 / (timerDelay/1000)); // 30min values
  mySensor.begin(SMOOTHED_AVERAGE, smooth);	
  //mySensor.begin(SMOOTHED_EXPONENTIAL, 10);
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

  // Time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone
  timeClient.setTimeOffset(3600 * 3);

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

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", html, processor); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", htmlMinimal, processor); });

  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    shouldReboot = true;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->redirect("/get");
    request->send(response); });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    minV = 1024.000; minP = 1024.000; minWP = 1024.000;
    maxV = 0.000;maxP = 0.000;maxWP = 0.000;
    Serial.println("RESET");
    request->redirect("/get"); });

  AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}
void loop()
{

  

  if (shouldReboot)
  {
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }

  // Send an HTTP POST request depending on timerDelay
  if ((millis() - lastTime) > timerDelay)
  {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client;
      HTTPClient http;

      timeClient.update();

      time_t epochTime = timeClient.getEpochTime();
      String formattedTime = timeClient.getFormattedTime();

      // int currentHour = timeClient.getHours();
      // int currentMinute = timeClient.getMinutes();
      // int currentSecond = timeClient.getSeconds();
      String weekDay = weekDays[timeClient.getDay()];

      // Get a time structure
      struct tm *ptm = gmtime((time_t *)&epochTime);

      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon + 1;
      String currentMonthName = months[currentMonth - 1];
      int currentYear = ptm->tm_year + 1900;

      currentDate = String(formattedTime) + " " + String(monthDay) + "/" + String(currentMonth) + "/" + String(currentYear);

      if (boolTimeDate == false)
      {
        bootTimeDate = currentDate;
        boolTimeDate = true;
        }

      Vreal = average(analogPin, 1, 0) * 1 / 1023; // Sensor output voltage

      V = (Vreal - OffSet);
      // P = (V - OffSet) * 250;             //Calculate water pressure for 5V ACD + 1 MPa pressure sensor

      mySensor.add(V);
      V = mySensor.get();

      P = mapfloat(V, 0.000, Max_ADCvoltage, 0, 10); // Calculate water pressure
      WP = mapfloat(V, 0.000, Max_ADCvoltage, 0, 100);

      storeMinMax(V, Vreal, P, WP);
      
      Serial.print(currentDate);
      Serial.print(" # Voltage:");
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