#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

//Let's find out what resistor values we will need for our RVD:
//https://www.esp8266.com/viewtopic.php?f=5&t=5556&start=5

//Vinmax = 5V
//Voutmax = VADCin_max = 1V
//Vout=Vin*R2/(R1+R2)

//From the resistor ratio calculation values are: 
//R1=40K and R2=10k. 
//You can choose also other values as long as you keep accurate 
//the ratio between, better go upper that that. 
//400k and 100k should give the same result for example but be 
//careful with the input impedance! As a general practice a Op Amp Buffer will be a good add-on.

const char *ssid = "xx";
const char *password = "xxx";
String serverName = "http://xxx/emoncms/input/post";

String apiKey = "xxxx";
String inputName = "waterLevel";
String inputNameSub1 = "waterPressure";
String inputNameSub2 = "Voltage";

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

const float OffSet = 0.483;
float V, P;

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

void setup()
{
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  Serial.println("/** ESP8266 Water pressure sensor **/");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" ");
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
      V = analogRead(A0) * 1.00 / 1024; // Sensor output voltage
      P = (V - OffSet) * 250;           // Calculate water pressure

      Serial.print("Voltage:");
      Serial.print(V, 3);
      Serial.println("V");

      Serial.print("Pressure:");
      Serial.print(P, 1);
      Serial.println(" KPa");
      Serial.println();

      String serverPath = serverName + "/" + inputName + "?fulljson={%22" + inputNameSub1 + "%22:" + P + ",%22" + inputNameSub2 + "%22:" + V + "}&apikey=" + apiKey;

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0)
      {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
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