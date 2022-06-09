/*
  I2C Master Demo
  i2c-master-demo.ino
  Demonstrate use of I2C bus
  Master sends character and gets reply from Slave
  Arpit Shrivastava
*/

// Include Arduino Wire library for I2C
#include <Wire.h>
#include <Arduino_JSON.h>

#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = "c0ea98d9ddd7411d966b057bef917054";
char ssid[] = "ni2-2G";
char pass[] = "nahimalum";

float rainfall_amount;
float windSPEED;

// Define Slave I2C Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define RAINANSWERSIZE 32
#define WINDANSWERSIZE 32

// int flag = 0;
String response0 = "";
// String response1 = "";
// String finalResult = "";

int seconds = 0;
double amount = 0;
int count = 0;
double windSpeed = 0;
double windDirection = 0.0;
String windDirNotation = "";
int rain_flag_count = 0;
int wind_flag_count = 0;

void whatIsRain() {
  // receive data for Rain Gauge
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  Serial.println("Receiving Rain data...");
  askForRain();
}

void whatIsWind() {
  // receive data for Weather Station
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(1);
  Wire.endTransmission();
  Serial.println("Receiving Wind data...");
  askForWind();
}

void askForRain()
{

  Serial.println();
  Wire.requestFrom(SLAVE_ADDR, RAINANSWERSIZE);
  response0 = "";
  while (Wire.available())
  {
    char b = Wire.read();
    response0 += b;
  }
  Serial.println(response0);

  //read JSON
  JSONVar myObject = JSON.parse(response0);
  //check integrity
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Parsing input rain failed!");
    if (rain_flag_count < 3)
    {
      delay(random(1000, 5000));
      askForRain();
      rain_flag_count += 1;
    }
    else
      return;
  }
  //if pauload is valid print Object
  Serial.print("JSON.typeof(myObject) = ");
  Serial.println(JSON.typeof(myObject)); // prints: object

  if (myObject.hasOwnProperty("s"))
  {
    Serial.print("s = ");
    seconds = (int)myObject["s"];
    Serial.println(seconds);
  }

  if (myObject.hasOwnProperty("a"))
  {
    Serial.print("a = ");
    amount = myObject["a"];
    rainfall_amount=amount;
    Blynk.virtualWrite(V1,rainfall_amount);
    Serial.println(amount);
  }

  if (myObject.hasOwnProperty("c"))
  {
    Serial.print("c = ");
    count = (int)myObject["c"];
    Blynk.virtualWrite(V0,count);
    Serial.println(count);
  }
  rain_flag_count = 0;
}

void askForWind()
{

  Serial.println();
  Wire.requestFrom(SLAVE_ADDR, WINDANSWERSIZE);
  response0 = "";
  while (Wire.available())
  {
    char b = Wire.read();
    response0 += b;
  }
  Serial.println(response0);

  //read JSON
  JSONVar myObject = JSON.parse(response0);
  //check integrity
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Parsing input wind failed!");
    if (wind_flag_count < 3)
    {
      delay(random(1000, 5000));
      askForWind();
      wind_flag_count += 1;
    }
    else
      return;
  }
  //if pauload is valid print Object
  Serial.print("JSON.typeof(myObject) = ");
  Serial.println(JSON.typeof(myObject)); // prints: object

  if (myObject.hasOwnProperty("ws"))
  {
    Serial.print("ws = ");
    windSpeed = (int)myObject["ws"];
    windSPEED=windSpeed;
    Blynk.virtualWrite(V2,windSPEED);
    Serial.println(windSpeed);
  }

  if (myObject.hasOwnProperty("d"))
  {
    Serial.print("d = ");
    windDirection = (myObject["d"]);
    Blynk.virtualWrite(V3,windDirection);
    Serial.println(windDirection);
  }

  if (myObject.hasOwnProperty("n"))
  {
    Serial.print("n = ");
    windDirNotation = myObject["n"];    
    Serial.println(windDirNotation);
  }
  wind_flag_count = 0;
}

void setup()
{
  // Initialize I2C communications as Master
  Wire.begin();

  // Setup serial monitor
  Serial.begin(9600);
  Serial.println("I2C Master Demonstration");

  Blynk.begin(auth, ssid, pass);

    
}


void loop()
{
  Blynk.run();
  whatIsRain();
  delay(10);
  whatIsWind();


Serial.println("****** vinay *******" );
Serial.print("ws = ");
Serial.print(windSPEED);
Serial.print("            dir = ");
Serial.println(windDirection);

Serial.print("Count = ");
Serial.print(count);
Serial.print("            Amt = ");
Serial.println(rainfall_amount); 
 
  delay(10000);
}
