/*
Sensor Values on Virtual Pins
V1	=	UV
V2	=	Lux
V3	=	BME Temperature
V4	=	BME Pressure
V5	=	BME Humidity
V6	=	Soil Temperature
V7	=	Soil Humidity
V8	=	Soil EC
V9	=	Soil TDS
V10	=	Soil CF
V11	=	PH
V12	=	Board Temperature
V13	=	Soil Ch1 Temperature
V14	=	Soil Ch2 Temperature
V15	=	Relay1
V16	=	Relay2
V17	=	Soft reset
V18	=	Sleep control
V19 = Rain count
V20 = Rain amount
V21 = Wind Speed
V22 = Wind Direction
V23 = Wind Notatio

*/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "TMPL6QpTbLFv"
#define BLYNK_DEVICE_NAME "EarthSense1"
#define BLYNK_AUTH_TOKEN "yGS8BXO7fOJyfYuwxBv_0NxbLNRfn4Gz"

#define PH_IN 36               //Sensor_VP
#define BATT_EXT_SENSE 39       //Sensor_VN
#define ANALOG_EXTRA 13
#define INDICATION_LED 2
#define ECE_MODE_EN 4
#define ECE_RX 17
#define ECE_TX 16
#define MISO 19
#define MOSI 23
#define DS18B20_INT 32
#define SOIL_TEMP_1 33
#define SOIL_TEMP_2 25
#define RELAY_CTRL_1 26
#define RELAY_CTRL_2 27
#define LEAF_OUT_1 34
#define LEAF_OUT_2 35
#define SEALEVELPRESSURE_HPA (1013.25)
#define SLAVE_ADDR 9

//Enable this to print values on serial port.
//#define SERIAL_PRINT_SENSOR_VALUES

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <SPI.h>
#include <BH1750.h>
#include <Adafruit_VEML6070.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_JSON.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "time.h"

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int off_cycle=0;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;


char auth[] = BLYNK_AUTH_TOKEN;

BH1750 lightMeter;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
Adafruit_BME280 bme; // I2C
SoftwareSerial master(ECE_RX, ECE_TX);

const char* ntpServer = "in.pool.ntp.org";
const long  gmtOffset_sec = 19800; //5.30 hrs 
const int   daylightOffset_sec = 0;


float BatteryValue = 0;
float Batt_Voltage = 0;

float Temp[10];
float avgTemp = 0;
float Press[10];
float avgPress = 0;
float Hum[10];
float avgHum = 0;
int i=0;

char val[11] ;
 //Define an array, the internal is a hexadecimal command that retrieves real-time data
//01 04 0000 0003 B00B
char tx[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x03, 0xB0, 0x0B};
float TDS_ppm=0;
float TDS_CF=0;
int relay1value=0;
int relay2value=0;
int relay1state=0;
int relay2state=0;
int sleepvalue=1; // if value less than 1 then sleep active

int16_t PH_buf[10], temp;
unsigned long int PHavgValue;
float PHvoltage=0;
float PHvalue=0;
char temperatureString[6];

// Weather sensor related declaration
#define RAINANSWERSIZE 32
#define WINDANSWERSIZE 32
// #define I2C_SDA 21
// #define I2C_SCL 22
String response0 = "";
int seconds = 0;
double rain_amount = 0;
int bucket_count = 0;
double windSpeed = 0;
double windDirection = 0.0;
String windDirNotation = "";
int rain_flag_count = 0;
int wind_flag_count = 0;




OneWire oneWire1(DS18B20_INT);
OneWire oneWire2(SOIL_TEMP_1);
OneWire oneWire3(SOIL_TEMP_2);
DallasTemperature DS18B20_1(&oneWire1);
DallasTemperature DS18B20_2(&oneWire2);
DallasTemperature DS18B20_3(&oneWire3);

// Your WiFi credentials.
// Set password to "" for open networks.
// char ssid[] = "VinayRog";
// char pass[] = "jaimatadi";

char ssid[] = "ni2-2G";
char pass[] = "nahimalum";

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
//    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    //case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
//    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
//    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  off_cycle++;
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
  // checkRelay1();
  // checkRelay2();
}

// reset the device
BLYNK_WRITE(V17)
{
ESP.restart();
}

void turnONRelay1()
{
    if(relay1value>0)
      digitalWrite(RELAY_CTRL_1, LOW);//Toggle relay in switch is ON
    else
      digitalWrite(RELAY_CTRL_1, HIGH);
}

void turnONRelay2()
{
    if(relay2value>0)
      digitalWrite(RELAY_CTRL_2, LOW);//Toggle relay in switch is ON
    else
      digitalWrite(RELAY_CTRL_2, HIGH);
}

void turnOFFRelay1()
{
  digitalWrite(RELAY_CTRL_1, HIGH);
}

void turnOFFRelay2()
{
  digitalWrite(RELAY_CTRL_2, HIGH);
}

// Inquire about Rain qty
void whatIsRain() 
{
  // receive data for Rain Gauge
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.println("Receiving Rain data...");
  #endif
  askForRain();
  Blynk.virtualWrite(V19,bucket_count); // Rain Bucket Count
  Blynk.virtualWrite(V20,rain_amount); // Rain Amount

}

void whatIsWind() {
  // receive data for Weather Station
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(1);
  Wire.endTransmission();
  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.println("Receiving Wind data...");
  #endif
  askForWind();
  Blynk.virtualWrite(V21,windSpeed); // wind speed in m/s
  Blynk.virtualWrite(V22,windDirection); // wind direction in degrees
  Blynk.virtualWrite(V23,windDirNotation); // wind direction notation in N, NE, NNE, etc.

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
  #if defined(SERIAL_PRINT_SENSOR_VALUES)  
  Serial.println(response0);
  #endif

  //read JSON
  JSONVar myObject = JSON.parse(response0);
  //check integrity
  if (JSON.typeof(myObject) == "undefined")
  {
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.println("Parsing input rain failed!");
    #endif
    if (rain_flag_count < 3)
    {
      // delay(random(1000, 5000));
      // askForRain();
      // rain_flag_count += 1;
    }
    else
      return;
  }
  //if pauload is valid print Object
  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.print("JSON.typeof(myObject) = ");
  Serial.println(JSON.typeof(myObject)); // prints: object
  #endif

  if (myObject.hasOwnProperty("s"))
  {
    seconds = (int)myObject["s"];
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("s = ");
    Serial.println(seconds);
    #endif
  }

  if (myObject.hasOwnProperty("a"))
  {
    
    rain_amount = myObject["a"];
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("a = ");
    Serial.println(rain_amount);
    #endif
  }

  if (myObject.hasOwnProperty("c"))
  {
    bucket_count = (int)myObject["c"];
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("c = ");
    Serial.println(bucket_count);
    #endif
  }
  rain_flag_count = 0;
}

void askForWind()
{

  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.println();
  #endif  
  Wire.requestFrom(SLAVE_ADDR, WINDANSWERSIZE);
  response0 = "";
  while (Wire.available())
  {
    char b = Wire.read();
    response0 += b;
  }
  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.println(response0);
  #endif

  //read JSON
  JSONVar myObject = JSON.parse(response0);
  //check integrity
  if (JSON.typeof(myObject) == "undefined")
  {
    Serial.println("Parsing input wind failed!");
    if (wind_flag_count < 3)
    {
      // delay(random(1000, 5000));
      // askForWind();
      // wind_flag_count += 1;
    }
    else
      return;
  }
  //if pauload is valid print Object
  #if defined(SERIAL_PRINT_SENSOR_VALUES)
  Serial.print("JSON.typeof(myObject) = ");
  Serial.println(JSON.typeof(myObject)); // prints: object
  #endif

  if (myObject.hasOwnProperty("ws"))
  {
    windSpeed = (int)myObject["ws"];
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("ws = ");
    Serial.println(windSpeed);
    #endif
  }

  if (myObject.hasOwnProperty("d"))
  {
    windDirection = (myObject["d"]);
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("d = ");
    Serial.println(windDirection);
    #endif
  }

  if (myObject.hasOwnProperty("n"))
  {
    windDirNotation = myObject["n"];
    #if defined(SERIAL_PRINT_SENSOR_VALUES)
    Serial.print("n = "); 
    Serial.println(windDirNotation);
    #endif
  }
  wind_flag_count = 0;
}


void setup()
{
  // Debug console
  Serial.begin(115200);
  Wire.begin();

  off_cycle=0;
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 10000000, true);
  // Start an alarm
  timerAlarmEnable(timer);

  pinMode(INDICATION_LED, OUTPUT);
  pinMode(RELAY_CTRL_1, OUTPUT);
  pinMode(RELAY_CTRL_2, OUTPUT);
  digitalWrite(INDICATION_LED, HIGH);
  pinMode(ECE_MODE_EN, OUTPUT);//Enable of RS485
  master.begin (9600); // software serial port initialization
  digitalWrite(ECE_MODE_EN, HIGH);//Enable RS485
  Blynk.begin(auth, ssid, pass);
  delay(1000);  
  digitalWrite(INDICATION_LED, LOW);

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*  First we configure the wake up source, We set our ESP32 to wake up every 10 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

}


// void printLocalTime(){
//   struct tm timeinfo;
//   if(!getLocalTime(&timeinfo)){
//     Serial.println("Failed to obtain time");
//     return;
//   }
//   Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
//   Serial.print("Day of week: ");
//   Serial.println(&timeinfo, "%A");
//   Serial.print("Month: ");
//   Serial.println(&timeinfo, "%B");
//   Serial.print("Day of Month: ");
//   Serial.println(&timeinfo, "%d");
//   Serial.print("Year: ");
//   Serial.println(&timeinfo, "%Y");
//   Serial.print("Hour: ");
//   Serial.println(&timeinfo, "%H");
//   Serial.print("Hour (12 hour format): ");
//   Serial.println(&timeinfo, "%I");
//   Serial.print("Minute: ");
//   Serial.println(&timeinfo, "%M");
//   Serial.print("Second: ");
//   Serial.println(&timeinfo, "%S");

//   Serial.println("Time variables");
//   char timeHour[3];
//   strftime(timeHour,3, "%H", &timeinfo);
//   Serial.println(timeHour);
//   char timeWeekDay[10];
//   strftime(timeWeekDay,10, "%A", &timeinfo);
//   Serial.println(timeWeekDay);
//   Serial.println();
// }

void get_time()
{
  int hrs=0;
  int mins=0;
  int secs=0;
  int date=0;
  int month=0;
  int year=0;

char timeHour[3];
char timeWeekDay[5];

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

strftime(timeHour,3, "%H", &timeinfo);  
Serial.print(timeHour);

strftime(timeHour,3, "%M", &timeinfo);  
Serial.print("::");
Serial.print(timeHour);

strftime(timeHour,3, "%S", &timeinfo);  
Serial.print("::");
Serial.println(timeHour);
Serial.println("--------");


strftime(timeWeekDay,3, "%d", &timeinfo);
Serial.print(timeWeekDay);
Serial.print("::");
strftime(timeWeekDay,4, "%B", &timeinfo);
Serial.print(timeWeekDay);
Serial.print("::");
strftime(timeWeekDay,5, "%Y", &timeinfo);
Serial.println(timeWeekDay);

Serial.println();

}

BLYNK_WRITE(V15)
{
  relay1value = param.asInt(); 
  turnONRelay1();
}

BLYNK_WRITE(V16)
{
  relay2value = param.asInt();
  turnONRelay2();
}

BLYNK_WRITE(V18)
{
//if value is < 1 then will go in sleep  
  sleepvalue = param.asInt();  
}

void turnONsensor()
{
#if defined(SERIAL_PRINT_SENSOR_VALUES)   
  Serial.print("Turning ON Sensor Supply");
  Serial.println("---------------"); 
#endif   
  digitalWrite(INDICATION_LED, HIGH);
  delay(100);
}

void turnOFFsensor()
{
#if defined(SERIAL_PRINT_SENSOR_VALUES)   
  Serial.print("Turning OFF Sensor Supply");
  Serial.println("---------------"); 
#endif   
  digitalWrite(INDICATION_LED, LOW);
}
 void UVmeasure()
 {
   uv.begin(VEML6070_1_T);
   delay(10);
   uint16_t uv_sensor = uv.readUV();
#if defined(SERIAL_PRINT_SENSOR_VALUES)   
  Serial.print("Light UV: ");
  Serial.println(uv_sensor);
  Serial.println("---------------"); 
#endif   
   Blynk.virtualWrite(V1,uv_sensor);
 }

void luxMeasure()
{
  lightMeter.begin();
  delay(10);
  uint16_t lux = lightMeter.readLightLevel();
#if defined(SERIAL_PRINT_SENSOR_VALUES)   
  Serial.print("Light lux: ");
  Serial.print(lux);
  Serial.println(" lx");
  Serial.println("---------------"); 
#endif
  Blynk.virtualWrite(V0,lux);
}

//Read Battery Voltage
void BatteryMeasure()
{
  BatteryValue=0;
 for (i = 0; i < 10; i++)
  {
    Temp[i] = analogRead(BATT_EXT_SENSE);
    BatteryValue += Temp[i];
    delay(10);
  }

  BatteryValue = BatteryValue/10; // average the battery value
  Batt_Voltage = ((((BatteryValue / 1024) * 0.85) + 0.07)/0.2);

#if defined(SERIAL_PRINT_SENSOR_VALUES) 
  Serial.print("Battery Value before conversion: ");
  Serial.println(BatteryValue);
  Serial.print("Battery Voltage : ");
  Serial.println(Batt_Voltage);
  Serial.println("---------------"); 
#endif

  Blynk.virtualWrite(V2,Batt_Voltage);
}

void WeatherBMEMeasure()
{
  bme.begin();
//Temperature    
  avgTemp = 0;
  for (i = 0; i < 10; i++)
  {
    Temp[i] = bme.readTemperature();
    avgTemp += Temp[i];
    delay(5);
  }
  //Pressure
  avgPress = 0;
  for (i = 0; i < 10; i++)
  {
    Press[i] = (bme.readPressure() / 100.0F);
    avgPress += Press[i];
    delay(5);
  }

  //Humidity
  avgHum = 0;
  for (i = 0; i < 10; i++)
  {
    Hum[i] = bme.readHumidity();
    avgHum += Hum[i];
    delay(5);
  }
  avgTemp = (avgTemp / 10);
  avgPress= (avgPress / 10);
  avgHum  = (avgHum / 10);    

#if defined(SERIAL_PRINT_SENSOR_VALUES) 
  Serial.print("BME Temperature : ");
  Serial.println(avgTemp);
  Serial.print("BME Pressure : ");
  Serial.println(avgPress);
  Serial.print("BME Humidity : ");
  Serial.println(avgHum);
  Serial.println("---------------"); 
#endif

Blynk.virtualWrite(V3,avgTemp);
Blynk.virtualWrite(V4,avgPress);
Blynk.virtualWrite(V5,avgHum);

}

void ECMeasure()
{
  digitalWrite(ECE_MODE_EN, HIGH);
  master.write(tx, 8);//Send the command to read data (tx=string, number of bytes)
  digitalWrite(ECE_MODE_EN, LOW);
  master.readBytes(val, 11); //Read array to "val", 11 bytes

  short t3;
  short t1 = (byte)val[3];
  short t2 = (byte)val[4];
  t3 = t1 << 8 | t2;
  float soil_temp = (float) t3/100;//Temperature

  short h3 ;
  short h1 = (byte)val[5];
  short h2 = (byte)val[6];
  h3 = h1 << 8 | h2;
  float soil_hum = (float) h3/100;

  short soil_EC;
  short c1 = (byte)val[7];
  short c2 = (byte)val[8];
  soil_EC = c1 << 8 | c2;

//TDS Conversion
  TDS_ppm = 500 * (soil_EC/1000); // For Hanna factor=500, Eutech its 640
//Conductivity Factor  
  TDS_CF = 10 * (soil_EC/1000); // Conductivity Factor, just 10 x EC(in ms/cm)
// send sensor data to the serial console
#if defined(SERIAL_PRINT_SENSOR_VALUES)  
  Serial.print("Temp: ");
  Serial.print(soil_temp, 2);
  Serial.println("*C");

  Serial.print("Hum: ");
  Serial.print(soil_hum);
  Serial.println("%");

  Serial.print("EC: ");
  Serial.print(soil_EC, DEC);
  Serial.println("uS/cm");
  Serial.println("---------------"); 
#endif

if (val[0] == 0x01 && val[1] == 0x04 && soil_hum >= 0)
{
  Blynk.virtualWrite(V6,soil_temp);
  Blynk.virtualWrite(V7,soil_hum);
  Blynk.virtualWrite(V8,soil_EC);
  Blynk.virtualWrite(V9,TDS_ppm);
  Blynk.virtualWrite(V10,TDS_CF);
}
else
{
  Blynk.virtualWrite(V6,0);
  Blynk.virtualWrite(V7,0);
  Blynk.virtualWrite(V8,0);    
  Blynk.virtualWrite(V9,0);
  Blynk.virtualWrite(V10,0);
}
}

// Measure Anlog PH
void PHMeasure()
{

delay(1000);
//Capturing analog value of PH sensor
  for (int i = 0; i < 10; i++)
  {
    PH_buf[i] = analogRead(PH_IN);
    delay(50);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (PH_buf[i] > PH_buf[j])
      {
        temp = PH_buf[i];
        PH_buf[i] = PH_buf[j];
        PH_buf[j] = temp;
      }
    }
  }
  PHavgValue = 0;
  for (int i = 2; i < 8; i++)
    PHavgValue += PH_buf[i];

PHavgValue = PHavgValue/6;
PHvoltage =(PHavgValue * 3.3) / 4096;  
//PHvalue = -5.70 * PHvoltage + 21.44;               //-5.7x+21.34
PHvalue = -5.70 * PHvoltage + 21.6;               //-5.7x+21.34

// send sensor data to the serial console
#if defined(SERIAL_PRINT_SENSOR_VALUES)  
 Serial.print("PH Avg: ");
 Serial.println(PHavgValue);
 Serial.print("PH Volt: ");
 Serial.println(PHvoltage);
 Serial.print("PH Final: ");
 Serial.println(PHvalue);
 Serial.println("---------------"); 
#endif 
Blynk.virtualWrite(V11,PHvalue);

}

void InternalTemperature()
{
  DS18B20_1.begin();

float temp;
int numberOfDevices=0;
numberOfDevices = DS18B20_1.getDeviceCount();
if(numberOfDevices>0)
 {  do {
    DS18B20_1.requestTemperatures();
    temp = DS18B20_1.getTempCByIndex(0);
    delay(100);
  } while (temp == 85.0 || temp == (-127.0));
  float temperature = temp;
  dtostrf(temperature, 2, 2, temperatureString);

// send sensor data to the serial console
#if defined(SERIAL_PRINT_SENSOR_VALUES)  
  Serial.print("Board Temp = ");
  Serial.println(temperatureString);
  Serial.println("---------------"); 
#endif
  Blynk.virtualWrite(V12,temperatureString);
 }
 else
 {
#if defined(SERIAL_PRINT_SENSOR_VALUES)     
   Serial.println("Board Temp = Absent");
   Serial.println("---------------"); 
#endif   
 }
} 


void Soil_Temperature_1()
{
  DS18B20_2.begin();

float temp;
int numberOfDevices=0;
numberOfDevices = DS18B20_2.getDeviceCount();
if(numberOfDevices>0)
 {  do {
    DS18B20_2.requestTemperatures();
    temp = DS18B20_2.getTempCByIndex(0);
    delay(100);
  } while (temp == 85.0 || temp == (-127.0));
  float temperature = temp;
  dtostrf(temperature, 2, 2, temperatureString);

// send sensor data to the serial console
#if defined(SERIAL_PRINT_SENSOR_VALUES)  
  Serial.print("Soil Ch1 Temp = ");
  Serial.println(temperatureString);
  Serial.println("---------------"); 
#endif
  Blynk.virtualWrite(V13,temperatureString);
 }
 else
 {
  #if defined(SERIAL_PRINT_SENSOR_VALUES)  
   Serial.println("Soil Ch1 = Absent");
   Serial.println("---------------"); 
  #endif
 }
 
} 

void Soil_Temperature_2()
{
  DS18B20_3.begin();

float temp;
int numberOfDevices=0;
numberOfDevices = DS18B20_3.getDeviceCount();
if(numberOfDevices>0)
 {  do {
    DS18B20_3.requestTemperatures();
    temp = DS18B20_3.getTempCByIndex(0);
    delay(100);
  } while (temp == 85.0 || temp == (-127.0));
  float temperature = temp;
  dtostrf(temperature, 2, 2, temperatureString);

// send sensor data to the serial console
#if defined(SERIAL_PRINT_SENSOR_VALUES)  
  Serial.print("Soil Ch2 Temp = ");
  Serial.println(temperatureString);
  Serial.println("---------------"); 
#endif
  Blynk.virtualWrite(V14,temperatureString);
 }
 else

 {
 #if defined(SERIAL_PRINT_SENSOR_VALUES)  
   Serial.println("Soil Ch2 = Absent");
   Serial.println("---------------"); 
 #endif   
 }

} 

void sleepbaby()
{
  Serial.println("Going to sleep now...");
  Serial.flush(); 
  esp_deep_sleep_start(); 
}

// Wait for inputs from user and indicate him with LED.
void waitforinputs()
{
  delay(3000);
}

void loop()
{
  Blynk.run(); 
  turnONsensor();
  waitforinputs();
  // turnONRelay1();
  // turnONRelay2();
  // UVmeasure();
  // luxMeasure();
  // BatteryMeasure();
  // WeatherBMEMeasure();
  // ECMeasure();
  //PHMeasure();
  // InternalTemperature();
  // Soil_Temperature_1();
  // Soil_Temperature_2();
  // whatIsRain();
  // delay(10);
  // whatIsWind();
  // If Timer has fired
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
  }
  get_time();  
  delay(5000); // Sampling interval
  turnOFFRelay1();
  turnOFFRelay2();
  turnOFFsensor();  
  delay(500);

//if sleep is enabled
  if (sleepvalue<1)
    sleepbaby();
}
