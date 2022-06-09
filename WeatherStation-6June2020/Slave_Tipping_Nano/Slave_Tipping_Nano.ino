// Rainfal measurement device.
// Tipping bucket logic

/* Master can send below commands

   Read Rainfall, rainfall count in last XX minutes
       he resets the count after this

*/
// #include <Debouncer.h>
#include <EEPROM.h>
#include <secTimer.h>
// for communication
#include <Wire.h>
#include <Arduino_JSON.h>

///////////////////////////////For Communication///////////
// Define Slave I2C Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define RAINANSWERSIZE 32
#define WINDANSWERSIZE 32
String rainJsonString;
String windJsonString;
// volatile int flag = 0;
///////////////////////////////For counting and storing
// Interrupt RainPin
int RainPin = 2;
int debounce_duration_ms = 30;

volatile int bucket_count = 0;
int right_bucket = 0;

float bucket_vol = 0.2794;    // Amount of water to tip
float catchment_area = 1.0; // catchment area , open face area
float rainfall_amount; // Final rainfall amount

// start reading from the first byte (address 0) of the EEPROM
int address_left = 0;
int address_right = 1;
int address_wind = 2;

int data_flag = 4;

//wind
int WindPin = 3;
float wind_constant = 2.4;
volatile int wind_count = 0;
float windSpeed = 0;
float windDirection = 0;
String windDirNotation = "";
float adc = 0;

//create a new istance of the library
secTimer myTimer;
unsigned long seconds = 0; //seconds passed after the sketch has started


///////////////////////////////Setup ///////////
void setup()
{
  Serial.begin(9600);

  ///////Communication
  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);
  // Function to run when data requested from master
  Wire.onRequest(requestEvent);
  // Function to run when data received from master
  Wire.onReceive(receiveEvent);

  ////////////Counting
  pinMode(RainPin, INPUT_PULLUP);
  pinMode(WindPin, INPUT_PULLUP);
  EEPROM.write(address_left, 0);
  EEPROM.write(address_right, 0);
  EEPROM.write(address_wind, 0);
  myTimer.startTimer(); //start the timer
  delay(100);
  Serial.println("Rainfall Sensor Start");

  //Interrupt
  attachInterrupt(digitalPinToInterrupt(RainPin), RainCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(WindPin), WindCounter, FALLING);

  //Make payload(empty)
  read_rainfall();
  read_wind();

}

void receiveEvent()
{
  byte x;
  // Read while data received
  while (0 < Wire.available())
  {
    x = Wire.read();
  }
  // Print to Serial Monitor
  //  Serial.print("Receive event:  ");
  //  Serial.println(x);
  //  Serial.println();

  if (x == 0)
  {
    data_flag = 0;
    read_rainfall();
  }
  else if (x == 1)
  {
    data_flag = 1;
    read_wind();
  }
  //  Serial.print("    Data Flag:  ");
  //  Serial.println(data_flag);
}

void requestEvent()
{
  detachInterrupt(digitalPinToInterrupt(RainPin));
  detachInterrupt(digitalPinToInterrupt(WindPin));
  if (data_flag == 0)
  {
    //    Serial.println("Sending Rain...");
    // Setup byte variable in the correct size
    byte response[RAINANSWERSIZE];
    //  Serial.println(flag);
    // Format answer as array
    for (byte i = 0; i < RAINANSWERSIZE; i++)
    {
      response[i] = (byte)rainJsonString.charAt(i);
      //      Serial.print((char)response[i]);
    }
    Wire.write(response, sizeof(response));
    //    Serial.println(response);
  }

  else if (data_flag == 1)
  {
    //    Serial.println("Sending Wind...");
    // Setup byte variable in the correct size
    byte response[WINDANSWERSIZE];
    //  Serial.println(flag);
    // Format answer as array
    for (byte i = 0; i < WINDANSWERSIZE; i++)
    {
      response[i] = (byte)windJsonString.charAt(i);
      //      Serial.print((char)response[i]);
    }
    Wire.write(response, sizeof(response));
    //    Serial.println(response);
  }
  attachInterrupt(digitalPinToInterrupt(RainPin), RainCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(WindPin), WindCounter, FALLING);
  
 //myTimer.sopTimer();;;
  
}

void read_rainfall()
{
  JSONVar myObject;

  bucket_count = EEPROM.read(address_left);
  //  Serial.print("Bucket Count : ");
  //  Serial.println(bucket_count);

  rainfall_amount = (bucket_count * bucket_vol) / catchment_area;
  //  Serial.print("Rainfall Amount : ");
  //  Serial.println(rainfall_amount);

  //  seconds = myTimer.readTimer();
  //  //  Serial.print("Seconds : ");
  //  //  Serial.println(seconds);

  //making string for communication
  myObject["c"] = bucket_count;
  myObject["a"] = String(rainfall_amount, 4);
  //  myObject["s"] = seconds;

  // JSON.stringify(myVar) can be used to convert the json var to a String
  rainJsonString = JSON.stringify(myObject);

  Serial.println(rainJsonString);
  EEPROM.write(address_left, 0);
  
}

void read_wind()
{
  JSONVar myObject;

  wind_count = EEPROM.read(address_wind);
  seconds = myTimer.readTimer();
  Serial.println(seconds);
  if (seconds >= 1)
  {
    windSpeed = (wind_count * wind_constant) / seconds;
    Serial.println(windSpeed);
  }
  else
  {
    windSpeed = 0;
  }
  adc = analogRead(A0);
  windDirection = calculateDirection();
  //  windDirNotation = "NS";

  //making string for communication
  myObject["ws"] = String(windSpeed, 2);
  myObject["d"] = windDirection;   //degree
  //  myObject["n"] = windDirNotation; //notation

  // JSON.stringify(myVar) can be used to convert the json var to a String
  windJsonString = JSON.stringify(myObject);
  Serial.println(windJsonString);
  EEPROM.write(address_wind, 0);
  myTimer.stopTimer();
  myTimer.startTimer();
}

float calculateDirection()
{
  if (adc < 70) return (112.5);
  if (adc < 87) return (67.5);
  if (adc < 100) return (90);
  if (adc < 130) return (157.5);
  if (adc < 190) return (135);
  if (adc < 250) return (202.5);
  if (adc < 300) return (180);
  if (adc < 420) return (22.5);
  if (adc < 470) return (45);
  if (adc < 610) return (247.5);
  if (adc < 650) return (225.5);
  if (adc < 715) return (315);
  if (adc < 800) return (0);
  if (adc < 900) return (292.5);
  if (adc < 950) return (270);
  return (-1); // error, disconnected?}
}

void RainCounter()
{
  Serial.print("in rain:  ");
  bucket_count = EEPROM.read(address_left);
  bucket_count += 1;
  Serial.println(bucket_count);
  EEPROM.write(address_left, bucket_count);
  
}

void WindCounter()
{
  Serial.print("in wind:  ");
  wind_count = EEPROM.read(address_wind);
  wind_count += 1;
  Serial.println(wind_count);
  EEPROM.write(address_wind, wind_count);
  
}

void loop()
{
}
