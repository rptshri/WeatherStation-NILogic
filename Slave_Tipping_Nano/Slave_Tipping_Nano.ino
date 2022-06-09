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
String windDirection = "";
String windDirNotation = "";
float adc = 0;

//create a new istance of the library
secTimer myTimer;
unsigned long seconds = 0; //seconds passed after the sketch has started

JSONVar myObjectRain;
JSONVar myObjectWind;


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
//   Serial.println("In receive event");
  byte x;
  // Read while data received
  while (0 < Wire.available())
  {
    x = Wire.read();
  }
  // Print to Serial Monitor
  Serial.print("Receive event:  ");
  Serial.println(x);
  Serial.println();

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
  //  read_rainfall();
  //  read_wind();
  Serial.println("Request event Func: IN ");
  detachInterrupt(digitalPinToInterrupt(RainPin));
  detachInterrupt(digitalPinToInterrupt(WindPin));
  if (data_flag == 0)
  {
    Serial.print("Rain payload sending : ");
    Serial.println(rainJsonString);
    //    Serial.println("Sending Rain...");
    // Setup byte variable in the correct size
    byte response[RAINANSWERSIZE];
    //  Serial.println(flag);
    // Format answer as array
    for (byte i = 0; i < RAINANSWERSIZE; i++)
    {
      response[i] = (byte)rainJsonString.charAt(i);
      // Serial.print((char)response[i]);
    }

    Wire.write(response, sizeof(response));
    //    Serial.print("Test rain = ");
    // Serial.println(response);
    Serial.println("Request event Func: OUT rain");
  }

  else if (data_flag == 1)
  {
    Serial.print("Wind payload sending : ");
    Serial.println(windJsonString);
    //    Serial.println("Sending Wind...");
    // Setup byte variable in the correct size
    byte response[WINDANSWERSIZE];
    //  Serial.println(flag);
    // Format answer as array
    for (byte i = 0; i < WINDANSWERSIZE; i++)
    {
      response[i] = (byte)windJsonString.charAt(i);
      // Serial.print((char)response[i]);
    }
    Wire.write(response, sizeof(response));
    //    Serial.print("Wind Test =");
    //    Serial.println(response);
//    Serial.println("Request event Func: OUT wind");

  }
  attachInterrupt(digitalPinToInterrupt(RainPin), RainCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(WindPin), WindCounter, FALLING);

}

void read_rainfall()
{
 Serial.println("In read rain");
  bucket_count = EEPROM.read(address_left);
  //  Serial.print("Bucket Count : ");
  //  Serial.println(bucket_count);

  rainfall_amount = (bucket_count * bucket_vol) / catchment_area;
  //  Serial.print("Rainfall Amount : ");
  //  Serial.println(rainfall_amount);

  //making string for communication
  myObjectRain["c"] = bucket_count;
  myObjectRain["a"] = rainfall_amount;
  myObjectRain["s"] = seconds;

  // JSON.stringify(myVar) can be used to convert the json var to a String
  rainJsonString = JSON.stringify(myObjectRain);

  Serial.println(rainJsonString);
  EEPROM.write(address_left, 0);

}

void read_wind()
{
  Serial.println("In read wind");
  wind_count = EEPROM.read(address_wind);
  seconds = myTimer.readTimer();
//  Serial.println(seconds);
  if (seconds >= 1)
  {
    windSpeed = (wind_count * wind_constant) / seconds;
//    Serial.println(windSpeed);
  }
  else
  {
    windSpeed = 0;
  }
  adc = analogRead(A0);
  windDirection = calculateDirection();
  //  windDirNotation = "NS";
//  Serial.println(windDirection);

  //making string for communication
  myObjectWind["ws"] = windSpeed;
  myObjectWind["d"] = windDirection;   //degree
  //  myObject["n"] = windDirNotation; //notation

  // JSON.stringify(myVar) can be used to convert the json var to a String
  windJsonString = JSON.stringify(myObjectWind);
  Serial.println(windJsonString);
  EEPROM.write(address_wind, 0);
  myTimer.stopTimer();
  myTimer.startTimer();
}

String calculateDirection()
{
  
//  Serial.println("Calculating Direction...");
//  if (adc < 70) return (112.5);
//  if (adc < 87) return (67.5);
//  if (adc < 100) return (90);
//  if (adc < 130) return (157.5);
//  if (adc < 190) return (135);
//  if (adc < 250) return (202.5);
//  if (adc < 300) return (180);
//  if (adc < 420) return (22.5);
//  if (adc < 470) return (45);
//  if (adc < 610) return (247.5);
//  if (adc < 650) return (225.5);
//  if (adc < 715) return (315);
//  if (adc < 800) return (0);
//  if (adc < 900) return (292.5);
//  if (adc < 950) return (270);
//  return (-1); // error, disconnected?}
//
//  Serial.println("Calculating Direction...");
  if (adc < 70) return ("N");
  if (adc < 87) return ("ENE");
  if (adc < 100) return ("E");
  if (adc < 130) return ("SSE");
  if (adc < 190) return ("SE");
  if (adc < 250) return ("SSW");
  if (adc < 300) return ("S");
  if (adc < 420) return ("NNE");
  if (adc < 470) return ("NE");
  if (adc < 610) return ("WSW");
  if (adc < 650) return ("SW");
  if (adc < 715) return ("NW");
  if (adc < 800) return ("N");
  if (adc < 900) return ("WNW");
  if (adc < 950) return ("W");
  return ("NA"); // error, disconnected?}
  
}

void RainCounter()
{
  Serial.print("in rain int:  ");
  bucket_count = EEPROM.read(address_left);
  bucket_count += 1;
  Serial.println(bucket_count);
  EEPROM.write(address_left, bucket_count);

}

void WindCounter()
{
  Serial.print("in wind int:  ");
  wind_count = EEPROM.read(address_wind);
  wind_count += 1;
  Serial.println(wind_count);
  EEPROM.write(address_wind, wind_count);

}

void loop()
{
}
