/**
 * Copyright (c) 2013 Michael D. Rayman. All rights reserved.
 *
 * This file is part of XBee-Home-Automation and is meant to be used along
 * with the associated open hardware designs for the node modules.
 *
 * XBee-Home-Automation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Home-Automation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Home-Automation.  If not, see <http://www.gnu.org/licenses/>.
 */

///////////////////////////////////////////////////////////////////////////////
//
// This file's code was leveraged from the common base arduino_xbee_endpoint_base.ino
// and then modified for the specific hardware configuration of this node as shown
// below.  This node is a basic water level and water presence detection endpoint.
// It monitors water level and presence of water with a generic LM339 based circuit
// that provides both an analog output and a digital output (1= no detect, 0= detect)
// where the analog output is connected to Arduino UNO R3 analog input A3 and the
// digital output is connected to Arduino UNO R3 digital input D6.
//
// Common Hardware Base:
//   * Arduino UNO R3 with ATMega328P
//   * XBee Series 2 XB-24-Z7WIT-XXX
//   * XBee Shield for Arduino UNO R3
//   * AD7415 I2C Temperature Sensor with AS pin connected to GND
//
// Additional Hardware Pieces Used On This Node:
//   * DHT11 I2C Humidity/Temp sensor
//   * Water level and detect sensor (analog and digital, resp.)
//
///////////////////////////////////////////////////////////////////////////////


// RESOURCES
//
// Pressure processing : http://www.aqua-calc.com/convert/pressure/kilopascal-to-inch-of-mercury
//
// BMP085: Temp in 0.1C, so divide by 10 to get temp in C (do fractional divide to preserve fraction)
//         Pressure is in Pascals, so divide by 1000 and convert to inHg using link above
//         Altitude is in centimeters and must be provided with a reference when library is initialized, use 330.201 meters
//         XBEE: Temp is * 10 to yield full integer with lowest two digits representing two digit fraction
//         XBEE: Pressure is / 10 to yield full integer with lowest two digits representing 100's and 10's of Pascals and upper digits kPa
//
// DHT11:  Temp in C represented as temp in C with fraction.  Humidity in % with fraction.
//         XBEE: Temp is * 100 to yield full integer with lowest two digits representing two digit fraction
//         XBEE: Humidity is * 100 to yield full integer with lowest two digits representing two digit fraction
//
// AD7415: Temp in C represented as temp in C with fraction.
//         XBEE: Temp is * 100 to yield full integer with lowest two digits representing two digit fraction
//
// Water:  Water Level is 0-1023 value, 0 at low level and 1023 at highest level.
//         Water Detect is 0 or 1, 0 is no detect, 1 is detected.
//         XBEE: Water Level is represented as-is: integer 0-1023
//         XBEE: Water Detect is represented as 0 or 1
//
// Motion: PIR sensors (right-hand side looking out from unit, left-hand side looking out from unit)
//         Sensors feed analog inputs, yielding 0 if no motion detected and ~675 if motion detected.
//         XBEE: Motion detect level sent as 0 for no motion detect or 1 for motion detect.
//


// INCLUDES
#include <XBee.h>
#include <Wire.h>
#include <BMP085.h>
#include <dht.h>
#include <AD741X.h>


// GLOBAL CONTROL DECLARATIONS:
#define XBEE_ENDPOINT_BAUD_RATE       115200
#define SERIAL_DEBUG_ON               0
#define SERIAL_DEBUG_VERBOSE          0
#define ENABLE_SENSOR_SLEEP_MODE      0

// PIN VARIABLE DECLARATIONS:
#define DHT11_INT_PIN                 2
#define DHT11_EXT_PIN                 3
#define DO_BEACON_LED                 7
#define DO_SENSOR_VCC                 8
#define DO_XBEE_SLEEP_N               9
#define DO_ARDUINO_UNO_R3_LED         13

#define AIN_MOTION_DETECT_LEFT        A0
#define AIN_MOTION_DETECT_RIGHT       A1
#define AIN_WATER_LEVEL_ANALOG        A3
#define DI_WATER_LEVEL_MONITOR        6

#define XBEE_TX_STRING_MAX_BYTES      50

#define PIR_MOTION_DETECT_THRESHOLD   600


// VARIABLE DECLARATIONS
XBee xbee;
XBeeAddress64 XBEE_COORDINATOR_ADDR64 = XBeeAddress64(0x00000000, 0x00000000);  // Specify the address of the remote XBee (this is the SH + SL)
char XBee_TX_String[XBEE_TX_STRING_MAX_BYTES] = "";
String XBee_Data_String;
BMP085 Sensor_BMP085 = BMP085();      // Digital Pressure Sensor 
long BMP085_Temp = 0, BMP085_Pressure = 0;
dht Sensor_DHT11;
int DHT11_Status = 0;
long DHT11_Humidity = 0, DHT11_Temp = 0;
AD741X Sensor_AD7415 = AD741X(7415, 0, AD741X_AS_GND);  //AD741X_AS_FLOAT
long AD7415_temp = 0;
int AD7415_Status = 0;
int Water_Level_Tripped = 0, Water_Level = 0;
int Motion_Detect_Right = 0, Motion_Detect_Left = 0;


// ----- SETUP -----
// the setup routine runs once when you press reset:
void setup() {                
  //if( SERIAL_DEBUG_ON == 1 )
    Serial.begin(XBEE_ENDPOINT_BAUD_RATE);

  // SETUP AND INITIALIZE GPIO:
  //if( SERIAL_DEBUG_ON == 1 )
    Serial.println("SETUP: Setting up and initializing Pins...");
  pinMode(DO_SENSOR_VCC, OUTPUT);
  digitalWrite(DO_SENSOR_VCC, HIGH);
  pinMode(DO_ARDUINO_UNO_R3_LED, OUTPUT);
  digitalWrite(DO_ARDUINO_UNO_R3_LED, LOW);  // disable standard Arduino UNO R3 LED
  pinMode(DO_BEACON_LED, OUTPUT);
  digitalWrite(DO_BEACON_LED, HIGH);
  pinMode(DO_XBEE_SLEEP_N, OUTPUT);

  pinMode(DI_WATER_LEVEL_MONITOR, INPUT);

  // SETUP AND INITIALIZE SENSORS
  //if( SERIAL_DEBUG_ON == 1 )
    Serial.println("Setting up and initializing Sensors...");
  Wire.begin();
  delay(1000);

/*
  // BMP085 - Digital Pressure Sensor
  Sensor_BMP085.init(MODE_ULTRA_HIGHRES, 33020, true);  // 330.20 meters, true = using meter units
                  // this initialization is useful if current altitude is known,
                  // pressure will be calculated based on TruePressure and known altitude.
*/

  AD7415_Status = Sensor_AD7415.init(); 


  // SETUP AND INITIALIZE XBEE RADIO:
  //if( SERIAL_DEBUG_ON == 1)
    Serial.println("Setting up and initializing XBee Radio...");
  xbee = XBee();

  XBee_Data_String = String("");

  //if( SERIAL_DEBUG_ON == 1 )
  {
    Serial.println("SETUP COMPLETE!");
    Serial.println("");
  }
}
// -----------------


// ----- LOOP -----
// the loop routine runs over and over again forever:
void loop() {

  if( SERIAL_DEBUG_ON == 0)
  {
    // XBEE INACTIVE
    digitalWrite(DO_BEACON_LED, LOW);
    if( ENABLE_SENSOR_SLEEP_MODE == 1 )
    {
      digitalWrite(DO_XBEE_SLEEP_N, HIGH);
      digitalWrite(DO_SENSOR_VCC, LOW);
    }
    delay(5000);
  }
  else
    Serial.println("Sleeping XBee, Sensors, and Arduino...");

  // XBEE ACTIVE - TRANSMIT
  digitalWrite(DO_BEACON_LED, HIGH);    // Enable Beacon LED
  digitalWrite(DO_XBEE_SLEEP_N, LOW);   // Pull XBee radio out of Pin Hibernate
  digitalWrite(DO_SENSOR_VCC, HIGH);    // Apply VCC to all Sensors
  if( SERIAL_DEBUG_ON == 1)
    Serial.println("Waking up!");
  else
   delay(1000);


  // Gather Sensor Data
  if( SERIAL_DEBUG_ON == 1 )
    Serial.println("Gathering Sensor Data:");

/*
  // --> IIC BMP085 temp/pressure
  Get_BMP085_Data(&BMP085_Temp, &BMP085_Pressure);
  XBee_Data_String = "BMP085|temp:";
  XBee_Data_String = XBee_Data_String + BMP085_Temp;
  XBee_Data_String = XBee_Data_String + "|pressure:";
  XBee_Data_String = XBee_Data_String + BMP085_Pressure;
  //if( SERIAL_DEBUG_ON == 1 )
  //  Serial.println(XBee_Data_String);
  //else
    SendStringToXBee(&XBee_Data_String);
*/

  // --> 1-WIRE DHT11 temp/humidity
  Get_DHT11_Data(DHT11_INT_PIN, &DHT11_Status, &DHT11_Humidity, &DHT11_Temp);
  XBee_Data_String = "DHT11|temp:";
  XBee_Data_String = XBee_Data_String + DHT11_Temp;
  XBee_Data_String = XBee_Data_String + "|humidity:";
  XBee_Data_String = XBee_Data_String + DHT11_Humidity;
  //if( SERIAL_DEBUG_ON == 1 )
  //  Serial.println(XBee_Data_String);
  //else
    SendStringToXBee(&XBee_Data_String);

  // --> IIC AD7415 temp
  Get_AD7415_Temp(&AD7415_temp);
  XBee_Data_String = "AD7415|temp:";
  XBee_Data_String = XBee_Data_String + AD7415_temp;
  //if( SERIAL_DEBUG_ON == 1 )
  //  Serial.println(XBee_Data_String);
  //else
    SendStringToXBee(&XBee_Data_String);

  // --> DIN Water Level
  Get_Water_Level(&Water_Level_Tripped, &Water_Level);
  XBee_Data_String = "water|level:";
  XBee_Data_String = XBee_Data_String + Water_Level;
  XBee_Data_String = XBee_Data_String + "|detect:";
  XBee_Data_String = XBee_Data_String + Water_Level_Tripped;
  //if( SERIAL_DEBUG_ON == 1 )
  //  Serial.println(XBee_Data_String);
  //else
    SendStringToXBee(&XBee_Data_String);

/*
  // --> DIN PIR Motion Sensor(s)
  Get_Motion_Detect(&Motion_Detect_Right, &Motion_Detect_Left);
  XBee_Data_String = "motion|rightdetect:";
  XBee_Data_String = XBee_Data_String + Motion_Detect_Right;
  XBee_Data_String = XBee_Data_String + "|leftdetect:";
  XBee_Data_String = XBee_Data_String + Motion_Detect_Left;
  //if( SERIAL_DEBUG_ON == 1 )
  //  Serial.println(XBee_Data_String);
  //else
    SendStringToXBee(&XBee_Data_String);
*/

  // Delay to ensure XBee transmissions are sent before looping to power down XBee Radio:
  if( SERIAL_DEBUG_ON == 0 )
    delay(4000);
}
// ----------------


void Get_BMP085_Data(long * Temp, long * Pressure)
{
  Sensor_BMP085.init(MODE_ULTRA_HIGHRES, 33020, true);  // 330.20 meters, true = using meter units
                  // this initialization is useful if current altitude is known,
                  // pressure will be calculated based on TruePressure and known altitude.

  Sensor_BMP085.getTemperature(Temp);
  Sensor_BMP085.getPressure(Pressure);
  *Temp = *Temp * 10;
  *Pressure = *Pressure / 10;

  if( SERIAL_DEBUG_ON == 1 && SERIAL_DEBUG_VERBOSE == 1 )
  {
    Serial.print("BMP085: \t");
    Serial.print("Temp(C): ");
    Serial.print((*Temp)/10.0);
    Serial.print(",\t");
    Serial.print("Pressure(kPa): ");
    Serial.println((*Pressure)/1000.0);
  }
}


void Get_DHT11_Data(int DHT_pin, int *Status, long *Humidity, long *Temp)
{
  int chk = Sensor_DHT11.read11(DHT11_INT_PIN);

  if( SERIAL_DEBUG_ON == 1 && SERIAL_DEBUG_VERBOSE == 1 )
  {
    Serial.print("DHT11: \t");

    switch (chk)
    {
      case DHTLIB_OK:  
        Serial.print("OK,\t"); 
        break;
      case DHTLIB_ERROR_CHECKSUM: 
        Serial.print("Checksum error,\t"); 
        break;
      case DHTLIB_ERROR_TIMEOUT: 
        Serial.print("Time out error,\t"); 
        break;
      default: 
        Serial.print("Unknown error,\t"); 
        break;
    }

    Serial.print("Temp(C): ");
    Serial.print(Sensor_DHT11.temperature,1);
    Serial.print(",\t");
    Serial.print("Humidity(%): ");
    Serial.println(Sensor_DHT11.humidity,1);
  }

  *Status = chk;
  *Humidity = (long)(Sensor_DHT11.humidity * 100);
  *Temp = (long)(Sensor_DHT11.temperature * 100);
}


void Get_AD7415_Temp(long *Temp)
{
  AD7415_Status = Sensor_AD7415.init(); 

  *Temp = (long)(Sensor_AD7415.GetTemp() * 100);

  if( SERIAL_DEBUG_ON == 1 && SERIAL_DEBUG_VERBOSE == 1 )
  {
    Serial.print("AD7415: \t");
    Serial.print("Temp(C): ");
    Serial.println(*Temp,1);
  }
}


void Get_Water_Level(int *SensorTripped, int *WaterLevel)
{
  *SensorTripped = digitalRead(DI_WATER_LEVEL_MONITOR);
  // Reverse the tripped value since level detection circuit pulls to Logic Low when water is detected:
  if( *SensorTripped >= 1 )
    *SensorTripped = 0;
  else
    *SensorTripped = 1;

  // read the input on analog pin 2:
  *WaterLevel = analogRead(AIN_WATER_LEVEL_ANALOG);

  if( SERIAL_DEBUG_ON == 1 && SERIAL_DEBUG_VERBOSE == 1 )
  {
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = *WaterLevel * (5.0 / 1023.0);
    // print out the value you read:
    Serial.print("Water Level Monitor: \t");
    Serial.print("Tripped: ");
    Serial.print(*SensorTripped,1);
    Serial.print(",\t");
    Serial.print("Level: ");
    Serial.print(*WaterLevel);
    Serial.print(",\t");
    Serial.print("As Voltage: ");
    Serial.println(voltage);
  }
}

void Get_Motion_Detect(int * Detect_Right, int * Detect_Left)
{
  // Read PIR sensors
  *Detect_Right = analogRead(AIN_MOTION_DETECT_RIGHT);
  *Detect_Left = analogRead(AIN_MOTION_DETECT_LEFT);

  if( *Detect_Right >= PIR_MOTION_DETECT_THRESHOLD )
    *Detect_Right = 1;
  else
    *Detect_Right = 0;

  if( *Detect_Left >= PIR_MOTION_DETECT_THRESHOLD )
    *Detect_Left = 1;
  else
    *Detect_Left = 0;

  if( SERIAL_DEBUG_ON == 1 && SERIAL_DEBUG_VERBOSE == 1 )
  {
    Serial.print("MOTION: \t");
    Serial.print("Detect Right: ");
    Serial.print(*Detect_Right,1);
    Serial.print(",\t");
    Serial.print("Detect Left: ");
    Serial.println(*Detect_Left,1);
  }
}

void SendStringToXBee(String * Xbee_String)
{
  Xbee_String->toCharArray(XBee_TX_String, XBEE_TX_STRING_MAX_BYTES);
  if( SERIAL_DEBUG_ON == 0 )
  {
    ZBTxRequest zbTx = ZBTxRequest(XBEE_COORDINATOR_ADDR64, (uint8_t*)(XBee_TX_String), strlen(XBee_TX_String));
    xbee.send(zbTx);
  }
  else
  {
    Serial.println(*Xbee_String);
  }

  delay(100); // Allow time for XBee to finish transmission of the packet
}
