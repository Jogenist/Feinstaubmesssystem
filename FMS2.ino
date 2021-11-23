#include "PMS.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SCD30.h>

Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_SCD30  scd30;
String temperature, humidity, pressure;
float CarbonDioxideConcentration;

PMS pms(Serial2);
PMS::DATA data;

String val1;
String val2;
String val3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  bme.begin(0x76);
  Serial.println("setup");
  
  bno.begin();
  bno.setExtCrystalUse(true);

  scd30.begin();
}

void loop() {
  /* Get a new sensor event */ 
  sensors_event_t event; 

  if (pms.read(data))
  {
    val1 = data.PM_AE_UG_1_0;
    val2 = data.PM_AE_UG_2_5;
    val3 = data.PM_AE_UG_10_0;
    
    bno.getEvent(&event);
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    
    
    Serial.println("Feinstaubmesssystem");
    Serial.print("\t");
    
    /* Display the data of PMS5003*/
    Serial.println("PMS5003");
    Serial.println("PM1.0 :" + val1 + "(ug/m3)");
    Serial.println("PM2.5 :" + val2 + "(ug/m3)");
    Serial.println("PM10  :" + val3 + "(ug/m3)");
    
    /* Display the data of BME280*/
    Serial.println("BME280");
    Serial.println("Temperature :" + temperature + " C°");
    Serial.println("Humidity :" + humidity + " %");
    Serial.println("Pressure :" + pressure + " hPa");

    /* Display the floating point data of BNO055*/
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.println(event.orientation.z, 4);

    /*Display the Carbondioxide data of SCD30*/
    if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }
    Serial.print("CO2: ");
    Serial.print(scd30.CO2, 3);
    Serial.println(" ppm");
    Serial.println("");


    Serial.println("");
  }
  delay(1000);
}
