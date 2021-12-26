#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#include <SD.h>
#include <time.h>

#define GPSSerial Serial2
#define GPSECHO false
#define SD_CS 5       // Define CS pin for the SD card module
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//declarations
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_SCD30  scd30;
Adafruit_INA219 ina219;
String temperature, humidity, pressure, carbondioxide, GPSString, nofix;
String dataMessage;
String val1;
String val2;
String val3;
uint32_t timer = millis();
uint32_t currentFrequency;
bool GPSread = true;

//Software/HardwareSerial configs
SoftwareSerial mySerial(27,26);
Adafruit_GPS GPS(&Serial2);



//struct for PMS measurements
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;                                  //declare PMS struct

void setup() {
  Serial.begin(9600);       
  mySerial.begin(9600);                                   //begin SoftwareSerial (PMS)
  Serial.println("setup");
bme.begin(0x76);                                          //begin bme temperature sensor
  bno.begin();                                            //begin bno acceleration sensor
  scd30.begin();                                          //begin CO2 sensor
  bno.setExtCrystalUse(true);
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  //begin current sensor
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_1A();                     //set to lower range (higher precision on volts and amps)


  //setup SD
  SD.begin(SD_CS);
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.csv");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.csv", "Feinstaubmesssystem Messdaten \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
  dataMessage = "PM1.0, PM2.5, PM10, Temperature, Humidity, Pressure, Acc-X, Acc-Y, Acc-Z, CO2, Time, Date, Fix, Quality, Voltage, Current, Power, Location, Location, Speed (knots), Angle, Altidude, Satellites \r\n"; 
  appendFile(SD, "/data.csv", dataMessage.c_str());
}


void loop() {

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  /* Get a new sensor event */ 
  sensors_event_t event; 

  if (readPMSdata(&mySerial)) {
    // reading data was successful!

    //store pms data to variables
    val1 = data.particles_10um;
    val2 = data.particles_25um;
    val3 = data.particles_100um;

    //read current sensor
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

//    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
//    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
//    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
//    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
//    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
//    Serial.println("");

    //read all the other sensors
    bno.getEvent(&event);
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }
    carbondioxide = scd30.CO2;

    
    
    while(GPSread)
    {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
      }
          // measure for approximately 2 seconds before printing status
      if (millis() - timer > 7000) {
        timer = millis(); // reset the timer

        //"PM1.0, PM2.5, PM10, Temperature, Humidity, Pressure, Acc-X, Acc-Y, Acc-Z, CO2, Time, Date, Fix, Quality, Location, Location, Speed (knots), Angle, Altidude, Satellites \r\n"; 
        dataMessage = String(val1) + ", " + String(val2) + ", " + String(val3) + ", " + String(temperature) + ", " + String(humidity) + ", " + String(pressure) + ", " + 
        String(event.orientation.x) + ", " + String(event.orientation.y) + ", " + String(event.orientation.z) + ", " + String(carbondioxide) + ", " + 
        String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + ":" + String(GPS.milliseconds) + ", " + String(GPS.day) + ":" + String(GPS.month) + ":" + String(GPS.year) + ", " + 
        String(GPS.fix) + ", " + String(GPS.fixquality) + ", " + String(loadvoltage) + ", " + String(current_mA) + ", " + String(power_mW);
  
        if (GPS.fix) {


          dataMessage = dataMessage + ", " + String(GPS.latitude) + String(GPS.lat) + ", " + String(GPS.longitude) + String(GPS.lon) + ", " +
                        String(GPS.speed) + ", " +
                        String(GPS.angle) + ", " + 
                        String(GPS.altitude) + ", " + 
                        String(GPS.satellites);
                   
        }
        dataMessage = dataMessage + "\r\n";
        Serial.println(dataMessage);
        appendFile(SD, "/data.csv", dataMessage.c_str());  
        GPSread = false;
      }
    }

    Serial.println("");
  }
  GPSread = true;
  delay(5000);
}

// read data from PMS particle sensor
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");    
    ESP.restart();
    
    return false;
  }
  // success!
  return true;
}


// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
