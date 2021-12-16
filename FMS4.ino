#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
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
String temperature, humidity, pressure, carbondioxide, GPSString, nofix;
String dataMessage;
String val1;
String val2;
String val3;
uint32_t timer = millis();
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
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Feinstaubmesssystem Messdaten \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
}


void loop() {
  /* Get a new sensor event */ 
  sensors_event_t event; 

  if (readPMSdata(&mySerial)) {
    // reading data was successful!

    //store pms data to variables
    val1 = data.particles_10um;
    val2 = data.particles_25um;
    val3 = data.particles_100um;

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
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
      }
          // measure for approximately 2 seconds before printing status
      if (millis() - timer > 7000) {
        timer = millis(); // reset the timer
//        Serial.print("\nTime: ");
//        if (GPS.hour < 10) { Serial.print('0'); }
//        Serial.print(GPS.hour, DEC); Serial.print(':');
//        if (GPS.minute < 10) { Serial.print('0'); }
//        Serial.print(GPS.minute, DEC); Serial.print(':');
//        if (GPS.seconds < 10) { Serial.print('0'); }
//        Serial.print(GPS.seconds, DEC); Serial.print('.');
//        if (GPS.milliseconds < 10) {
//          Serial.print("00");
//        } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
//          Serial.print("0");
//        }
//        Serial.println(GPS.milliseconds);
//        Serial.print("Date: ");
//        Serial.print(GPS.day, DEC); Serial.print('/');
//        Serial.print(GPS.month, DEC); Serial.print("/20");
//        Serial.println(GPS.year, DEC);
//        Serial.print("Fix: "); Serial.print((int)GPS.fix);
//        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        
        dataMessage = "Time: " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + ":" + String(GPS.milliseconds) + "\r\n" + 
                      "Date: " + String(GPS.day) + ":" + String(GPS.month) + ":" + String(GPS.year) + "\r\n" + 
                      "Fix: " + String(GPS.fix) + "\r\n" + 
                      "Quality: " + String(GPS.fixquality) + "\r\n";
        Serial.println(dataMessage);
        appendFile(SD, "/data.txt", dataMessage.c_str());
        if (GPS.fix) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

          dataMessage = "Location: " + String(GPS.latitude) + String(GPS.lat) + ", " + String(GPS.longitude) + String(GPS.lon) + "\r\n" + 
                        "Speed: " + String(GPS.speed) + "\r\n" + 
                        "Angle: " + String(GPS.angle) + "\r\n" + 
                        "Altitude: " + String(GPS.altitude) + "\r\n" + 
                        "Satellites: " + String(GPS.satellites) + "\r\n";
          Serial.println(dataMessage);
          appendFile(SD, "/data.txt", dataMessage.c_str());           
        }
        GPSread = false;
      }
      }

    
    /* Save readings to SD Card*/
     dataMessage = 
                "PM1.0: " + String(val1) + ", " + "PM2.5: " + String(val2) + ", " + "PM10: " + String(val3) + "\r\n" + 
                "Temperature: " + String(temperature) + ", " + "Humidity: " + String(humidity) + ", " + "Pressure: " + String(pressure) + "," + "\r\n" +
                "Acc-X: " + String(event.orientation.x) + ", " + "Acc-Y: " + String(event.orientation.y) + ", " + "Acc-Z: " + String(event.orientation.z) + "," + "\r\n" + 
                "CO2: " + String(carbondioxide) + "\r\n" + "----------------------" + "\r\n";
    Serial.print("Save data: ");
    Serial.println(dataMessage);
    appendFile(SD, "/data.txt", dataMessage.c_str());

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
