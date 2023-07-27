#include <Arduino.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GPS.h>
// #include <RadioHead.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

Adafruit_ADXL375 adxl375 = Adafruit_ADXL375(1);

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp390;

#define GPSSerial Serial1 //hardware I2C on teensy 4.1 (uc tx 0, uc rx 1, opposite for gps)
Adafruit_GPS GPS(&GPSSerial); 

// accelerometer calibration offsets
double x_offset = 0.39;
double y_offset = 0.20;
double z_offset = 0.52;


void setup(void) {
    Serial.begin(115200);
    Serial.println("Flight Computer Test"); Serial.println("");

    /* ACCELEROMETER -------------------------------------------------------------------------------------- */
    if(!adxl375.begin())
    {
        /* There was a problem detecting the ADXL375 ... check your connections */
        Serial.println("NO ADXL375 DETECTED");
        while(1);
    }
    /* Display some basic information on the sensor */
    adxl375.printSensorDetails();

    /* ALTIMETER -------------------------------------------------------------------------------------- */
    if (!bmp390.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        Serial.println("NO BMP390 DETECTED");
        while (1);
    }
    // Set up oversampling and filter initialization
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_50_HZ);


    /* GPS -------------------------------------------------------------------------------------- */
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC and GGA NMEA codes
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);
    GPSSerial.println(PMTK_Q_RELEASE);

    Serial.println("");
}

void loop(void) {

    // accelerometer prints
    /* Get a new sensor event */
    sensors_event_t event;
    adxl375.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(event.acceleration.x / 9.81 - x_offset); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y / 9.81 - y_offset); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z / 9.81 - z_offset); Serial.print("  ");Serial.println("g ");


    // altimeter prints
    if (! bmp390.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
    Serial.print("Temperature = ");
    Serial.print(bmp390.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp390.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp390.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    // gps prints
    char c = GPS.read();


    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (true) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }

    Serial.println("--------------------------------------------------------------------------------------------------------------");
    delay(500);
}