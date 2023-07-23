#include <Arduino.h>
#include <LittleFS.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <SD.h>

// Flash chip
LittleFS_QSPIFlash myfs; 
File flashFile;
File sdFile;
void writeFlash(float temp, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ);
// Accel & Gyro
Adafruit_LSM6DS3TRC lsm6ds3trc;


void setup() {
    Serial.begin(9600);
    if (!myfs.begin()) {
        Serial.printf("Error starting %s\n", "QSPI");
    }
    

    lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);

    lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
    delay(1000);
}

int n = 0;
void loop() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    

    
    if (n <10){
        writeFlash(temp.temperature, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
        delay(250);
        Serial.println(temp.temperature);
        Serial.println(accel.acceleration.x);
        Serial.println(accel.acceleration.y);
        Serial.println(accel.acceleration.z);
        Serial.println(gyro.gyro.x);
        Serial.println(gyro.gyro.y);
        Serial.println(gyro.gyro.z);
        Serial.println("\n\n\n");
        n++;
    }

    if(Serial.available()) {
        String commandString = Serial.readString();

        if(commandString == "init sd") {
            if (!SD.begin(BUILTIN_SDCARD)) {
                Serial.println("initialization failed!");
            }
        }

        if(commandString == "write sd") {
            flashFile = myfs.open("flightData.csv", FILE_READ);
            sdFile = SD.open("flightData.csv", FILE_WRITE);
            if(sdFile) {
                while(flashFile.available()){
                    sdFile.println(flashFile.readString());
                }
                sdFile.close();
                flashFile.close();
            }
        }

        if (commandString == "clear sd") {
            SD.format();
        }

        if (commandString == "clear flash") {
            myfs.quickFormat();
        }

        if(commandString == "print contents"){
            sdFile = SD.open("flightData.csv", FILE_READ);
            while (sdFile.available())
            {
                Serial.print(sdFile.readString());
            }
            Serial.printf("\n");
            sdFile.close();
        }
    }

}

void writeFlash(float temp, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
    String dataString = "";
    String delimeter = ",";

    dataString += String(temp); dataString += String(delimeter);

    dataString += String(accelX); dataString += String(delimeter);
    dataString += String(accelY); dataString += String(delimeter);
    dataString += String(accelZ); dataString += String(delimeter);

    dataString += String(gyroX);  dataString += String(delimeter);
    dataString += String(gyroY);  dataString += String(delimeter);
    dataString += String(gyroZ);  dataString += String(delimeter);

    flashFile = myfs.open("flightData.csv", FILE_WRITE);
    if(flashFile) {
        flashFile.println(dataString);
        Serial.println(dataString);
        flashFile.flush();
    }
    flashFile.close();
}

