#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h> 

File sdFile;
File flashFile;

// Flash chip
LittleFS_QSPIFlash myfs; 

void setup() {
    Serial.begin(9600);
    if (!myfs.begin()) {
        Serial.printf("Error starting %s\n", "QSPI");
    }
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("initialization failed!");
    }
    Serial.println("initialization done.");
    myfs.quickFormat();
    flashFile = myfs.open("testFlash.txt", FILE_WRITE);
    flashFile.println("writing hello world to flash then sd card");
    flashFile.close();


    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.




    
}

void loop() {
    if(Serial.available()) {
        String commandString = Serial.readString();
        if(commandString == "write sd") {
            flashFile = myfs.open("testFlash.txt", FILE_READ);
            sdFile = SD.open("testSD.txt", FILE_WRITE);
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

        

        if(commandString == "print contents"){
            sdFile = SD.open("testSD.txt", FILE_READ);
            while (sdFile.available())
            {
                Serial.print(sdFile.readString());
            }
            Serial.printf("\n");
            sdFile.close();
        }
    }
}