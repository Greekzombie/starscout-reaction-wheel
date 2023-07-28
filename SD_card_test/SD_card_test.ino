/*
Have to implement code that can write sensor data to an SD card, preferably in CSV format.

PINS - Equivalent on Arduino Nano

GND  - GND
MISO - D12
CLK  - SCK=D13
MOSI - D11
CS   - SS=D10
3V3  - 3V3

https://toptechboy.com/arduino-lesson-21-log-sensor-data-to-an-sd-card/
https://www.circuitbasics.com/writing-data-to-files-on-an-sd-card-on-arduino/

*/

#include <SD.h>
#include <SPI.h>

File myFile;
char name_file[] = "csv.txt";

const int chipSelect = 10;

char dataStr[200] = "";
char buffer[50];

unsigned long t = 0;
float w_rocket = 2.3789;   
float w_rw = 40.89;  
int signal_motor = 27; 

void setup(){
    Serial.begin(115200);
    initialise_rw_csv_file(chipSelect);

}

void loop(void) {
    t = millis();
    send_rw_data_to_SD_card(t, w_rocket, w_rw, signal_motor);  
    delay(100); 

}

void initialise_rw_csv_file(int chipSelect){
    if (SD.begin(chipSelect)){
        Serial.println("SD card is present & ready");
    } 
    else
    {
        Serial.println("SD card missing or failure");
        while(1); //halt program
    }

    //clear out old data file
    if (SD.exists(name_file)) 
    {
        Serial.println("Removing existing file with same name");
        SD.remove(name_file);
        Serial.println("Done");
    } 

    //write csv headers to file:
    myFile = SD.open(name_file, FILE_WRITE);  
    if (myFile){ // it opened OK
        Serial.println("Writing headers to csv.txt");
        myFile.println("Time,w_rocket,w_rw,signal_motor");
        myFile.close(); 
        Serial.println("Headers written");
    }
    else {
        Serial.print("Error opening ");
        Serial.println(name_file)
    }
        
}

void send_rw_data_to_SD_card(unsigned long t, float w_rocket, float w_rw, int signal_motor){
    dataStr[0] = 0;

    //convert floats to string and assemble c-type char string for writing:
    ltoa(t, buffer, 12); //conver long to charStr
    strcat(dataStr, buffer);//add it onto the end
    strcat(dataStr, ", "); //append the delimeter

    //dtostrf(floatVal, minimum width, precision, character array);
    dtostrf(w_rocket, 10, 4, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
    strcat( dataStr, buffer); //append the converted float
    strcat( dataStr, ", "); //append the delimeter

    dtostrf(w_rw, 10, 4, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
    strcat( dataStr, buffer); //append the converted float
    strcat( dataStr, ", "); //append the delimeter

    ltoa(signal_motor, buffer, 12); //conver int to charStr
    strcat(dataStr, buffer);//add it onto the end
    strcat( dataStr, 0); //terminate correctly 

    // open the file. note that only one file can be open at a time,
    myFile = SD.open(name_file, FILE_WRITE);     
    // if the file opened okay, write to it:
    if (myFile) {
        Serial.print("Writing to ");
        Serial.println(name_file);

        myFile.println(dataStr); 
        myFile.close();
    } 
    else {
        Serial.print("Error opening ");
        Serial.println(name_file);
    }
}