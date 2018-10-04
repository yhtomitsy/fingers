//#include <i2c_t3.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define stabilityThresh 0.7
#define maxAcceleration 5
#define timeDelay 200
#define TCAADDR 0x70

BNO080 palmIMU;
BNO080 wristIMU;

boolean oriented = true;     // the hand has not been oriented
uint8_t enableIMU[] = {2, 3}; // enable pins
uint8_t tries = 0;            // holds number of initialization trials made

float quatI[6] = {0};
float quatJ[6] = {0};
float quatK[6] = {0};
float quatReal[6] = {0};

// first value is the current  acceleration, second value is the previous acceleration
float x[2] = {0};
float y[2] = {0};
float z[2] = {0};

// debug data
int counter = 0;
int errorCount[2] = {0};

// pitch and roll vars
float roll[6] = {0};
float yaw[6] = {0};
float heading[2] = {0};

void tcaselect(uint8_t i) {
    //if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void setup() {
    Serial.begin(2000000);
    Wire.begin();
    Wire.setClock(400000); //Increase I2C data rate to 400kHz
    /*Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);*/
    
    pinMode(13, OUTPUT);
    
    //Serial.print("elyon"); 
    delay(1000);    
    for (uint8_t i = 2; i < 8; i++) { 
        tcaselect(i);
        delay(100);
        initIMU(1);
        //Serial.print(i);
    }
    initIndicate();
}



void loop() {
  if(Serial.available()){
    int c = Serial.available();
    for(uint8_t i = 0; i < c; i++)Serial.read();
  }
  getReadings();
  listenForTrig();
  //delay(3);
}

void getReadings(){
    //uint32_t t = millis();  
    for (uint8_t i = 2; i < 8; i++) {
        tcaselect(i);
        if (wristIMU.dataAvailable() == true){
            quatI[0 + i - 2] = wristIMU.getQuatI();
            quatJ[0 + i - 2] = wristIMU.getQuatJ();
            quatK[0 + i - 2] = wristIMU.getQuatK();
            quatReal[0 + i - 2] = wristIMU.getQuatReal();
        }
        else{
            errorControl(1, errorCount[1]++);
            //Serial.println("error");
        } 
    }
    //Serial.println(millis() - t);
    //Serial.println();
}

void listenForTrig(){
    //int c = Serial.available();
    //if( c > 0){
        //char c = Serial.read();
        //if(c == '*'){
            /*String s = "$," +String(quatReal[0], 2) + "," + String(quatI[0], 2) + "," + String(quatJ[0], 2) + "," + String(quatK[0], 2);
            for(uint8_t i = 0; i < 6; i++){
                  s += "," + String(roll[i], 2); 
            }
            for(uint8_t i = 0; i < 6; i++){
                  s += "," + String(yaw[i], 2);
            }
            s += "\r";*/
            /*String s = "$," +String(quatReal[0], 2) + "," + String(quatI[0], 2) + "," + String(quatJ[0], 2) + "," + String(quatK[0], 2)
                       + "," + String(quatReal[1], 2) + "," + String(quatI[1], 2) + "," + String(quatJ[1], 2) + "," + String(quatK[1], 2)
                       + "," + String(quatReal[2], 2) + "," + String(quatI[2], 2) + "," + String(quatJ[2], 2) + "," + String(quatK[2], 2)
                       + "," + String(quatReal[3], 2) + "," + String(quatI[3], 2) + "," + String(quatJ[3], 2) + "," + String(quatK[3], 2)
                       + "," + String(quatReal[4], 2) + "," + String(quatI[4], 2) + "," + String(quatJ[4], 2) + "," + String(quatK[4], 2)
                       + "," + String(quatReal[5], 2) + "," + String(quatI[5], 2) + "," + String(quatJ[5], 2) + "," + String(quatK[5], 2) + "\r\n";*/
            String s = String(quatReal[0], 2) + "," + String(quatI[0], 2) + "," + String(quatJ[0], 2) + "," + String(quatK[0], 2)
                       + "," + String(quatReal[1], 2) + "," + String(quatI[1], 2) + "," + String(quatJ[1], 2) + "," + String(quatK[1], 2)
                       + "," + String(quatReal[2], 2) + "," + String(quatI[2], 2) + "," + String(quatJ[2], 2) + "," + String(quatK[2], 2)
                       + "," + String(quatReal[3], 2) + "," + String(quatI[3], 2) + "," + String(quatJ[3], 2) + "," + String(quatK[3], 2)
                       + "," + String(quatReal[4], 2) + "," + String(quatI[4], 2) + "," + String(quatJ[4], 2) + "," + String(quatK[4], 2)
                       + "," + String(quatReal[5], 2) + "," + String(quatI[5], 2) + "," + String(quatJ[5], 2) + "," + String(quatK[5], 2) + "\r\n";
            Serial.print(s);
        //}
        //for(uint8_t i = 0; i < c; i++)Serial.read();
     //}
}
void errorControl(uint8_t i, uint8_t counter){
    if(counter > 5){
       initIMU(i);
       errorCount[i] = 0;
    }
}
void initIMU(uint8_t i){
    switch(i){
        case 1:
            while(!wristIMU.begin(0x4A, Wire)){
              //Serial.print("."); 
              delay(500);  
            }
            //Serial.println("Successful wrist IMU");
            wristIMU.enableRotationVector(1); //Send data update every 50ms 
            //wristIMU.enableRealAccelerometer(5); //Send data update every 50ms
            //wristIMU.enableMagnetometer(5);
            break;
        case 0:
            while(!palmIMU.begin(0x4B, Wire)){
              //Serial.print("."); 
              delay(500);  
            }
            //Serial.println("Successful palm IMU");
            palmIMU.enableRotationVector(1); //Send data update every 50ms 
            //palmIMU.enableRealAccelerometer(5); //Send data update every 50ms
            //palmIMU.enableMagnetometer(5); 
            break;
        default:
            break;
    }
}

void initIndicate(){
   for(uint8_t i = 0; i < 10; i++){
        digitalWrite(13, HIGH);
        delay(50);
        digitalWrite(13, LOW);
        delay(50);
    }
}

