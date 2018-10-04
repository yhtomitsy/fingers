//#include <i2c_t3.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define stabilityThresh 0.7
#define maxAcceleration 5
#define timeDelay 200
#define TCAADDR 0x70
#define BNO_ADDRESS 0x4A            // Device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD
#define QP(n)                       (1.0f / (1 << n))                   // 1 << n ==  2^-n
#define radtodeg                    (180.0f / PI)

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

const uint8_t quat_report        = 0x05;          // defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),
uint8_t cargo[23]; 
uint8_t next_data_seqNum          = 0x10;         // next data sequence number 
uint8_t stat_;                                     // Status (0-3)
float h_est;                                      // heading accurracy estimation

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
  /*if(Serial.available()){
    int c = Serial.available();
    for(uint8_t i = 0; i < c; i++)Serial.read();
  }*/
  getReadings();
  listenForTrig();
  //delay(10);
}

void getReadings(){
    uint32_t t = millis();  
    for (uint8_t i = 2; i < 8; i++) {
        tcaselect(i);
        get_QUAT(i);
        //delay(10);
        /*if (wristIMU.dataAvailable() == true){
            Serial.println(millis() - t);
            quatI[0 + i - 2] = wristIMU.getQuatI();
            quatJ[0 + i - 2] = wristIMU.getQuatJ();
            quatK[0 + i - 2] = wristIMU.getQuatK();
            quatReal[0 + i - 2] = wristIMU.getQuatReal();
        }
        else{
            errorControl(1, errorCount[1]++);
            //Serial.println("error");
        }*/
        
    }
    Serial.println(millis() - t);
    //Serial.println();
}

void get_QUAT(uint8_t i){                                                               
    if (quat_report == 0x08 || quat_report == 0x29){
        Wire.requestFrom(BNO_ADDRESS,21);
        int i=0; 
        while (Wire.available()){
          cargo[i] = Wire.read();
          i++;
        }
    } 
    else{ 
        Wire.requestFrom(BNO_ADDRESS,23);
        int i=0; 
        while (Wire.available()){
            cargo[i] = Wire.read();
            i++;
        }
    }
    if((cargo[9] == quat_report)){          //  && ((cargo[10]) == next_data_seqNum ) check for report and incrementing data seqNum
        //next_data_seqNum = ++cargo[10];                                           // predict next data seqNum              
        stat_ = cargo[11] & 0x03;                                                 // bits 1:0 contain the status (0,1,2,3)  
    
        quatI[i - 2] = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        quatJ[i - 2] = (((int16_t)cargo[16] << 8) | cargo[15] );
        quatK[i - 2] = (((int16_t)cargo[18] << 8) | cargo[17] );
        quatReal[i - 2] = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        quatReal[i - 2] *= QP(14); quatI[i - 2] *= QP(14); quatJ[i - 2] *= QP(14); quatK[i - 2] *= QP(14);                  // apply Q point (quats are already unity vector)

        if (quat_report == 0x05 || quat_report == 0x09 || quat_report == 0x28 ){  // heading accurracy only in some reports available
            h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
            h_est *= QP(12);                                                         // apply Q point 
            h_est *= radtodeg;                                                       // convert to degrees                
        }
    }
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

