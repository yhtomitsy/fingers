/* BNO080 Rock bottom code for Arduino Atmega 328P 16Mhz (Arduino Nano etc) and Cortex M0 (Simblee)
 by: JP Schramel
 date: January 22, 2018
 
 Disclaimer:
 Freeware:  The code is provided "AS IS",without warranty of any kind.
 
 # Demonstrates basic  functionality 9DOF fused quaternions up to 400Hz data rate.
 # Stores calibration data in flash.
 # Tares the quaternion output to (1,0,0,0) in any arbitrary position
 # Enables control about dynamic automatic calibration
 # heading accurracy estimation
 
 
 It uses wire.h library @ 400kHz for communication.
 Two buttons store dynamic calibration  data in flash and for tare function:
 
 Other feature reports than quaternions (gravities, linear acceleration etc) can be implemented similarly (see HillcrestLab data sheets and application notes)

 Data  are available via Serial.print. Note that this may take significant time and limits max data rate.
 
 Hardware requirements:
 
 ATmega 328p based boards (Nano..., 16MHz) 
 BNO080 breakout board e.g. Hillcrest FSM300
 Needs a 3V3 supply and connect I2C Bus via a level converter 5V --> 3V3!!!  SDA = A4; SCL = A5. 
 
 LED via 470 Ohm --> pin 5 (pin 13), to ground. 
 Button pin A3 to GND => Tare function
 Button pin 6 to GND  =  Calibration function
  
*/

/*************************************
                INCLUDES
 *************************************/
#include <Wire.h> 

/*************************************
                DEFINITIONS
 *************************************/



//#define Simblee
#define Nano

#define Led                       10              // Led  (Arduino nano = 13) to measure loop duration (10)
#define btn_TARE                  A3              // input pin for button TARE
#define btn_CAL                   6               // input pin for button TARE
int plot_interval                 = 1000;          // plot interval in ms

#define BNO_ADDRESS               0x4A            // Device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD

uint8_t cargo[23]; 
uint8_t next_data_seqNum          = 0x10;         // next data sequence number 

float q0,q1,q2,q3;                                // quaternions q0 = qw 1 = i; 2 = j; 3 = k;
float h_est;                                      // heading accurracy estimation
uint8_t stat_;                                     // Status (0-3)

                               

const uint8_t quat_report        = 0x05;          // defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),                                                 // without magnetometer : game rotation vector (0x08), AR/VR Game (0x29)
const int reporting_frequency    = 400;           // reporting frequency in Hz  // note that serial output strongly reduces data rate

const uint8_t B0_rate            = 1000000 / reporting_frequency;              //calculate LSB (byte 0)
const uint8_t B1_rate            = B0_rate >> 8;                               //calculate byte 1

                               
/******* Conversions *************/

#define QP(n)                       (1.0f / (1 << n))                   // 1 << n ==  2^-n
#define radtodeg                    (180.0f / PI)
#define TCAADDR 0x70

float quatI[6] = {0};
float quatJ[6] = {0};
float quatK[6] = {0};
float quatReal[6] = {0};

void tcaselect(uint8_t i) {
    //if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}


void setup() {
  Serial.begin(2000000);                  // 115200 baud
  Wire.begin();                          // start I2C communication     
  Wire.setClock(400000L);                // set I2C to 400kHz
  for (uint8_t i = 2; i < 8; i++) {
      tcaselect(i);
      initializeIMU();
  }
  initIndicate();
  delay(1000);
}

void loop() {
    //uint32_t t = millis();
    for (uint8_t i = 2; i < 8; i++) {
        tcaselect(i);
        get_QUAT(i);                                                                    // get actual QUAT data (if new are available)    
        if(quatI[i - 2] == 0 && quatJ[i - 2] == 0 && quatK[i - 2] == 0){
            initializeIMU();
        }
    }
    listenForTrig();
    //Serial.println(1000 / (millis() - t));                      
}

void initIndicate(){
   for(uint8_t i = 0; i < 10; i++){
        digitalWrite(13, HIGH);
        delay(50);
        digitalWrite(13, LOW);
        delay(50);
    }
}

void initializeIMU(){
    Wire.beginTransmission(BNO_ADDRESS);
    while (Wire.endTransmission() != 0);         //wait until device is responding (32 kHz XTO running)
    //Serial.println("BNO found");
    delay(100);                           //needed to accept feature command; minimum not tested
    set_feature_cmd_QUAT();                // set the required feature report data rate  are generated  at preset report interva 
}

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate

void get_QUAT(uint8_t i){                                                               
    Wire.requestFrom(BNO_ADDRESS,23);
    int j = 0; 
    while (Wire.available()){
        cargo[j] = Wire.read();
        j++;
    }

    //Check to see if this packet is a sensor reporting its data to us
    if((cargo[9] == quat_report) && (cargo[2] == 0x03) && (cargo[4] == 0xFB)){    //  && ((cargo[10]) == next_data_seqNum ) check for report and incrementing data seqNum
        //next_data_seqNum = ++cargo[10];                                           // predict next data seqNum              
        stat_ = cargo[11] & 0x03;                                                 // bits 1:0 contain the status (0,1,2,3)  
    
        float qI = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        float qJ = (((int16_t)cargo[16] << 8) | cargo[15] );
        float qK = (((int16_t)cargo[18] << 8) | cargo[17] );
        float qReal = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        quatReal[i - 2] = qToFloat_(qReal, 14); //pow(2, 14 * -1);//QP(14); 
        quatI[i - 2] = qToFloat_(qI, 14); //pow(2, 14 * -1);//QP(14); 
        quatJ[i - 2] = qToFloat_(qJ, 14); //pow(2, 14 * -1);//QP(14); 
        quatK[i - 2] = qToFloat_(qK, 14); //pow(2, 14 * -1);//QP(14);                  // apply Q point (quats are already unity vector)

        //if (quat_report == 0x05){  // heading accurracy only in some reports available
        h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
        h_est *= QP(12);                                                         // apply Q point 
        h_est *= radtodeg;                                                       // convert to degrees                
        //}
    }
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat_(int16_t fixedPointValue, uint8_t qPoint){
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}

void listenForTrig(){
    //int c = Serial.available();
    //if( c > 0){
        //char c = Serial.read();
        //if(c == '*'){
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


//************************************************************************
//                COMMANDS
//************************************************************************

// This code activates quaternion output  at defined rate

void set_feature_cmd_QUAT(){                                 // quat_report determines the kind of quaternions (see data sheets)
  uint8_t quat_setup[21] = {21,0,2,0,0xFD,quat_report,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};  
   Wire.beginTransmission(BNO_ADDRESS);   
   Wire.write(quat_setup, sizeof(quat_setup));            
   Wire.endTransmission();
}

//***********************************************************************************************************************************************
/* This code tares  and stores results in flash after correct positioning and calibration (follow Tare procedure Hillcrest BNO080 Tare function Usage Guide 1000-4045)
* Make sure to run  rotation vector (0x05) and watch h_est and stat (should be << 10 deg and 3 respectively)
* NOTE: tare and calibration are different things
* NOTE: to undo tare persist requires calibration of the device followed by another tare persist with the device oriented y = north bfz = vertical before. 
*/

void TARE(){
    uint8_t tare_now[16] = {16,0,2,0,0xF2,0,0x03,0,0x07,0,0,0,0,0,0,0};                //0x07 means all axes 0x04 = Z axis only; based on rotation vector
    uint8_t tare_persist[16] = {16,0,2,0,0xF2,0,0x03,0x01,0,0,0,0,0,0,0,0};
    Wire.beginTransmission(BNO_ADDRESS);
    Wire.write(tare_now, sizeof(tare_now));
    //Wire.write(tare_persist, sizeof(tare_persist));                                  // uncomment  for tare persist;
    Wire.endTransmission();              
}

//***********************************************************************************************************************************************
/* The calibration data in RAM is always updated in background. 'Save DCD'  push the RAM record into the FLASH for reload @ next boot.
   The algorithm will only update the calibration data when it feels the current accuracy is good. You don't need to care about the status or heading error.
   Save  before power off or whenever you would like to do. (Hillcrest information)  
 */

void save_DCD(){                                             
  uint8_t save_dcd[16] = {16,0,2,0,0xF2,0,0x06,0,0,0,0,0,0,0,0,0};
  Wire.beginTransmission(BNO_ADDRESS);  
  Wire.write(save_dcd, sizeof(save_dcd));
  Wire.endTransmission();    
}

//***********************************************************************************************************************************************
/* This code disables the calibration running in the background of the accelerometer gyro and magnetometer.  
 * sensors can be set individually on and off (chapter 6.4.7.1)
 * P0 = accelerometer, P1 = gyro; P2 =magnetometer; P4 = planar accelerometer (without Z axis)
 * 0 = disable; 1 = enable
 */

void ME_cal(uint8_t P0, uint8_t P1, uint8_t P2, uint8_t P4){
  uint8_t me_cal[16] = {16,0,2,0,0xF2,0,0x07,P0,P1,P2,0,P4,0,0,0,0};
  Wire.beginTransmission(BNO_ADDRESS);              
  Wire.write(me_cal, sizeof(me_cal));
  Wire.endTransmission();  
}

//***********************************************************************************************************************************************
