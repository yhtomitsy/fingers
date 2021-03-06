//  if (stat_ == 3)  digitalWrite(Led, !digitalRead(Led));                   // blink Led every loop run (--> to measure loop duration);             
//  else digitalWrite(Led, LOW);                                            // status Led 


// *************  buttons *******************************
  
//  if(digitalRead(btn_TARE) == LOW ){                        // button pressed  stores actual phi as mean value and saves actual calibration to flash
//    delay(200);
//    while(digitalRead(btn_TARE) == LOW);                    // wait for button release
//    //actions follow  here
//    TARE();                                                 
//   }    
//
//
//   if(digitalRead(btn_CAL) == LOW){
//     delay(200);
//     while(digitalRead(btn_CAL) == LOW );       // wait for button release
//    
//     //actions follow here
//     save_DCD();                                     // store cal in flash
//     delay(200);
//     ME_cal(0,0,1,0);                                //autocal acc + gyro stop; magnetometer  cal continues
//    }
 //*******************************************************

//}                                                    // loop ends here


/*******************************************************************************************************************
/* This code reads quaternions 
 * kind of reporting (quat_report) is defined above
 */


/*void get_QUAT(){                                                               
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
          //Serial.print(".");
          cargo[i] = Wire.read();
          i++;
        }
        //Serial.println();
    }
    
    if((cargo[9] == quat_report)){        //&& ((cargo[10]) == next_data_seqNum ) check for report and incrementing data seqNum
        //next_data_seqNum = ++cargo[10];                                         // predict next data seqNum              
        stat_ = cargo[11] & 0x03;                                                // bits 1:0 contain the status (0,1,2,3)  
    
        q1 = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        q2 = (((int16_t)cargo[16] << 8) | cargo[15] );
        q3 = (((int16_t)cargo[18] << 8) | cargo[17] );
        q0 = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        q0 *= QP(14); q1 *= QP(14); q2 *= QP(14); q3 *= QP(14);                  // apply Q point (quats are already unity vector)

       if (quat_report == 0x05 || quat_report == 0x09 || quat_report == 0x28 ){  // heading accurracy only in some reports available
          h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
          h_est *= QP(12);                                                         // apply Q point 
          h_est *= radtodeg;                                                       // convert to degrees                
       }
//       Serial.print ("; q0 "); Serial.print (q0 + 0.00005f,4);                  // = qw (more digits to find out north direction (y axis N --> q0 = 1)
//       Serial.print ("; q1 "); Serial.print (q1 + 0.0005f,3);
//       Serial.print ("; q2 "); Serial.print (q2 + 0.0005f,3);
//       Serial.print ("; q3 "); Serial.println (q3 + 0.0005f,3);
    }
}*/

/*if (millis() - plot_interval > 0){ 
            //Serial.print ("S "); Serial.print (stat_);
            //Serial.print ("; E "); Serial.print (h_est + 0.05f,1);                   // including rounding
            Serial.print ("; q0 "); Serial.print (q0 + 0.00005f,4);                  // = qw (more digits to find out north direction (y axis N --> q0 = 1)
            Serial.print ("; q1 "); Serial.print (q1 + 0.0005f,3);
            Serial.print ("; q2 "); Serial.print (q2 + 0.0005f,3);
            Serial.print ("; q3 "); Serial.println (q3 + 0.0005f,3);
            plot_interval = millis();
        }*/


/* Utilities (Quaternion mathematics)
 
  q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3  = 1
   
 // Gravity vector from quaternions
  
  gx = q1 * q3 - q0 * q2;
  gy = q0 * q1 + q2 * q3;
  gz = q0 * q0 + q3 * q3 -0.5f; 
  norm = sqrtf(gx + gx + gy * gy + gz * gz);                                                              
  norm = 1.0f / norm;
  gx *= norm; gy *= norm; gz *= norm;
 
 // Euler angles from quaternions (radians)   

  yaw   =  atan2((q1 * q2 + q0 * q3), ((q0 * q0 + q1 * q1) - 0.5f));   
  pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
  roll  =  atan2((q0 * q1 + q2 * q3), ((q0 * q0 + q3 * q3) - 0.5f));
  yaw *= radtodeg;    pitch *= radtodeg;    roll *= radtodeg; 
    
 # Rotation matrix (half)
 
 R00 = q1 * q1 + q2 * q2 -0.5f; // = 0.5f - q3 * q3 - q4 * q4;
 R01 = q2 * q3 - q1 * q4;
 R02 = q2 * q4 + q1 * q3;
 R10 = q2 * q3 + q1 * q4;
 R11 = q1 * q1 + q3 * q3 -0.5; // = 0.5f - q2 * q2 - q4 * q4;
 R12 = q3 * q4 - q1 * q2;
 R20 = q2 * q4 - q1 * q3;
 R21 = q3 * q4 + q1 * q2;
 R22 = q1 * q1 + q4 * q4 -0.5f; // = 0.5f -q2 * q2 -q3 * q3;

*/
