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
