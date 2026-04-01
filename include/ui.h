static volatile uint64_t last_isr_time = 0;
#define DEBOUNCE_DELAY_US 10000ULL  // Debounce delay in microseconds (50 ms)

static void IRAM_ATTR gpioGet(void *arg) 
{
  uint64_t now = esp_timer_get_time();

  if (now - last_isr_time > DEBOUNCE_DELAY_US) {

    uint32_t gpio_num = (uint32_t) arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    last_isr_time = now;
    xQueueSendFromISR(ioQ, &gpio_num, &higher_priority_task_woken);
    xSemaphoreGiveFromISR(physinput, &higher_priority_task_woken);
    vTaskResume(ioTASKHANDLE);
    
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}
}

static void parseIO(void *pvparemeters) {
  static uint32_t whatio;
  for(;;)
  {
    xSemaphoreTake(physinput, portMAX_DELAY);

    xQueueReceive(ioQ, &whatio, portMAX_DELAY);

    Serial.println(whatio);

    if (whatio == motDirBckSwitch)
    {
      motorCommander('r');
    } else if (whatio == motDirFwdSwitch)
    {
      motorCommander('f');
    } else if (whatio == safeSwitch)
    {
      motorCommander('x');
    }

    vTaskSuspend(NULL);
  }
}



void readUI(void *pvparemeters) { 


    static int ledPotVal = 0;
    static int ledB = 0;
    static int ledSlewValOld;
    static int motSlewValOld;
    static int ledSlewVal;

    static IRAM_ATTR int fwdS = 9;
    static IRAM_ATTR int revS= 9;
    static IRAM_ATTR int safeS= 9;
    static IRAM_ATTR int btnF= 9;
    static IRAM_ATTR int btnR= 9;



static rampInt ledAvg; // ramp object for LED brightness slewing

    for (;;) {
        // if (xSemaphoreTake(controlLock, portMAX_DELAY) == pdTRUE) {  // take control lock to read UI values and update shared variables that are used in other tasks and ISRs

      xQueueReceive(q_motRunFwd, &fwdS, 3);
      xQueueReceive(q_motRunBack, &revS, 3);
      xQueueReceive(q_safemode, &safeS, 3);
      xQueueReceive(q_buttonF, &btnF, 3);
      xQueueReceive(q_buttonR, &btnR, 3);

      if(fwdS == 1) 
      {
        Serial.print("FWD");
        fwdS = 9;
      } else if (fwdS == 0) 
      {
        Serial.print("x");
        fwdS = 9;
      }

      if(revS == 1)
      {
        Serial.print("REV");
        revS = 9;

      } else if (revS == 0)
      {
        Serial.print("x");
        revS = 9;
      }
      
       if (safeS == 1)
      {
        Serial.print("SAFE");
        safeS = 9;
      } else if (safeS == 0)
      {
        Serial.print("x");
        safeS = 9;
      }

      if (btnF == 1)
      {
        Serial.print("FB");
        btnF = 9;
      } else if (btnF == 0)
      {
        Serial.print("x");
        btnF = 9;
      }

      if (btnR == 1)
      {
        Serial.print("RB");
        btnR = 9;
      } else if (btnR == 0)
      {
        Serial.print("x");
        btnR = 9;
      }


// Serial.print(ledWrite_period);
// Serial.println("uS");



      // Serial.print(fwdS);
      // Serial.print(revS);
      // Serial.print(safeS);
      // Serial.print(btnF);
      // Serial.println(btnR);






  //if (onboardcontrol == 1) {

// #if (enableButtons)
//   buttonA.loop();  // update button managed by Bounce2 library
//   buttonB.loop();  // update button managed by Bounce2 library


// #endif
    motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

    xQueueSend(motPot, &motPotVal, 5);  // send motor pot value to motor task via queue

      ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));

          if (safeMode == 1) {
    if (motSingle != 0) {
      // We're in one of the single modes, so ...
      ledPotVal = ledPotVal * safeMin;  // dim lamp to min safe brightness immediately
    } else {
      // safeSpeedMult = fscale(4, 24.0, safeMin, 1.0, abs(FPSrealAvg), 0);  // safety multiplier for FPS (with optional nonlinear scaling)
      safeSpeedMult = constrain(safeSpeedMult, safeMin, 1.0);             // clip values
      ledPotVal = ledPotVal * safeSpeedMult;                              // decrease brightness to prevent film burns
    }
  }




// #if (enableSafeSwitch)
//   safeMode = digitalRead(safeSwitch);
// #endif

    //first 

  ledAvg.update();  // LED slewing managed by Ramp library

  #if (enableSlewPots)
  ledSlewVal = map(ledSlewVal, 0, 4095, ledSlewMin, ledSlewMax);  // turn slew val pot into ms ramp time
#endif
  // if knobs have changed sufficiently, calculate new slewing ramp time
    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledPotVal - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledPotVal, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledPotVal;
    }

    // set brightness to slewed version of pot value (mapped/clipped because kalman filter sometimes doesn't allow us to reach min/max)
    ledB = map(ledAvg.getValue(), 40, 4045, ledMin, ledMax);
    ledB = constrain(ledB, 0, 4095);

    xQueueSend(ledPot, &ledB, 5);  // send LED brightness value to LED task via queue

// xSemaphoreGive(controlLock);  // give back control lock after updating shared variables that are used in other tasks and ISRs
vTaskDelay(60 / portTICK_PERIOD_MS);}
    
}



void calcFPS(void *pvParameters) { for(;;) { //moveto core 0 with related interrupts
  myFPScount = FPScount;        // copy volatile FPS to nonvolatile variable so it's safe
  myFPSframe = FPSframe;        // copy volatile FPS to nonvolatile variable so it's safe
  myMotModeReal = motModeReal;  // copy volatile FPS to nonvolatile variable so it's safe

  if (myFPScount > 5) {
    FPSreal = myFPSframe;
  } else {
    FPSreal = myFPScount;
  }

  FPSreal = FPSreal * FPSmultiplier;  // convert the FPS for the P26

  FPSAvgTotal = FPSAvgTotal - FPSAvgArray[FPSAvgReadIndex];  // subtract the last reading
  FPSAvgArray[FPSAvgReadIndex] = FPSreal;                    // store the new reading
  FPSAvgTotal = FPSAvgTotal + FPSAvgArray[FPSAvgReadIndex];  // add the reading to the total
  FPSAvgReadIndex = FPSAvgReadIndex + 1;                     // advance to the next position in the array
  if (FPSAvgReadIndex >= FPSnumReadings) {                   // if we're at the end of the array...
    FPSAvgReadIndex = 0;                                     // ...wrap around to the beginning:
  }
  FPSrealAvg = FPSAvgTotal / FPSnumReadings;  // calculate the average
  FPSrealAvg = FPSrealAvg * myMotModeReal;    // add sign based on film travel direction

  // If digital shutter is enabled but no encoder movement in last 200 msec, assume that we're stopped.
  if (countPeriod > 200000 && enableShutter) {
    FPSrealAvg = 0;
  }
  float FPSunsigned = abs(FPSrealAvg);
  // if (FPSunsigned > 23.5 && FPSunsigned < 24.5) {
  //   updateStatusLED(0, 0, 30, 0);  // green LED at 24fps
  // } else if (FPSunsigned > 17.5 && FPSunsigned < 18.5) {
  //   updateStatusLED(0, 20, 0, 16);  // purple LED at 18fps
  // } else {
  //   updateStatusLED(0, 18, 16, 10);  // white LED at idle
  // }
  vTaskDelay(200 / portTICK_PERIOD_MS);
}
}

// #if (enableButtons)
// void pressed(Button2& btn) {
//   if (btn == buttonA) {
//     if (buttonAstate == 0) {
//       if (debugUI) {
//         Serial.println("Button A pressed");
//       }
//       buttonAstate = 1;
//       if (motSwitchMode) {
//         // Eiki motor switch mode, so we have a button for each single frame direction
//         if (motMode == 0) {
//           motSingle = 1;  // If stopped when button was pressed, set single frame fwd
//         } else {
//           motSingle = 2;  // Set freeze frame
//         }
//       } else {
//         // P26 motor switch mode with one button, so we use a specific mode
//         motSingle = 3;
//       }
//     }
//   } else if (btn == buttonB) {
//     if (buttonBstate == 0) {
//       if (debugUI) {
//         Serial.println("Button B pressed");
//       }
//       buttonBstate = 1;
//       if (motSwitchMode) {
//         // Eiki motor switch mode, so we have a button for each single frame direction
//         // P26 doesn't have a buttonB, so need to test further
//         if (motMode == 0) {
//           motSingle = -1;  // If stopped when button was pressed, set single frame rev
//         } else {
//           motSingle = 2;  // Set freeze frame
//         }
//       }
//     }
//   }

// }


// void released(Button2& btn) {
//   if (btn == buttonA) {
//     if (buttonAstate == 1) {

//       if (debugUI) {
//         Serial.println("Button A released");
//       }
//       if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
//       motSinglePrev = motSingle;
//       buttonAstate = 0;
//       shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
//       Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
//       Serial.println(motSingle);
//       if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
//       motSinglePrev = motSingle;
//       buttonAstate = 0;
//       shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
//       Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
//       Serial.println(motSingle);
//     }
//   } else if (btn == buttonB) {
//     if (buttonBstate == 1) {
//       if (debugUI) {
//         Serial.println("Button B released");
//       }
//       if (motSingle == 2 || motSingle == 3) motSingle = 0;  // turn off single flag if we're leaving Eiki freeze-frame mode or P26 open shutter mode
//       motSinglePrev = motSingle;
//       buttonBstate = 0;
//       shutterQueue(shutBladesVal, shutAngleVal);  // return shutter map to normal
//       Serial.print("Leaving Eiki / P26 FREEZE/BURN mode... motSingle = ");
//       Serial.println(motSingle);
//     }
//   }
// }
// #endif

void IRAM_ATTR debugTask(void *pvParameters) { 
    static int startup = 0;
    static int debugcnt1;
    static int debugcnt2;
    // *ang1 = &ang;
    for(;;) {
//  if (debug == 1) {
// static int *ang1;

      // Serial.println("");
      //           for (int indx = 0; indx < 1024; indx++)
      //           {
      //                   Serial.print(shutterMap[indx]);
      //                       Serial.flush();


      //           }
                //     if (indx == ang)
                //     {
                //       if (shutterMap[indx] == 1)
                //       {
                //         Serial.print("O");
                //       }
                //       else if (shutterMap[indx] == 0)
                //       {
                //         Serial.print(" ");
                        
                //       }

                //     } else
                //     {
                //         if (shutterMap[indx] == 1)
                //         {
                //             Serial.print(" ");
                //         } else if (shutterMap[indx] == 0)
                //         {
                //             Serial.print("X");
                //         }
                //     }
                // }
            


        



          }

  } 

  void readShutterControls(void *parameter)
{ 
    static int shutBladesVal = 0;
    static int shutAngleVal = 0;


    for (;;)
    {
        #if (enableSlewPots)
        motSlewVal = motSlewPotKalman.updateEstimate(analogRead(motSlewPotPin));
        // delay(2);
        ledSlewVal = ledSlewPotKalman.updateEstimate(analogRead(ledSlewPotPin));
        // delay(2);
        #endif

        #if (enableShutterPots && enableShutter)
        if (musicMode == 0)
        {
            shutBladesPotVal = shutBladesPotKalman.updateEstimate(analogRead(shutBladesPotPin));
            if (shutBladesPotVal < 800)
            {
                shutBladesVal = 1;
            }
            else if (shutBladesPotVal < 2500)
            {
                shutBladesVal = 2;
            }
            else
            {
                shutBladesVal = 3;
            }
            shutAnglePotVal = shutAnglePotKalman.updateEstimate(analogRead(shutAnglePotPin));
            shutAngleVal = map(shutAnglePotVal, 0, 4090, 1, 100); // map ADC input to range of shutter angle
        }
        else if (musicMode == 1)
        {
            shutBladesPotVal = map(CC2ProjBlades, 0, 100, 0, 4095);
            // inject values during Music Mode
        }
        
        xQueueSend(q_shutterBlade, &shutBladesVal, 2); // send shutter blade pot value to shutter task via queue
        xQueueSend(q_shutterAngle, &shutAngleVal, 2);  // send shutter angle pot value to shutter task via queue
                                                        // clip values
        #endif
    }
}