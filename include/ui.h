void readUI(void *pvparemeters) { 




    for (;;) {
        if (xSemaphoreTake(controlLock, portMAX_DELAY) == pdTRUE) {  // take control lock to read UI values and update shared variables that are used in other tasks and ISRs
        



  //if (onboardcontrol == 1) {

#if (enableButtons)
  buttonA.loop();  // update button managed by Bounce2 library
  buttonB.loop();  // update button managed by Bounce2 library


#endif
    motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

    xQueueSend(motPot, &motPotVal, 5);  // send motor pot value to motor task via queue

      ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));

          if (safeMode == 1) {
    if (motSingle != 0) {
      // We're in one of the single modes, so ...
      ledPotVal = ledPotVal * safeMin;  // dim lamp to min safe brightness immediately
    } else {
      safeSpeedMult = fscale(4, 24.0, safeMin, 1.0, abs(FPSrealAvg), 0);  // safety multiplier for FPS (with optional nonlinear scaling)
      safeSpeedMult = constrain(safeSpeedMult, safeMin, 1.0);             // clip values
      ledPotVal = ledPotVal * safeSpeedMult;                              // decrease brightness to prevent film burns
    }
  }
    xQueueSend(ledPot, &ledPotVal, 5);  // send LED pot value to LED task via queue




#if (enableSafeSwitch)
  safeMode = digitalRead(safeSwitch);
#endif
xSemaphoreGive(controlLock);  // give back control lock after updating shared variables that are used in other tasks and ISRs
vTaskDelay(20 / portTICK_PERIOD_MS);}
    } 
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
  if (FPSunsigned > 23.5 && FPSunsigned < 24.5) {
    updateStatusLED(0, 0, 30, 0);  // green LED at 24fps
  } else if (FPSunsigned > 17.5 && FPSunsigned < 18.5) {
    updateStatusLED(0, 20, 0, 16);  // purple LED at 18fps
  } else {
    updateStatusLED(0, 18, 16, 10);  // white LED at idle
  }
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

void debugTask(void *pvParameters) { 
    static int startup = 0;
    static int debugcnt1;
    static int debugcnt2;
    static int ang;
    for(;;) {
  if (debug) {


//     // Serial.print("Motor Pot Queue: ");
//     // Serial.print(xQueuePeek(motPot, NULL, 0)); 
//     // Serial.print(" | LED Pot Queue: ");
//     // Serial.print(xQueuePeek(ledPot, NULL, 0));
//     // Serial.print(" | LED Bright Queue: ");
//     // Serial.print(xQueuePeek(q_ledBright, NULL, 0));
//     // Serial.print(" | Shutter Blade Queue: ");
//     // Serial.print(xQueuePeek(q_shutterBlade, NULL, 0));

//     Serial.print("Shutter Position: ");
// if (xSemaphoreTake(spiRead, 200 / portTICK_PERIOD_MS) == pdTRUE)
// {
//     xQueueReceive(q_spiRead, &ang, 25/portTICK_PERIOD_MS);  // receive shutter map from shutter task via queue
//     Serial.print(ang);
//     xSemaphoreGive(spiRead);
// } 
// //     Serial.print(ang);
// // }
// //     // Serial.print(" | outCommanderQueue");
// //     // Serial.println(xQueuePeek(outCommanderQueue, NULL, 0));
// //     Serial.println('\n');



    Serial.print("encoderRead? ");
    if(uxSemaphoreGetCount(spiRead) == 0) {
      Serial.print("No");
    } else {
      Serial.print("Yes");
    }

                Serial.print(" || ");

            Serial.print("shutterMapping? ");
    if(uxSemaphoreGetCount(shutterMapping) == 0) {
      Serial.print("No");
    } else {
      Serial.print("Yes");
    }

                Serial.print(" || ");


            Serial.print("controlLock? ");
    if(uxSemaphoreGetCount(controlLock) == 0) {
      Serial.print("No");
    } else {
      Serial.print("Yes");
    }
                    Serial.print(" || ");



    Serial.print("Heap free: ");
    Serial.print(xPortGetFreeHeapSize());

                Serial.print(" || ");


//             Serial.println('\n');
xQueueReceive(q_intr1_hasRun, &debugcnt1, 1);  // receive from the queue to see if the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue
Serial.print("LED ISR: " + String(debugcnt1));
xQueueReceive(q_intr2_hasRun, &debugcnt2, 1);  // receive from the queue to see if the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue
Serial.print(" | " + String(debugcnt2));

                                Serial.print(" || ");


xQueueReceive(q_debugShutterPosition, &ang, 1);  // receive from the queue to see if the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue
Serial.print("encCount: " + String(ang));

                Serial.println(" || ");


        



vTaskDelay(50 / portTICK_PERIOD_MS);}

  } }