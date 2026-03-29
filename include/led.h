void updateLed(void *pvParameters) { 

   
    static int ledPotVal = 0;
    static int ledBright = 0;
    static int ledSlewValOld;
    static int motSlewValOld;
    static int shutBladesValOld;
    static float shutAngleValOld;





    for(;;) {
            // if (xSemaphoreTake(shutterMapping, portMAX_DELAY) == pdTRUE) { // take the semaphore to ensure exclusive access to shared variables

        xQueueReceive(ledPot, &ledPotVal, 3);  // receive motor pot value from UI task via queue

  //noInterrupts();


//   if (abs(shutBladesVal != shutBladesValOld) || abs(shutAngleVal - shutAngleValOld) >= 0.05) {
//     if (enableShutter == 1) {
//       //Serial.println("(SHUTTERMAP SHUTTER POTS)");
//     }
//     shutBladesValOld = shutBladesVal;
//     shutAngleValOld = shutAngleVal;
//   }


  

  // SAFE MODE CHECK
  

  xQueueSend(q_ledBright, &ledBright, portMAX_DELAY);  // send LED brightness value to commander task via queue for potential forwarding to remote control

  if ((opticalPrinter == 1 || scanFlag == 1) && capFlag == 1) {
    ledBright = ledBright;
    // shutterQueue(1, 1.0);  // force open shutter for single framing
    digitalWrite(ledPin, 1);
    send_LEDC();
  } else if ((opticalPrinter == 1 || scanFlag == 1) && capFlag == 0) {
    ledBright = 0;
    send_LEDC();
    //this is for emulating a capper, so that i can work with intervals in Mcopy
  }

  // STOPPED CHECK (Turn off lamp if projector is stopped, except when single buttons are pressed or single is active)
  if (opticalPrinter == 0 && scanFlag == 0) {
    if (FPSrealAvg == 0 && motSingle == 0 && !buttonAstate && !buttonBstate) {
      ledBright = 0;
    }
    //Serial.print("STOP CHECK ");
  }




  //interrupts();

//   if (debugLed) {
//     Serial.print("LED Slew: ");
//     Serial.print(ledSlewVal);
//     Serial.print(", LED Pot: ");
//     Serial.print(ledPotVal);
//     Serial.print(", Spd Mult: ");
//     Serial.print(safeSpeedMult);
//     Serial.print(", LED Bright: ");
//     Serial.print(ledBright);
//     Serial.print(", Shutter Blades: ");
//     Serial.print(shutBladesVal);
//     Serial.print(", Shutter Angle: ");
//     Serial.println(shutAngleVal);
//   }
//   // at slow speeds OR if the shutter is fully open OR shutter disabled, update the LED PWM directly because ISR isn't firing often (or at all)
//   if (countPeriod > 50000 || shutAngleVal == 1.0 || !enableShutter || musicMode == 1) {
//     send_LEDC();
//   }
// xSemaphoreGive(shutterMapping); // give the semaphore back so other tasks can access shared variables
// vTaskDelay(100 / portTICK_PERIOD_MS);}
//     }
// }
    }
}


    static int ledSlewVal = 0;
    static int ledSlewValOld;
    static rampInt motAvg; // ramp object for motor speed slewing
    static rampInt ledAvg; // ramp object for LED brightness slewing
        static int ledBright;
   static bool shutMapIRQ[countsPerFrame];  // local variable to hold the shutter map value received from the queue, which will be used to update the LED state in real time in the ISR based on how many blades are actually open at any given moment
   static int actualShutterMapPosition;

void IRAM_ATTR send_LEDC() {





    taskENTER_CRITICAL_ISR(&shutterMappingLock);
    if (xSemaphoreTakeFromISR(controlLock, &xHigherPriorityTaskWoken) == pdTRUE) { // take the semaphore to ensure exclusive access to shared variables
    xQueueReceiveFromISR(ledPot, &ledBright, &xHigherPriorityTaskWoken);  // receive LED brightness value from LED task via queue
    xSemaphoreGiveFromISR(controlLock, &xHigherPriorityTaskWoken); // give the semaphore back after updating shared variables
    }


    //first 

  ledAvg.update();  // LED slewing managed by Ramp library

  #if (enableSlewPots)
  ledSlewVal = map(ledSlewVal, 0, 4095, ledSlewMin, ledSlewMax);  // turn slew val pot into ms ramp time
#endif
  // if knobs have changed sufficiently, calculate new slewing ramp time
    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledBright - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledBright, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledBright;
    }

    // set brightness to slewed version of pot value (mapped/clipped because kalman filter sometimes doesn't allow us to reach min/max)
    ledBright = map(ledAvg.getValue(), 40, 4045, ledMin, ledMax);
    ledBright = constrain(ledBright, 0, 4095);

    // if (xSemaphoreTakeFromISR(shutterMapping, NULL) == pdTRUE) { // take the semaphore to ensure exclusive access to shared variables

    xQueueReceiveFromISR(q_actualShutterMap, &actualShutterMapPosition, &xHigherPriorityTaskWoken);  // receive shutter blade value from UI task via queue
    xQueueReceiveFromISR(q_shutterMap, &shutMapIRQ, &xHigherPriorityTaskWoken);  // receive shutter map from shutter task via queue
    // xSemaphoreGiveFromISR(shutterMapping, NULL); // give the semaphore back after updating shared variables
    // }







  if (LedDimMode) {  // PWM mode
    //if shutter map is set to be open at the point in the map/array pointed to by the actual shutter map position
    if (shutMapIRQ[actualShutterMapPosition] == 0 || enableShutter == 0) {
      // LED ON for this step of shutter OR shutter is disabled OR single framing, so LED is always on
    //   ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);  // configure LED PWM function using LEDC channel
    //   ledcAttachPin(ledPin, ledChannel);                   // attach the LEDC channel to the GPIO to be controlled
    //   if (LedInvert) {
    //     ledcWrite(ledChannel, (1 << ledBrightRes) - ledBright);  // set lamp to desired brightness (inverted)
    //   } else {
    //     ledcWrite(ledChannel, ledBright);  // set lamp to desired brightness
    //   }
        ledcWrite(ledChannel, 0);  // set lamp to desired brightness



    } else if (shutMapIRQ[actualShutterMapPosition] == 1 && enableShutter == 1) {
      // LED OFF for this segment of shutter

    //   if (LedInvert) {
    //     // ledcDetachPin(ledPin);    // detach the LEDC channel to the GPIO to be controlled
    //     digitalWrite(ledPin, 1);  // send pin high to turn off LED
    //   } else {

        ledcWrite(ledChannel, ledBright);  // set lamp to desired brightness

        // digitalWrite(ledChannel, 0);  // send pin high to turn off LED
        // ledcDetachPin(ledPin);  // detach the LEDC channel to the GPIO to be controlled

        //i need to figure out why detachment is necessary
    //   }
    //   ledcDetachPin(ledPin);
    }

//   } else {
//     if (LedInvert) {
//       if (enableShutter) {
//         digitalWrite(ledPin, !(shutterState));  // active low, using shutter
//         ledcDetachPin(ledPin);
//       } else {
//         digitalWrite(ledPin, 0);  // active low, no shutter
//       }
//     } else {
//       if (enableShutter) {
//         digitalWrite(ledPin, shutterState);  // active high, using shutter
//       } else {
//         digitalWrite(ledPin, 1);  // active high, no shutter
//       }
//     }
  }
  taskEXIT_CRITICAL_ISR(&shutterMappingLock);
  if(debug)
  {
    debugShutterPosition = actualShutterMapPosition;
    intr2_hasRun++;
    xQueueSendFromISR(q_debugShutterPosition, &debugShutterPosition, &xHigherPriorityTaskWoken);  // send the current shutter position to the debug task so that it can be printed to the serial monitor for debugging purposes
  xQueueSendFromISR(q_intr2_hasRun, &intr2_hasRun, &xHigherPriorityTaskWoken);  // send a value to the shutter task to indicate that the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue

  }
  
}