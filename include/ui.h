



#define DEBOUNCE_DELAY_US 20000ULL // Debounce delay in microseconds (50 ms)


static uint32_t gpio_num;
static void IRAM_ATTR switchGet(void *arg)
{
  uint64_t now = esp_timer_get_time();

  if (now - last_switch_isr_time > DEBOUNCE_DELAY_US)
  {

    gpio_num = (uint32_t)arg;
    BaseType_t higher_priority_task_woken = pdTRUE;
    last_switch_isr_time = now;

    
    xQueueSendFromISR(ioQ, &gpio_num, &higher_priority_task_woken);

    xSemaphoreGiveFromISR(physinput, &higher_priority_task_woken);


    if (higher_priority_task_woken)
    {
      portYIELD_FROM_ISR();
    }

  }
}


static IRAM_ATTR void parseIO(void *pvparemeters)
{
  static uint8_t whatio;
  for (;;)
  {
    if (xSemaphoreTake(physinput, 2)== pdTRUE)
    {
      xQueueReset(ioQ);
      xQueueReset(runMsg);

    xQueueReceive(ioQ, &whatio, portMAX_DELAY);
            ESP_LOGI("motCont", "hello %u", whatio);


    if(whatio == motDirFwdSwitch)
    {
      motDirFwdState++;
      if(motDirFwdState > 1)
      {
        motDirFwdState = 0;
        char isDir = 'b';
        xQueueSend(runMsg, &isDir, 2);
      }

      if (motDirFwdState == 1)
      {
        char isDir = 'f';
        xQueueSend(runMsg, &isDir, 2);
      }
      // xQueueSend(ioQ, &whatio, 1);

    } else if (whatio == motDirBckSwitch)
    {
      motDirBckState++;
      if(motDirBckState > 1)
      {
        motDirBckState = 0;
        char isDir = 'l';
        xQueueSend(runMsg, &isDir, 2);
      }
      if(motDirBckState == 1)
      {
        char isDir = 'r';
        xQueueSend(runMsg, &isDir, 2);
      }
      // xQueueSend(ioQ, &whatio, 1);

    }else if (whatio == buttonApin)
    {
      buttonApinState++;
      if(buttonApinState > 1)
      {
        buttonApinState = 0;
      }
      singleFrameActor('i', buttonApin, buttonApinState);

    }else if (whatio == buttonBpin)
    {
      buttonBpinState++;
      if(buttonBpinState > 1)
      {
        buttonBpinState = 0;
      }
      singleFrameActor('i', buttonBpin, buttonBpinState);
    }
  xSemaphoreGive(physinput);


  }
  vTaskDelay(60);

  }
}
  static uint8_t button = 1;


static void singleFrameActor(char fromwhere, uint8_t frombutton, uint8_t level)
{

  button = level;
  if (fromwhere == 'i')
  {
    if (frombutton == buttonApin)
    {
      char fb = 'a';
      xQueueSend(q_buttonF, &button, portMAX_DELAY);
      xQueueSend(q_whichbutton, &fb, portMAX_DELAY);
      // motorCommander('f', true);



    } else if (frombutton == buttonBpin)
    {
      char fb = 'b';

      xQueueSend(q_buttonR, &button, portMAX_DELAY);
      xQueueSend(q_whichbutton, &fb, portMAX_DELAY);

      // motorCommander('r', true);
    }
  }
}

IRAM_ATTR char controllerInternalDirection = 'x';

/*supposed to function like a wrapper.
'i' is for 'i'nterrupt
'e' is for 'e'xternal
's' is for 's'ensor */


  static int ledSlewValOld;
  static int ledSlewVal;

  static IRAM_ATTR int fwdS = 9;
  static IRAM_ATTR int revS = 9;
  static IRAM_ATTR int safeS = 9;
  static IRAM_ATTR int btnF = 9;
  static IRAM_ATTR int btnR = 9;
  static uint8_t safeModeIs = 0;

  // static rampInt ledAvg; // ramp object for LED brightness slewing

    static int MPotEstimate;
    static int LEDPotEstimate;

void IRAM_ATTR readUI(void *pvparemeters)
{





  for (;;)
  {

    // if (eTaskGetState(ledDraw) == pdTRUE){}
    // else if (eTaskGetState(ledDraw) == pdFALSE)
    // {

    // }
    




            
    adc_oneshot_read(adc_handle, motPotPin, &motPotVal);
    adc_oneshot_read(adc_handle, ledPotPin, &ledPotVal);
    // motPotVal = 2048;
    // ledPotVal = 2048;

    // MPotEstimate = motPotKalman -> UpdateRefMeasurement((float)1.0, (float)shutBladesPotVal, kalmanMEA, kalmanQ);
    // LEDPotEstimate = ledPotKalman -> UpdateRefMeasurement((float)1.0, (float)ledPotVal, kalmanMEA, kalmanQ);

    MPotEstimate = motPotVal;
    LEDPotEstimate =ledPotVal;

    // if(eTaskGetState(motorCommander) == pdFALSE)
 
    // motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

    xQueueSend(motPot, &MPotEstimate, 5); // send motor pot value to motor task via queue

    // ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));

    xQueueReceive(q_safemode, &safeModeIs, 1);
    if (safeModeIs == 1)
    {

      safeSpeedMult = constrain(safeSpeedMult, safeMin, 1.0); // clip values
      LEDPotEstimate = LEDPotEstimate * safeSpeedMult;                  // decrease brightness to prevent film burns
    }
    // ledAvg.update(); // LED slewing managed by Ramp library

#if (enableSlewPots)
    ledSlewVal = map(ledSlewVal, 0, 4095, ledSlewMin, ledSlewMax); // turn slew val pot into ms ramp time
#endif
    // if knobs have changed sufficiently, calculate new slewing ramp time
    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(LEDPotEstimate - ledPotValOld) >= 50)
    {
      // Serial.println("(LED UPDATE SLEW)");
      // ledAvg.go(LEDPotEstimate, ledSlewVal); // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = LEDPotEstimate;
    }

    // set brightness to slewed version of pot value (mapped/clipped because kalman filter sometimes doesn't allow us to reach min/max)
    // ledB = map(ledAvg.getValue(), 40, 4045, ledMin, ledMax);
    // ledB = constrain(ledB, 0, 4095);

    ledB = LEDPotEstimate;

    xQueueSend(ledPot, &ledB, 5); // send LED brightness value to LED task via queue

    // xSemaphoreGive(controlLock);  // give back control lock after updating shared variables that are used in other tasks and ISRs
    vTaskDelay(60 / portTICK_PERIOD_MS);
  }
}

void calcFPS(void *pvParameters)
{
  for (;;)
  {                              // moveto core 0 with related interrupts



    myFPScount = FPScount;       // copy volatile FPS to nonvolatile variable so it's safe
    myFPSframe = FPSframe;       // copy volatile FPS to nonvolatile variable so it's safe
    myMotModeReal = motModeReal; // copy volatile FPS to nonvolatile variable so it's safe

    if (myFPScount > 5)
    {
      FPSreal = myFPSframe;
    }
    else
    {
      FPSreal = myFPScount;
    }

    FPSreal = FPSreal * FPSmultiplier; // convert the FPS for the P26

    FPSAvgTotal = FPSAvgTotal - FPSAvgArray[FPSAvgReadIndex]; // subtract the last reading
    FPSAvgArray[FPSAvgReadIndex] = FPSreal;                   // store the new reading
    FPSAvgTotal = FPSAvgTotal + FPSAvgArray[FPSAvgReadIndex]; // add the reading to the total
    FPSAvgReadIndex = FPSAvgReadIndex + 1;                    // advance to the next position in the array
    if (FPSAvgReadIndex >= FPSnumReadings)
    {                      // if we're at the end of the array...
      FPSAvgReadIndex = 0; // ...wrap around to the beginning:
    }
    FPSrealAvg = FPSAvgTotal / FPSnumReadings; // calculate the average
    FPSrealAvg = FPSrealAvg * myMotModeReal;   // add sign based on film travel direction

    // If digital shutter is enabled but no encoder movement in last 200 msec, assume that we're stopped.
    if (countPeriod > 200000 && enableShutter)
    {
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

void IRAM_ATTR debugTask(void *pvParameters)
{
  static int startup = 0;
  static int debugcnt1;
  static int debugcnt2;
  static int indx = 0;
  for (;;)
  {



        // Serial.println(ledtimer);
    vTaskDelay(20);
    // // ESP_LOGI(ledDr, "led timer value: %u", rtc_clk_apb_freq_get());
    // UBaseType_t stack0 = uxTaskGetStackHighWaterMark(ioTASKHANDLE);
    // UBaseType_t stack1 = uxTaskGetStackHighWaterMark(motContinuousHandle);
    // UBaseType_t stack2 = uxTaskGetStackHighWaterMark(singleFrame);
    // UBaseType_t stack3 = uxTaskGetStackHighWaterMark(ledDraw);
    // UBaseType_t stack4 = uxTaskGetStackHighWaterMark(shutterPotTranslate);
    // UBaseType_t stack5 = uxTaskGetStackHighWaterMark(motorSlewRead);
    // UBaseType_t stack6 = uxTaskGetStackHighWaterMark(FPSactor);
    // UBaseType_t stack7 = uxTaskGetStackHighWaterMark(externalControlParse);
    // UBaseType_t stack8 = uxTaskGetStackHighWaterMark(internalSerialRX);
    // UBaseType_t stack9 = uxTaskGetStackHighWaterMark(internalSerialTX);
    // UBaseType_t stack10 = uxTaskGetStackHighWaterMark(readControls);
    // UBaseType_t stack11 = uxTaskGetStackHighWaterMark(debugPrinter);
    // ESP_LOGI(ioTH, "Remaining stack: %u bytes", (unsigned int) stack0);
    // ESP_LOGI(motCont, "Remaining stack: %u bytes", (unsigned int) stack1);
    // ESP_LOGI(sFF, "Remaining stack: %u bytes", (unsigned int) stack2);
    // ESP_LOGI(ledDr, "Remaining stack: %u bytes", (unsigned int) stack3);
    // ESP_LOGI(shPot, "Remaining stack: %u bytes", (unsigned int) stack4);
    // ESP_LOGI(slew, "Remaining stack: %u bytes", (unsigned int) stack5);
    // ESP_LOGI(fpsCalc, "Remaining stack: %u bytes", (unsigned int) stack6);
    // ESP_LOGI(extCont, "Remaining stack: %u bytes", (unsigned int) stack7);
    // ESP_LOGI(iRx, "Remaining stack: %u bytes", (unsigned int) stack8);
    // ESP_LOGI(iTx, "Remaining stack: %u bytes", (unsigned int) stack9);
    // ESP_LOGI(controlR, "Remaining stack: %u bytes", (unsigned int) stack10);
    // ESP_LOGI(db, "Remaining stack: %u bytes", (unsigned int) stack11);
    // ESP_LOGI(genPurp, "-----------------------------");

    // ESP_LOGI(controlR, "LED: %u", ledPotVal);
    //         ESP_LOGI(controlR, "LEDCONV: %u", ledB);

    // ESP_LOGI(controlR, "MOT: %u", motPotVal);
    //     ESP_LOGI(controlR, "MOTCONV: %u", MPotEstimate);

    // ESP_LOGI(controlR, "BLADES: %u", shutBladesPotVal);
    //     ESP_LOGI(controlR, "BLADESCONV: %u", shutBladesVal);

    // ESP_LOGI(controlR, "ANGLE: %u", shutAnglePotVal);
    // ESP_LOGI(controlR, "ANGLECONV: %u", shutAngEstimate);

        // ESP_LOGI(ledDr, "led angle: %u", ang);
    // ESP_LOGI(ledDr, "shutter angle post float conversion: %u", (uint32_t)shAngF);
    //     ESP_LOGI(ledDr, "led in queue: %u", ledBright);
    // uint64_t timer1;
    // gptimer_get_raw_count(ledtick, &timer1);
    // ESP_LOGI(ledDr, "led timer value: %u", (uint16_t)timer1);









    //  if (debug == 1) {
    //     uint16_t *angle2 = &ang;
    // uint32_t *Smap = &shutterMap[*angle2];


              // for (int indx = 0; indx < 1024; indx+32)
              // {
              //       uint32_t *Smap = &shutterMap[indx];

              //         Serial.print(*Smap);
              //         Serial.flush();

              // }

        // indx++;

        // // uint32_t *Smap2 = &shutterMap[indx];
        // if (indx == ang)
        // {
        //   if (shutterMap[indx] == 1)
        //   {
        //     Serial.print("O");
        //   }
        //   else if (shutterMap[indx] == 0)
        //   {
        //     Serial.print(" ");

        //   }

        // } else
        // {
        //     if (shutterMap[indx] == 1)
        //     {
        //         Serial.print(" ");
        //     } else if (shutterMap[indx] == 0)
        //     {
        //         Serial.print("X");
        //     }
        // }

        // if (indx>1024)
        // {
        //       Serial.println("");
        //       indx = 0;

        // }
    
  }
}



  // static int shutBladesVal = 0;
  // static int shutAngleVal = 0;


void readShutterControls(void *parameter)
{
  


  for (;;)
  {
    vTaskDelay(500/portTICK_PERIOD_MS);
// ESP_LOGI("shutterRead", "READ SHUTTER POT");

    adc_oneshot_read(adc_handle, shutBladesPotPin, &shutBladesPotVal);
    adc_oneshot_read(adc_handle, shutAnglePotPin, &shutAnglePotVal);
        // shutBladesPotVal = 2048;
        // shutAnglePotVal = 2048;
#if (enableSlewPots)
    motSlewVal = motSlewPotKalman.updateEstimate(analogRead(motSlewPotPin));
    // delay(2);
    ledSlewVal = ledSlewPotKalman.updateEstimate(analogRead(ledSlewPotPin));
// delay(2);
#endif

#if (enableShutterPots && enableShutter)
    if (musicMode == 0)
    {
// shutPotEstimate = shutBladesPotKalman -> UpdateRefMeasurement((float)1.0, (float)shutBladesPotVal, kalmanMEA, kalmanQ);      
shutPotEstimate = shutBladesPotVal;      

if (shutPotEstimate < 800)
      {
        shutBladesVal = 1;
      }
      else if (shutPotEstimate < 2500)
      {
        shutBladesVal = 2;
      }
      else
      {
        shutBladesVal = 3;
      }
      // shutAngEstimate = shutAnglePotKalman -> UpdateRefMeasurement((float)1.0, (float)shutBladesPotVal, kalmanMEA, kalmanQ);
            shutAngEstimate = shutAnglePotVal;

      shutAngEstimate = map(shutAngEstimate, 0, 4090, 1, 100); // map ADC input to range of shutter angle
    }
    else if (musicMode == 1)
    {
      shutBladesPotVal = map(CC2ProjBlades, 0, 100, 0, 4095);
      // inject values during Music Mode
    }

    xQueueSend(q_shutterBlade, &shutBladesVal, 2); // send shutter blade pot value to shutter task via queue
    xQueueSend(q_shutterAngle, &shutAngEstimate, 2);  // send shutter angle pot value to shutter task via queue
                                                   // clip values

#endif
  }
}