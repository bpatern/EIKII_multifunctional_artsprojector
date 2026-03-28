void updateLed(void *pvParameters) { 
    int shutBladesVal = 0;
    float shutAngleVal = 0;
    int ledSlewVal = 0;
    int ledPotVal = 0;
    int ledBright = 0;


    for(;;) {
        xQueueReceive(ledPot, &ledPotVal, portMAX_DELAY);  // receive motor pot value from UI task via queue
        xQueueReceive(q_shutterBlade, &shutBladesVal, portMAX_DELAY);
        xQueueReceive(q_shutterAngle, &shutAngleVal, portMAX_DELAY);  // receive shutter blade and angle values from UI task via queue
  //noInterrupts();
    Serial.println("runLEDREAD");


  if (abs(shutBladesVal != shutBladesValOld) || abs(shutAngleVal - shutAngleValOld) >= 0.05) {
    if (enableShutter == 1) {
      //Serial.println("(SHUTTERMAP SHUTTER POTS)");
    }
    shutBladesValOld = shutBladesVal;
    shutAngleValOld = shutAngleVal;
  }

#if (enableSlewPots)
  ledSlewVal = map(ledSlewVal, 0, 4095, ledSlewMin, ledSlewMax);  // turn slew val pot into ms ramp time
#endif
  ledAvg.update();  // LED slewing managed by Ramp library
  // if knobs have changed sufficiently, calculate new slewing ramp time
  if (musicMode == 0) {
    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledPotVal - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledPotVal, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledPotVal;
    }

    // set brightness to slewed version of pot value (mapped/clipped because kalman filter sometimes doesn't allow us to reach min/max)
    ledBright = map(ledAvg.getValue(), 40, 4045, ledMin, ledMax);
    ledBright = constrain(ledBright, 0, 4095);
  } else if (musicMode == 1) {

    if (abs(ledSlewVal - ledSlewValOld) >= 50 || abs(ledPotVal - ledPotValOld) >= 50) {
      //Serial.println("(LED UPDATE SLEW)");
      ledAvg.go(ledPotVal, ledSlewVal);  // set next ramp interpolation in ms
      ledSlewValOld = ledSlewVal;
      ledPotValOld = ledPotVal;
    }

    ledBright = map(ledAvg.getValue(), 0, 100, ledMin, ledMax);

    ledBright = constrain(ledBright, 0, 4095);
  }

  // SAFE MODE CHECK
  if (safeMode == 1) {
    if (motSingle != 0) {
      // We're in one of the single modes, so ...
      ledBright = ledBright * safeMin;  // dim lamp to min safe brightness immediately
    } else {
      safeSpeedMult = fscale(4, 24.0, safeMin, 1.0, abs(FPSrealAvg), 0);  // safety multiplier for FPS (with optional nonlinear scaling)
      safeSpeedMult = constrain(safeSpeedMult, safeMin, 1.0);             // clip values
      ledBright = ledBright * safeSpeedMult;                              // decrease brightness to prevent film burns
    }
  }

  xQueueSend(q_ledBright, &ledBright, portMAX_DELAY);  // send LED brightness value to commander task via queue for potential forwarding to remote control

  if ((opticalPrinter == 1 || scanFlag == 1) && capFlag == 1) {
    ledBright = ledBright;
    shutterQueue(1, 1.0);  // force open shutter for single framing
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

  if (debugLed) {
    Serial.print("LED Slew: ");
    Serial.print(ledSlewVal);
    Serial.print(", LED Pot: ");
    Serial.print(ledPotVal);
    Serial.print(", Spd Mult: ");
    Serial.print(safeSpeedMult);
    Serial.print(", LED Bright: ");
    Serial.print(ledBright);
    Serial.print(", Shutter Blades: ");
    Serial.print(shutBladesVal);
    Serial.print(", Shutter Angle: ");
    Serial.println(shutAngleVal);
  }
//   // at slow speeds OR if the shutter is fully open OR shutter disabled, update the LED PWM directly because ISR isn't firing often (or at all)
//   if (countPeriod > 50000 || shutAngleVal == 1.0 || !enableShutter || musicMode == 1) {
//     send_LEDC();
//   }
vTaskDelay(50 / portTICK_PERIOD_MS);}
}