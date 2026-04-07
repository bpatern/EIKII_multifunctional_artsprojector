void updateMotor(void *pvParameters) { 


  for(;;) {
  
#if (enableSlewPots)
  motSlewVal = map(motSlewVal, 0, 4095, motSlewMin, motSlewMax);  // turn slew val pot into ms ramp time
#endif
  // motAvg.update();  // Motor slewing managed by Ramp library
  // if knobs have changed sufficiently, calculate new slewing ramp time
  if (abs(motSlewVal - motSlewValOld) >= 10 || abs(motPotFPS - motPotFPSOld) >= 10) {
    //Serial.println("(MOT RAMP CALC)");
    // motAvg.go(motPotFPS, motSlewVal);  // set next ramp interpolation in ms
    motSlewValOld = motSlewVal;
    motPotFPSOld = motPotFPS;
  }

  // float FPStemp = motAvg.getValue() / 100.0;  // use slewed value for target FPS (dividing by 100 to get floating point FPS)

  // Add more natural scaling when we translate from pot to actual FPS
  // These values may be negative, but fscale only handles positive values, so...

  // if (FPStemp < 0.0) {
  //   // negative FPS values
  //   FPStemp = FPStemp * -1.0;                                       // make it positive before fscale function
  //   FPStarget = (fscale(0.0, 24.0, 0.0, 24.0, FPStemp, 3) * -1.0);  // reverse log scale and turn it negative again
  // } else {
  //   FPStarget = fscale(0.0, 24.0, 0.0, 24.0, FPStemp, 3);  // number is positive so just reverse log scale it
  // }

 

  if (opticalPrinter == 1) {
    pullDownPos = 20;
  } else if (opticalPrinter == 0) {
    pullDownPos = 20;
  }  

     vTaskDelay(50 / portTICK_PERIOD_MS);

    }
}


