void readUI(void *pvparemeters) { 
    int motPotVal = 0;  // current value of Motor pot (not necessarily the current speed since we might be ramping toward this value)
int ledPotVal = 0;  // current value of LED pot (not necessarily the current LED brightness since we might be fading or strobing)
int shutBladesPotVal;
int shutAnglePotVal;
int shutBladesVal;
float shutAngleVal;



    for (;;) {
        if (xSemaphoreTake(controlLock, portMAX_DELAY) == pdTRUE) {  // take control lock to read UI values and update shared variables that are used in other tasks and ISRs
        

  Serial.println("runUIREAD");


  //if (onboardcontrol == 1) {

#if (enableButtons)
  buttonA.loop();  // update button managed by Bounce2 library
  buttonB.loop();  // update button managed by Bounce2 library


#endif
    motPotVal = motPotKalman.updateEstimate(analogRead(motPotPin));

    xQueueSend(motPot, &motPotVal, portMAX_DELAY);  // send motor pot value to motor task via queue

      ledPotVal = ledPotKalman.updateEstimate(analogRead(ledPotPin));
    xQueueSend(ledPot, &ledPotVal, portMAX_DELAY);  // send LED pot value to LED task via queue


#if (enableSlewPots)
  motSlewVal = motSlewPotKalman.updateEstimate(analogRead(motSlewPotPin));
  // delay(2);
  ledSlewVal = ledSlewPotKalman.updateEstimate(analogRead(ledSlewPotPin));
  // delay(2);
#endif

#if (enableShutterPots && enableShutter)
  if (musicMode == 0) {
    shutBladesPotVal = shutBladesPotKalman.updateEstimate(analogRead(shutBladesPotPin));
    
    // delay(2);
  } else if (musicMode == 1) {
    shutBladesPotVal = map(CC2ProjBlades, 0, 100, 0, 4095);
    //inject values during Music Mode
  }
  shutAnglePotVal = shutAnglePotKalman.updateEstimate(analogRead(shutAnglePotPin));
  // delay(2);
  // using custom scaling for shutBladesPotVal to make control feel right
  if (shutBladesPotVal < 800) {
    shutBladesVal = 1;
  } else if (shutBladesPotVal < 2500) {
    shutBladesVal = 2;
  } else {
    shutBladesVal = 3;
  }
      xQueueSend(q_shutterBlade, &shutBladesVal, portMAX_DELAY);  // send shutter blade pot value to shutter task via queue  

  shutAngleVal = mapf(shutAnglePotVal, 0, 4090, 0.1, 1.0);  // map ADC input to range of shutter angle
  shutAngleVal = constrain(shutAngleVal, 0.1, 1.0); 
      xQueueSend(q_shutterAngle, &shutAngleVal, portMAX_DELAY);  // send shutter angle pot value to shutter task via queue
        // clip values
#endif

#if (enableSafeSwitch)
  safeMode = digitalRead(safeSwitch);
#endif
xSemaphoreGive(controlLock);  // give back control lock after updating shared variables that are used in other tasks and ISRs
vTaskDelay(100 / portTICK_PERIOD_MS);}
    } 
}