// void shutterQueue(void *pvParameters) {
//     byte shutterBlades;
//     float shutterAngle;
//     for(;;) {
//     xQueueReceive(q_shutterBlade, &shutterBlades, portMAX_DELAY);
//     xQueueReceive(q_shutterAngle, &shutterAngle, portMAX_DELAY);
//     xQueueSend(q_shutterBlade, &shutterBlades, portMAX_DELAY);
//     xQueueSend(q_shutterAngle, &shutterAngle, portMAX_DELAY);
// }


void readEncoder(void *pvParameters) {

  for (;;)
  {
  as5047absAngle = map(as5047.readAngle(), 0, 360, 0, 100);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void fixCount() {
  encCount = map(as5047.readAngle(), 0, 360, 0, 100);
  Serial.println("   (Updated count via SPI)");
}

// fill shutterMap array with boolean values to control LED state at each position of shutter rotation
void updateShutterMap(void * parameter) { //move to core 0 with related interrupts
 int sB = 0;
 float sA = 0;

    for(;;){

    xQueueReceive(q_shutterBlade, &sB, portMAX_DELAY);
    xQueueReceive(q_shutterAngle, &sA, portMAX_DELAY);
  //Serial.print("Update ShutterMap");
  // shutterBlades: number of virtual shutter blades (must be > 0)
  // shutterAngle: ratio between on/off for each shutter blade segment (0.5 = 180d)
  if (sB < 1) {
    sB = 1;          // it would break if set to 0
  sA = constrain(sA, 0.0, 1.0);  // make sure it's 0-1
  }
  for (int myBlade = 0; myBlade < sB; myBlade++) {
    int countOffset = myBlade * (countsPerFrame / sB);
    for (int myCount = 0; myCount < countsPerFrame / sB; myCount++) {
      if (myCount < countsPerFrame / sB * (1.0 - sA)) {
        // shutterMap[myCount + countOffset] = 0;
        xRingbufferSend(q_shutterMap, &"0", sizeof(shutterMap), portMAX_DELAY);  // send updated shutter map to shutter task via queue

      } else {
        // shutterMap[myCount + countOffset] = 1;
        xRingbufferSend(q_shutterMap, &"1", sizeof(shutterMap), portMAX_DELAY);  // send updated shutter map to shutter task via queue


      }

    }
  }
}
}

void IRAM_ATTR indexISR(void *arg) {

    if (xSemaphoreTakeFromISR(encoderMutex, NULL) == pdTRUE) {
        EncIndexCount = 1;
        encPos = 0;
        encCountOld = 0;
        xSemaphoreGiveFromISR(encoderMutex, NULL);  // give the semaphore back after updating shared variables}
    }
}

void IRAM_ATTR pinChangeISR(void *arg) {
    //first store last value

  static int encCount;                        // current rotary count


  if (xSemaphoreTakeFromISR(encoderMutex, NULL) == pdTRUE) {  // take the semaphore to ensure exclusive access to shared variables

    taskENTER_CRITICAL_ISR(&shutterFunctionLock); //we are in the real meat + potatoes
    //this makes sure that this interrupt is seen through


  //check for changes
    pcnt_get_counter_value((pcnt_unit_t)0, encPos);  // read the current count from the PCNT unit


  //since we've read the encoder again, the getTicks value could be different than what we stored in encCountOld before taking a new reading.
//   if (encPos > encCountOld) {
  if (encPos > 0) {

    //use this little function to see if we are going frowards or backwards. for example,
    //if encCountOld is 75 and then enc.getTicks reads 76, then enc.getTicks has a greater value
    //it means we are going forwards. 76-75 = 1
    //if its the same 75-75 
    //then it will be 0! and we are likely weither not moving OR the interrupt somehow has been running without the sensor movement.

    //   //     // at index
    if (EncIndexCount == 1) {  // reset counter on 'middle" transition during index condition

    //internal frame count should only change at index pin trigger. so, we add a frame to that integer with the handy ++
      frame++;

      //this takes care of fps calculation (for now)
      FPSframe = 1000000.0 / framePeriod;  // update FPS calc based on period between each frame
      framePeriod = 0;
    encCount = 0;

      pcnt_counter_clear((pcnt_unit_t)0);  // reset counter to zero since because the counter would want to keep increasing past 100
    } 
    //but if the index pin hasnt been triggered, we run an alternative function
    else 
    {
      // normal forwards count
      FPScount = 10000.0 / countPeriod;  // update FPS calc based on period between the 100 encoder counts
      countPeriod = 0;
      motModeReal = 1;  // mark that we're running forwards
    }
    //enccount is a "true" representation of the virtual shutter blades since it rolls over and 
    encCount++;
    // this handles the ca
    if (encCount > countsPerFrame - 1) {
      encCount = 0;
    }

    //this is the backwards scenario. 
    //like the previous example, if 74 is the value gotten from enc.getTicks(), then 74-75(the hypothetical value in encCountOld) is -1
    //now we know we're going backwards
  } else if (encPos < 0){
    //   //     // moving backwards ...
    if (EncIndexCount == 1) {  // reset counter on 'middle" transition during index condition
      // at index
      // count = 0;
        frame--;


      FPSframe = 1000000.0 / framePeriod;  // update FPS calc based on period between each frame
      framePeriod = 0;
    } else {
      // normal backwards count
      FPScount = 10000.0 / countPeriod;  // update FPS calc based on period between the 100 encoder counts
      countPeriod = 0;
      motModeReal = -1;  // mark that we're running backwards
    }
    encCount--;
    // wrap around the circle instead of using negative steps
    if (encCount < 0) {
      encCount = countsPerFrame - 1;
    }
  }



//   bool shutterState = map;  // copy shutter state to local variable in case it changes during the ISR execution (not possible?)
//   if (shutterState != shutterStateOld) {  // only update LED if shutter state changes (not every step)
//       // the next thing to happen is

    send_LEDC();                          // actual update code is abstracted so it can be run in different contexts
    //check led.h for the above function. when it is done, it will return here!
    
//   } else if (shutterState == shutterStateOld)
//   {

//   }
//   shutterStateOld = shutterState;  // store to global variable for next time


    taskEXIT_CRITICAL_ISR(&shutterFunctionLock); //exit critical section
    xSemaphoreGiveFromISR(encoderMutex, NULL);  // give the semaphore back after updating shared variables
  }

}


// send info to the LEDC peripheral to update LED PWM (abstracted here because it's called from loop or ISR)



// check for encoder magnet proximity
void as5047MagCheck(void *pvParameters) { for(;;) {

  // read magnet AGC data from sensor registers
  readDataFrame = as5047.readRegister(DIAGAGC_REG);
  Diaagc diaagc;
  diaagc.raw = readDataFrame.values.data;
  //Serial.println(diaagc.values.magl);

  // check result for magnet errors and update global var
  if (diaagc.values.magh || diaagc.values.magl) {
    as5047MagOK = 0;
  } else {
    as5047MagOK = 1;
  }

  // take action if global var has changed
  if (as5047MagOK_old != as5047MagOK) {
    if (as5047MagOK) {
      Serial.println("Magnet OK");
      fixCount();  // magnet is back after loss, so fix the count using SPI
    } else {
      Serial.println("Magnet ERROR");
      fixCount();  // magnet is back after loss, so fix the count using SPI
    }
    as5047MagOK_old = as5047MagOK;
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}