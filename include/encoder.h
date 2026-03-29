// void shutterQueue(void *pvParameters) {
//     byte shutterBlades;
//     float shutterAngle;
//     for(;;) {
//     xQueueReceive(q_shutterBlade, &shutterBlades, portMAX_DELAY);
//     xQueueReceive(q_shutterAngle, &shutterAngle, portMAX_DELAY);
//     xQueueSend(q_shutterBlade, &shutterBlades, portMAX_DELAY);
//     xQueueSend(q_shutterAngle, &shutterAngle, portMAX_DELAY);
// }

void readEncoder(void *pvParameters)
{
    static int ang;
    for (;;)
    {
        if (xSemaphoreTake(spiRead, portMAX_DELAY) == pdTRUE)
        {
        xQueueReset(q_spiRead); // clear the queue to ensure we don't have old readings sitting in there
        ang = map(as5047.readAngle(), 0, 360, 0, 100);
        xQueueSend(q_spiRead, &ang, sizeof(ang)); // send updated shutter map to shutter task via queue
        // vTaskDelay(10 / portTICK_PERIOD_MS);
        xSemaphoreGive(spiRead); // give the semaphore back after updating shared variables
        vTaskDelay(20 / portTICK_PERIOD_MS); // adjust this delay as needed to control the rate of reading the encoder and updating the shutter map. if we're seeing performance issues, we might want to increase this delay to reduce the frequency of SPI reads and shutter map updates.

        }
    }
}

void fixCount()
{
    encCount = map(as5047.readAngle(), 0, 360, 0, 100);
    Serial.println("   (Updated count via SPI)");
}

// fill shutterMap array with boolean values to control LED state at each position of shutter rotation
void updateShutterMap(void *parameter)
{ // move to core 0 with related interrupts

        static int shutBladesVal = 0;
    static float shutAngleVal = 0;
     static bool  *shutterMap[countsPerFrame];  // array to hold the on/off state of the shutter for each count position


    for (;;)
    {
        // if (xSemaphoreTake(shutterMapping, portMAX_DELAY) == pdTRUE)
        { // take the semaphore to ensure exclusive access to shared variables
            // Serial.println("Updating Shutter Map");
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
    //   xQueueSend(q_shutterBlade, &shutBladesVal, 5);  // send shutter blade pot value to shutter task via queue  

  shutAngleVal = mapf(shutAnglePotVal, 0, 4090, 0.1, 1.0);  // map ADC input to range of shutter angle
  shutAngleVal = constrain(shutAngleVal, 0.1, 1.0); 
    //   xQueueSend(q_shutterAngle, &shutAngleVal, 5);  // send shutter angle pot value to shutter task via queue
        // clip values
#endif
            // xQueueReceive(q_shutterBlade, &sB, portMAX_DELAY);
            // xQueueReceive(q_shutterAngle, &sA, portMAX_DELAY);
            Serial.println("Update ShutterMap");
            //  shutterBlades: number of virtual shutter blades (must be > 0)
            //  shutterAngle: ratio between on/off for each shutter blade segment (0.5 = 180d)
            if (shutBladesVal < 1)
            {
                shutBladesVal = 1;                       // it would break if set to 0
                shutAngleVal = constrain(shutAngleVal, 0.0, 1.0); // make sure it's 0-1
            }
            for (int myBlade = 0; myBlade < shutBladesVal; myBlade++)
            {
                int countOffset = myBlade * (countsPerFrame / shutBladesVal);
                for (int myCount = 0; myCount < countsPerFrame / shutBladesVal; myCount++)
                {
                    if (myCount < countsPerFrame / shutBladesVal * (1.0 - shutAngleVal))
                    {
                        shutterMap[myCount + countOffset] = (bool*)false;
                        // xQueueSend(q_shutterMap, &"0", sizeof(shutterMap)); // send updated shutter map to shutter task via queue
                    }
                    else
                    {
                        shutterMap[myCount + countOffset] = (bool*)true;
                        // xQueueSend(q_shutterMap, &"1", sizeof(shutterMap)); // send updated shutter map to shutter task via queue
                    }
                }
            }
            xQueueSend(q_shutterMap, &shutterMap[countsPerFrame], sizeof(shutterMap)); // send updated shutter map to shutter task via queue

            // xSemaphoreGive(shutterMapping); // give the semaphore back after updating shared variables
        }
    }
}
    void IRAM_ATTR indexISR(void *arg)
    {

        if (xSemaphoreTakeFromISR(spiRead, NULL) == pdTRUE)
        //higher prio than readEncoder, but regardless it means we've hit the index and need to be reset
        {
            EncIndexCount = 1;

            xSemaphoreGiveFromISR(spiRead, NULL); // give the semaphore back after updating shared variables}
        }
    }

    void IRAM_ATTR pinChangeISR(void *arg)
    {

        // first store last value

        static int encCount; // current rotary count
        static int encCountOld; // previous rotary count, used to determine direction

        if (xSemaphoreTakeFromISR(spiRead, &xHigherPriorityTaskWoken) == pdTRUE)
        { // take the semaphore to ensure exclusive access to shared variables

            taskENTER_CRITICAL_ISR(&shutterFunctionLock); // we are in the real meat + potatoes
            // this makes sure that this interrupt is seen through
                  if(debug)
  {
    intr1_hasRun++;
  xQueueSendFromISR(q_intr1_hasRun, &intr1_hasRun, &xHigherPriorityTaskWoken);  // send a value to the shutter task to indicate that the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue

  }

            encCountOld = encCount;
            xQueueReceiveFromISR(q_spiRead, &encCount, &xHigherPriorityTaskWoken); // read the current count from the PCNT unit
            
            if (encCountOld < encCount)
            {

                if (EncIndexCount == 1)
                { // reset counter on 'middle" transition during index condition
                    frame++;
                    FPSframe = 1000000.0 / framePeriod; // update FPS calc based on period between each frame
                    framePeriod = 0;
                    EncIndexCount = 0; // reset index count so we can detect the next index trigger
                }
                // but if the index pin hasnt been triggered, we run an alternative function
                else
                {
                    FPScount = 10000.0 / countPeriod; // update FPS calc based on period between the 100 encoder counts
                    countPeriod = 0;
                    motModeReal = 1; // mark that we're running forwards
                }
            }
            else if (encCountOld > encCount)
            {
                //   //     // moving backwards ...
                if (EncIndexCount == 1)
                {   // reset counter on 'middle" transition during index condition
                    frame--;
                    FPSframe = 1000000.0 / framePeriod; // update FPS calc based on period between each frame
                    framePeriod = 0;
                    EncIndexCount = 0; // reset index count so we can detect the next index trigger

                }
                else
                {
                    // normal backwards count
                    FPScount = 10000.0 / countPeriod; // update FPS calc based on period between the 100 encoder counts
                    countPeriod = 0;
                    motModeReal = -1; // mark that we're running backwards
                }
            }
            xQueueSendFromISR(q_actualShutterMap, &encCount, &xHigherPriorityTaskWoken); // read the current count from the PCNT unit again to ensure we have the most up-to-date value for any subsequent interrupts
            send_LEDC(); // actual update code is abstracted so it can be run in different contexts

            taskEXIT_CRITICAL_ISR(&shutterFunctionLock); // exit critical section
            xSemaphoreGiveFromISR(spiRead, &xHigherPriorityTaskWoken);   // give the semaphore back after updating shared variables
        }
    }

    // send info to the LEDC peripheral to update LED PWM (abstracted here because it's called from loop or ISR)

    // check for encoder magnet proximity
    void as5047MagCheck(void *pvParameters)
    {
        for (;;)
        {

            // read magnet AGC data from sensor registers
            readDataFrame = as5047.readRegister(DIAGAGC_REG);
            Diaagc diaagc;
            diaagc.raw = readDataFrame.values.data;
            // Serial.println(diaagc.values.magl);

            // check result for magnet errors and update global var
            if (diaagc.values.magh || diaagc.values.magl)
            {
                as5047MagOK = 0;
            }
            else
            {
                as5047MagOK = 1;
            }

            // take action if global var has changed
            if (as5047MagOK_old != as5047MagOK)
            {
                if (as5047MagOK)
                {
                    Serial.println("Magnet OK");
                }
                else
                {
                    Serial.println("Magnet ERROR");
                }
                as5047MagOK_old = as5047MagOK;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }