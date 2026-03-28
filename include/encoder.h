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
  count = map(as5047.readAngle(), 0, 360, 0, 100);
  Serial.println("   (Updated count via SPI)");
}

// fill shutterMap array with boolean values to control LED state at each position of shutter rotation
void updateShutterMap(void * parameter) { //move to core 0 with related interrupts
 int sB = 0;
 float sA = 0;
 volatile bool shutterMap[countsPerFrame];  // array to hold the on/off state of the shutter for each count position

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