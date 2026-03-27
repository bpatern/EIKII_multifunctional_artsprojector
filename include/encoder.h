void readEncoder(void *pvParameters) {

  for (;;)
  {
  as5047absAngle = map(as5047.readAngle(), 0, 360, 0, 100);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}