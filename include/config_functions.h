Encoder enc(EncA, EncB, FULLQUAD, 0);
const int FPSnumReadings = 3;      // how many FPS readings to average together
float FPSAvgArray[FPSnumReadings]; // the FPS readings array
int FPSAvgReadIndex = 0;           // the index of the current reading
float FPSAvgTotal = 0;             // total of all readings

// Ramp library setup
rampInt motAvg; // ramp object for motor speed slewing
rampInt ledAvg; // ramp object for LED brightness slewing
// Simple Kalman Filter Library Setup
// see tuning advice here: https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html
// and this playlist: https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
float kalmanMEA = 2;   // "measurement noise estimate" (larger = we assmume there is more noise/jitter in input)
float kalmanQ = 0.010; // "Process Noise" AKA "gain" (smaller = more smoothing but more latency)

SimpleKalmanFilter motPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter ledPotKalman(kalmanMEA, kalmanMEA, kalmanQ);

#if (enableSlewPots) // if these pots are enabled in the config file,
SimpleKalmanFilter motSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter ledSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
#endif // now we check for the next option, which asks about the

#if (enableShutterPots) // are you using shutter blade angle controls?
SimpleKalmanFilter shutBladesPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter shutAnglePotKalman(kalmanMEA, kalmanMEA, kalmanQ);
#endif

#if (enableButtons)
// Bounce2 Library setup for buttons
Button2 buttonA;
Button2 buttonB;

bool buttonAstate = 0;
bool buttonBstate = 0;
#endif

#if (enableStatusLed)
// NeoPixel library setup
#define NUMPIXELS 1 // How many NeoPixels are attached?

Adafruit_NeoPixel pixels(NUMPIXELS, NeoPixelPin, NEO_RGB + NEO_KHZ800);
// last argument should match your LED type. Add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (5mm LEDS, v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
#endif

#if (useAS5047)
// Start connection to the AS5047sensor.
AS5X47 as5047(EncCSN);
// will be used to read data from the magnetic encoder
ReadDataFrame readDataFrame;
int as5047MagOK = 0; // status of magnet near AS5047 sensor
int as5047MagOK_old = 0;

#endif

// prepare to intialize interrupt tasks
TaskHandle_t pcISR;
TaskHandle_t ledCshutter;

// configure timers
elapsedMicros framePeriod, countPeriod;
elapsedMillis timerUI, serialTimerUI, motSingleRunTimer, motModeMidiTimer, midiParseTimer; // MS since last time we checked/updated the user interface

void createTasks() {

        if(enableShutter == 1)
    {
        q_shutterBlade = xQueueCreate(20, sizeof(int*));
        q_shutterAngle = xQueueCreate(20, sizeof(float*)); //shutter angle is never a float in the program which is why i changed it
        q_ledBright = xQueueCreate(20, sizeof(int*));
        q_shutterMap = xRingbufferCreate(100, RINGBUF_TYPE_NOSPLIT);
            xTaskCreatePinnedToCore(
                updateShutterMap,
                "updateShutter",
                3000,
                NULL,
                12,
                NULL,
                1
            );

    }

    xTaskCreatePinnedToCore(
    serialReadTask,
    "serialReadTask",
    5000,
    NULL,
    5,
    NULL,
    1
  );

  motPot = xQueueCreate(2, sizeof(int*));
    ledPot = xQueueCreate(2, sizeof(int*));
  xTaskCreatePinnedToCore(
    readUI,
    "readUI",
    5000,
    NULL,
    16,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    updateMotor,
    "updateMotor",
    4000,
    NULL,
    10,
    NULL,
    1
  );

    xTaskCreatePinnedToCore(
        updateLed,
        "updateLed",
        5000,
        NULL,
        12,
    NULL,
    1
  );

      xTaskCreatePinnedToCore(
    as5047MagCheck,
    "as5047MagCheck",
    3000,
    NULL,
    12,
    NULL,
    0
  );

        xTaskCreatePinnedToCore(
    calcFPS,
    "calcFPS",
    3000,
    NULL,
    12,
    NULL,
    1
  );



          xTaskCreatePinnedToCore(
    readEncoder,
    "readEncoder",
    3000,
    NULL,
    20,
    NULL,
    0
    );
    


    
        // xTaskCreatePinnedToCore(
    // pcISRCORE,
    // "pcISR",
    // 10000,
    // NULL,
    // 24,
    // &pcISR,
    // 0);

}
void interfaceConfig()
{
    if (enableStatusLed)
    {
          pixels.begin();                // INITIALIZE NeoPixel object
            updateStatusLED(0, 30, 0, 0);  // start with LED red while booting
        if (statusLedColor == 1) {
          updateStatusLED(0, 18, 16, 10);  // white LED at idle
        }

    }
    if (enableSafeSwitch)
    {
        pinMode(safeSwitch, INPUT_PULLUP);
    }

    if (enableMotSwitch)
    {
        pinMode(motDirFwdSwitch, INPUT_PULLUP);
        pinMode(motDirBckSwitch, INPUT_PULLUP);
    }
    if (enableButtons)
    {
        buttonA.begin(buttonApin, INPUT_PULLUP, true);
        buttonA.setDebounceTime(20);
        buttonA.setPressedHandler(pressed);
        buttonA.setReleasedHandler(released);

        buttonB.begin(buttonBpin, INPUT_PULLUP, true);
        buttonB.setDebounceTime(20);
        buttonB.setPressedHandler(pressed);
        buttonB.setReleasedHandler(released);

        if (digitalRead(buttonApin) == 0)
        {
            // Program the ESC settings if user holds down buttonA during startup

            ESCprogram();
            while (1)
                ; // don't continue setup since the ESC needs to be rebooted before we can continue
        }
    }
}

void as5047Config()
{
    abOld = count = countOld = 0;
    // Set rotation direction (see AS5047 datasheet page 17)
    Settings1 settings1;
    settings1.values.dir = encoderDir;
    as5047.writeSettings1(settings1);
    // Set ABI output resolution (see AS5047 datasheet page 19)
    // (pulses per rev: 5 = 50 pulses, 6 = 25 pulses, 7 = 8 pulses)
    Settings2 settings2;
    settings2.values.abires = 6; // 25 pulses per rev = 100 transitions. This is what we want.
    as5047.writeSettings2(settings2);
    // Disable ABI output when magnet error (low or high) exists (see AS5047 datasheet page 24)
    Zposl zposl;
    Zposm zposm;
    zposl.values.compLerrorEn = 1;
    zposl.values.compHerrorEn = 1;
    as5047.writeZeroPosition(zposm, zposl);

    attachInterrupt(EncA, pinChangeISR, CHANGE);
    attachInterrupt(EncB, pinChangeISR, CHANGE);
    if (enc.init()) {
      Serial.println("PWM Encoder GOOD");
    } else {
      Serial.println("PWM Encoder ERROR");
    }

      fixCount();                                     // at startup we don't know the absolute position via ABI, so ask SPI

}

void mathConfig() {
      for (int thisReading = 0; thisReading < FPSnumReadings; thisReading++) {
    FPSAvgArray[thisReading] = 0;
  }

}

void motorConfig() {
      // Motor PWM setup
  ledcSetup(motPWMChannel, motPWMFreq, motPWMRes);  // configure motor PWM function using LEDC channel
  ledcAttachPin(escPin, motPWMChannel);             // attach the LEDC channel to the GPIO to be controlled

  //it's important to send the ESC a "0" speed signal (1500us) whenever the motor is stopped. Otherwise the ESC goes into "failsafe" mode thinking that our RC car has lost contact with the TX!
  ledcWrite(motPWMChannel, (1 << motPWMRes) * 1500 / motPWMPeriod);  // duty = # of values at current res * US / pulse period
  Serial.print("Sending neutral signal to ESC...");
  delay(4000);
  Serial.println("Done");
}

void digitalShutterConfig() {
    //   shutterQueue(shutterBlades, shutterAngle);  //generate initial shutter map ... (1, 0.05 = 1 PPF and narrowest shutter angle)
    
}

void mechanicalShutterConfig() {
      // Motor PWM setup for mechanical shutter control (same as motorConfig but with different channel and pin)
    ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);   // configure LED PWM function using LEDC channel
    ledcAttachPin(ledPin, ledChannel); // attach the LEDC channel to the GPIO to be controlled
    ledcWrite(ledChannel, 1); // turn it off if inverted
    //ledupdate function needs to happen earlier.
}

void uartConfig() {

      Serial.begin(57600);  // Setup Serial Monitor

    //characters stored as integers because they really are integers!!!!!!!!!!!! rah!!!!
    const uart_port_t uart_num = UART_NUM_2;

    uart_config_t commanderlink = {
.baud_rate = 57600,
.data_bits = UART_DATA_8_BITS,
.parity = UART_PARITY_DISABLE,
.stop_bits = UART_STOP_BITS_1,
.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
.source_clk = UART_SCLK_APB,
};

ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2048, 0, 0, NULL, 0)); // set rx buffer size to 2048 and no tx buffer since we're not using it. also no queue since we'll be using our own FreeRTOS queue
vTaskDelay(100 / portTICK_PERIOD_MS); // small delay to ensure driver is installed before configuring parameters
ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &commanderlink));
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, auxTransmitter, auxReceiver, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); //last two are for handshake pins. didnt know these exist but might be useful since the run is internal and thus i can do whatever i want
// ESP_ERROR_CHECK(uart_isr_free(UART_NUM_2));
// ESP_ERROR_CHECK(uart_isr_register(UART_NUM_2,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));
// ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_2));
Serial2.begin(57600);



outCommanderQueue = xQueueCreate(50, sizeof(char*));   


xTaskCreatePinnedToCore(
    serial2RX,
    "serial2RX",
    1024,
    NULL,
    16, //this should be sort of high. note to self 2 check and adj +stack size with goated freertos debug toolz
    NULL,
    0 
);

xTaskCreatePinnedToCore(
    serial2tx,
    "serial2tx",
    11520,
    NULL,
    20,
    NULL,
    1
);

  vTaskDelay(200/portTICK_PERIOD_MS); // give some time for the serial monitor to connect before we start sending debug info
  Serial.println("-----------------------------");
  Serial.println("SPECTRAL Projector Controller");
  Serial.println("-----------------------------");


}

void lightSetup() {
    if(LedInvert) {
        GPIO.func_out_sel_cfg[ledPin].inv_sel = 1;
        //above function inverts the led pin at a lower(?) level. this should make writing lamp functions easier
    }
    ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);   // configure LED PWM function using LEDC channel
    ledcAttachPin(ledPin, ledChannel); // attach the LEDC channel to
    ledcWrite(ledChannel, 0);
}

