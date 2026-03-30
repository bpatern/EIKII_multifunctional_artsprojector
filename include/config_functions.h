Encoder enc(EncA, EncB, FULLQUAD, 0);
const int FPSnumReadings = 3;      // how many FPS readings to average together
float FPSAvgArray[FPSnumReadings]; // the FPS readings array
int FPSAvgReadIndex = 0;           // the index of the current reading
float FPSAvgTotal = 0;             // total of all readings

// Ramp library setup

// Simple Kalman Filter Library Setup
// see tuning advice here: https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html
// and this playlist: https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
float kalmanMEA = 2;   // "measurement noise estimate" (larger = we assmume there is more noise/jitter in input)
float kalmanQ = 0.010; // "Process Noise" AKA "gain" (smaller = more smoothing but more latency)

static SimpleKalmanFilter motPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
static SimpleKalmanFilter ledPotKalman(kalmanMEA, kalmanMEA, kalmanQ);

#if (enableSlewPots) // if these pots are enabled in the config file,
SimpleKalmanFilter motSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
SimpleKalmanFilter ledSlewPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
#endif // now we check for the next option, which asks about the

#if (enableShutterPots) // are you using shutter blade angle controls?
static SimpleKalmanFilter shutBladesPotKalman(kalmanMEA, kalmanMEA, kalmanQ);
static SimpleKalmanFilter shutAnglePotKalman(kalmanMEA, kalmanMEA, kalmanQ);
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
static AS5X47 as5047(EncCSN);
// will be used to read data from the magnetic encoder
static IRAM_ATTR ReadDataFrame readDataFrame;
int as5047MagOK = 0; // status of magnet near AS5047 sensor
int as5047MagOK_old = 0;

#endif

// prepare to intialize interrupt tasks
TaskHandle_t pcISR;
TaskHandle_t ledCshutter;

// configure timers
elapsedMicros framePeriod, countPeriod;
elapsedMillis timerUI, serialTimerUI, motSingleRunTimer, motModeMidiTimer, midiParseTimer; // MS since last time we checked/updated the user interface

void core0setup(void *parameter) {
as5047Config();
gpioConfig();
vTaskDelete(NULL);
}

void createTasks() {


        q_shutterBlade = xQueueCreate(2, sizeof(int));
        q_shutterAngle = xQueueCreate(2, sizeof(int)); //shutter angle is never a float in the program which is why i changed it
        q_ledBright = xQueueCreate(36, sizeof(int));
        q_motSpeed = xQueueCreate(36, sizeof(int));
        q_shutterMap = xQueueCreate(8, sizeof(int));
        q_spiRead = xQueueCreate(10, sizeof(int)); // queue used as a binary semaphore to manage access to SPI reads from AS5047 sensor. we use a queue instead of a semaphore because we want the option to add more data to this queue in the future if needed without having to change the type of synchronization primitive we're using. currently we just send an int (with value 0) to this queue whenever a task wants to read from the AS5047, and then receive from the queue after the read is done to allow other tasks to read from the AS5047.
        // xSemaphoreGive(shutterMapping); // give the semaphore so that it's available for the first task to take. after that, any task that wants to update the shutter map will take this semaphore before updating and give it back after it's done, which ensures that only one task can update the shutter map at a time and prevents conflicts between tasks. this is important because updating the shutter map involves multiple steps and we want to make sure those steps are not interrupted by another task trying to update the shutter map at the same time, which could lead to inconsistent or corrupted shutter map data being sent to the LED task and used in the LED ISR.
        q_actualShutterMap = xQueueCreate(100, sizeof(char)); // this queue is used to send the actual shutter map (as opposed to the desired shutter map) to the LED task so that it can update the LED brightness accordingly in real time based on how many blades are actually open
        q_intr1_hasRun = xQueueCreate(1, sizeof(int)); // this queue is used to send a value to the shutter task to indicate that the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue
        q_intr2_hasRun = xQueueCreate(1, sizeof(int)); // this queue is used to send a value to the shutter task to indicate that the encoder pin change ISR has run at least once, which we use as a signal to start sending the actual shutter map position to the LED task via its queue so that the LED task can update the LED brightness accordingly in real time based on how many blades are actually open
        q_debugShutterPosition = xQueueCreate(1, sizeof(int)); // this queue is used to send the current shutter position to the debug task so that it can be printed to the serial monitor for debugging purposes
        q_peekLED = xQueueCreate(3, sizeof(int));
    led_iswriting = xSemaphoreCreateMutex(); // create a mutex to manage access to the LED for writing. this is important because we have multiple tasks and ISRs that want to write to the LED, and we need to make sure that only one of them can write to the LED at a time to prevent conflicts and ensure that the LED state is updated correctly based on the most recent data.
    xSemaphoreGive(led_iswriting);           // give the semaphore so that it's available for the first task to take. after that, any task that wants to write to the LED will take this semaphore before writing and give it back after it's done, which ensures that only one task can write to the LED at a time and prevents conflicts between tasks and ISRs when updating the LED state.

    xTaskCreatePinnedToCore(
    serialReadTask,
    "serialReadTask",
    5000,
    NULL,
    5,
    NULL,
    1
  );

  motPot = xQueueCreate(1, sizeof(int*));
    ledPot = xQueueCreate(2, sizeof(int*));
  xTaskCreatePinnedToCore(
    readUI,
    "readUI",
    8000,
    NULL,
    16,
    NULL,
    1
  );

//       xTaskCreatePinnedToCore(
//     as5047MagCheck,
//     "as5047MagCheck",
//     3000,
//     NULL,
//     10,
//     NULL,
//     1
//   );

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
                updateShutterMap,
                "updateShutter",
                8000,
                NULL,
                15,
                NULL,
                1
            );

    



          xTaskCreatePinnedToCore(
    readEncoder,
    "readEncoder",
    16000,
    NULL,
    16,
    NULL,
    0
    );

      xTaskCreatePinnedToCore(
    updateMotor,
    "updateMotor",
    4000,
    NULL,
    14,
    NULL,
    1
  );

  if (debug==1 ) {
    xTaskCreatePinnedToCore(
        debugTask,
        "debugTask",
        5000,
        NULL,
        12,
        NULL,
        1
    );
  }
    


    
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

    controlLock = xSemaphoreCreateMutex();
    xSemaphoreGive(controlLock); // give the semaphore so that it's available for the first task to take. after that, any task that wants to update shared variables will take this semaphore before updating and give it back after it's done, which ensures that only one task can update shared variables at a time and prevents conflicts between tasks.
    if (enableStatusLed)
    {
          pixels.begin();                // INITIALIZE NeoPixel object
            updateStatusLED(0, 30, 0, 0);  // start with LED red while booting
        if (statusLedColor == 1) {
          updateStatusLED(0, 18, 16, 10);  // white LED at idle
        }

    }
    // if (enableSafeSwitch)
    // {
    //     pinMode(safeSwitch, INPUT_PULLUP);
    // }

    // if (enableMotSwitch)
    // {
    //     pinMode(motDirFwdSwitch, INPUT_PULLUP);
    //     pinMode(motDirBckSwitch, INPUT_PULLUP);
    // }
    // if (enableButtons)
    // {
    //     buttonA.begin(buttonApin, INPUT_PULLUP, true);
    //     buttonA.setDebounceTime(20);
    //     // buttonA.setPressedHandler(pressed);
    //     // buttonA.setReleasedHandler(released);

    //     buttonB.begin(buttonBpin, INPUT_PULLUP, true);
    //     buttonB.setDebounceTime(20);
    //     // buttonB.setPressedHandler(pressed);
    //     // buttonB.setReleasedHandler(released);

    //     if (digitalRead(buttonApin) == 0)
    //     {
    //         // Program the ESC settings if user holds down buttonA during startup

    //         ESCprogram();
    //         while (1)
    //             ; // don't continue setup since the ESC needs to be rebooted before we can continue
    //     }
    // }
}

void gpioConfig() {

    q_motRunFwd = xQueueCreate(6, sizeof(int));
    q_motRunBack = xQueueCreate(6, sizeof(int));
    q_safemode = xQueueCreate(6, sizeof(int));
    q_buttonF = xQueueCreate(12, sizeof(int));
    q_buttonR = xQueueCreate(12, sizeof(int));

    static gpio_config_t safemode_conf = {
        .pin_bit_mask = 1ULL << safeSwitch,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    static gpio_config_t fwdsw_conf = {
        .pin_bit_mask = 1ULL << motDirFwdSwitch,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
        static gpio_config_t revsw_conf = {
        .pin_bit_mask = 1ULL << motDirBckSwitch,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
        static gpio_config_t btnF_conf = {
        .pin_bit_mask = 1ULL << buttonApin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
            static gpio_config_t btnR_conf = {
        .pin_bit_mask = 1ULL << buttonBpin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

        gpio_config(&safemode_conf);
        gpio_config(&fwdsw_conf);
        gpio_config(&revsw_conf);
        gpio_config(&btnF_conf);
    gpio_config(&btnR_conf);

    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirFwdSwitch, gpioGet, (void*) (gpio_num_t)motDirFwdSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirBckSwitch, gpioGet, (void*)(gpio_num_t)motDirBckSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)safeSwitch, gpioGet, (void*)(gpio_num_t)safeSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonApin, gpioGet, (void*)(gpio_num_t)buttonApin)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonBpin, gpioGet, (void*)(gpio_num_t)buttonBpin)); // attach the interrupt service routine to the GPIO pin for index





}

void as5047Config()
{
    //mutex means mutual exclusion. we need this to make sure certain things arent being accessed at the same exact time
    //example: imagine a burger being eaten in a doorless house
    //what if i was trying to eat a burger and some freak who was able to get in snatched the burger half eaten from my hands

    // now i'd only have half a burger of satisfaction and so would the burger thief.
    //now imagine if the completion of the burger meal was the key to world peace. now everythings fucked

    //mutex is kinda like locking a door to make sure just one person can eat the burger at a time
    //you can decide when to unlock that door

    

        // attachInterrupt(EncA, pinChangeISR, CHANGE);
    // attachInterrupt(EncB, pinChangeISR, CHANGE);






    static gpio_config_t index_conf = {
        .pin_bit_mask = 1ULL << EncI,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&index_conf);

    static gpio_config_t encA_conf = {
        .pin_bit_mask = 1ULL << EncA, // we can set both encoder pins to trigger the same interrupt since we read both pin states in the ISR and can determine the direction and amount of rotation based on those states
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // we want to trigger on both edges to ensure we catch every single change in the encoder state, which allows us to have more accurate readings of the encoder position and speed, which is crucial for the performance of the system. if we only triggered on one edge, we would only be able to detect half of the changes in the encoder state, which would lead to less accurate readings and potentially cause issues with the motor control and shutter timing.
    };
    gpio_config(&encA_conf);

    static gpio_config_t encB_conf = {
        .pin_bit_mask = 1ULL << EncB, // we can set both encoder pins to trigger the same interrupt since we read both pin states in the ISR and can determine the direction and amount of rotation based on those states
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // we want to trigger on both edges to ensure we catch every single change in the encoder state, which allows us to have more accurate readings of the encoder position and speed, which is crucial for the performance of the system. if we only triggered on one edge, we would only be able to detect half of the changes in the encoder state, which would lead to less accurate readings and potentially cause issues with the motor control and shutter timing.
    };
    gpio_config(&encB_conf);

gpio_install_isr_service(0); // install GPIO ISR service with default configuration
// ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)EncA, send_LEDC, NULL)); // attach the interrupt service routine to the GPIO pin for channel A
// ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)EncB, send_LEDC, NULL)); // attach the interrupt service routine to the GPIO pin for channel B
// ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)EncI, indexISR, NULL)); // attach the interrupt service routine to the GPIO pin for index
    
          encoderMutex = xSemaphoreCreateMutex();
          xSemaphoreGive(encoderMutex); // give the semaphore so that it's available for the first interrupt to take. after that, the interrupt will give it back after it's done updating shared variables, and then other interrupts can take it again when they need to update shared variables. this ensures that only one interrupt can update shared variables at a time, which prevents
            spiRead = xSemaphoreCreateMutex();
            xSemaphoreGive(spiRead); // give the semaphore so that it's available for the first SPI read. after that, any task that wants to read from the AS5047 will take this semaphore before reading and give it back after it's done, which ensures that only one task can read from the AS5047 at a time and prevents conflicts on the SPI bus.
//       if (enc.init()) {
//     Serial.println("Encoder Initialization OK");
//   } else {
//     Serial.println("Encoder Initialization Failed");
//     while(1);
//   }


    abOld = encCount = encCountOld = 0;
    // Set rotation direction (see AS5047 datasheet page 17)
    Settings1 settings1;
    settings1.values.dir = encoderDir;
        settings1.values.daecdis = 1;

    as5047.writeSettings1(settings1);
    // Set ABI output resolution (see AS5047 datasheet page 19)
    // (pulses per rev: 5 = 50 pulses, 6 = 25 pulses, 7 = 8 pulses)
    Settings2 settings2;
    settings2.values.abires = 2; // 25 pulses per rev = 100 transitions. This is what we want.
    as5047.writeSettings2(settings2);
    // Disable ABI output when magnet error (low or high) exists (see AS5047 datasheet page 24)
    Zposl zposl;
    Zposm zposm;
    zposl.values.compLerrorEn = 1;
    zposl.values.compHerrorEn = 1;
    as5047.writeZeroPosition(zposm, zposl);

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
    ledcWrite(motPWMChannel, (1 << motPWMRes) * 1250 / motPWMPeriod);  // duty = # of values at current res * US / pulse period

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

      Serial.begin(921600);  // Setup Serial Monitor

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
    3750,
    NULL,
    14, //this should be sort of high. note to self 2 check and adj +stack size with goated freertos debug toolz
    NULL,
    1
);

xTaskCreatePinnedToCore(
    serial2tx,
    "serial2tx",
    3750,
    NULL,
    15,
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
        //above function inverts the led pin at a lower(?) level. this should make writing lamp functions easier
    }
    ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);   // configure LED PWM function using LEDC channel
    ledcAttachPin(ledPin, ledChannel); // attach the LEDC channel to

            GPIO.func_out_sel_cfg[ledPin].inv_sel = 1;

    ledcWrite(ledChannel, 0);
}

