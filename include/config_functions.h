// Encoder enc(EncA, EncB, FULLQUAD, 0);
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


#endif

// prepare to intialize interrupt tasks
TaskHandle_t pcISR;
TaskHandle_t ledCshutter;

// configure timers
elapsedMicros framePeriod, countPeriod;
elapsedMillis timerUI, serialTimerUI, motSingleRunTimer, motModeMidiTimer, midiParseTimer; // MS since last time we checked/updated the user interface

void createObj() {
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
      motPot = xQueueCreate(1, sizeof(int*));
    ledPot = xQueueCreate(2, sizeof(int*));
     controlLock = xSemaphoreCreateMutex();
    xSemaphoreGive(controlLock); // give the semaphore so that it's available for the first task to take. after that, any task that wants to update shared variables will take this semaphore before updating and give it back after it's done, which ensures that only one task can update shared variables at a time and prevents conflicts between tasks.
      shutterMapping = xSemaphoreCreateMutex(); // create the shutter mapping semaphore after the delay to ensure that it is created after the tasks that use it are created, which prevents potential issues with tasks trying to take a semaphore that hasn't been created yet. this is important because the shutter mapping semaphore is used in

}
void core0setup(void *parameter) {

    // Serial.println("HELLO");
      lightSetup();
as5047Config();
vTaskDelete(NULL);
}

void core1setup(void *parameter) {
    createObj();
      uartConfig();
      gpioConfig();





    // Serial.println("HELLO");
vTaskDelete(NULL);
}

void createTasks() {




    xTaskCreatePinnedToCore(
    serialReadTask,
    "serialReadTask",
    5000,
    NULL,
    5,
    NULL,
    1
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
    10000,
    NULL,
    configMAX_PRIORITIES - 2,
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

   
    // if (enableStatusLed)
    // {
    //       pixels.begin();                // INITIALIZE NeoPixel object
    //         updateStatusLED(0, 30, 0, 0);  // start with LED red while booting
    //     if (statusLedColor == 1) {
    //       updateStatusLED(0, 18, 16, 10);  // white LED at idle
    //     }

    // }
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

static TaskHandle_t process = NULL;

void gpioConfig() {



    physinput = xSemaphoreCreateBinary();
    ioQ = xQueueCreate(12, sizeof(uint32_t));



    xTaskCreatePinnedToCore(
        parseIO,
        "Parse IO",
        2048,
        NULL,
        configMAX_PRIORITIES - 1,
        &ioTASKHANDLE,
        1
    );

        vTaskDelay(500 / portTICK_PERIOD_MS); // small delay to ensure everything is set up before we start creating tasks




    q_motRunFwd = xQueueCreate(2, sizeof(int));
    q_motRunBack = xQueueCreate(2, sizeof(int));
    q_safemode = xQueueCreate(2, sizeof(int));
    q_buttonF = xQueueCreate(6, sizeof(int));
    q_buttonR = xQueueCreate(6, sizeof(int));



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

    gpio_install_isr_service(0);


    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirFwdSwitch, gpioGet, (void*) (gpio_num_t)motDirFwdSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirBckSwitch, gpioGet, (void*)(gpio_num_t)motDirBckSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)safeSwitch, gpioGet, (void*)(gpio_num_t)safeSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonApin, gpioGet, (void*)(gpio_num_t)buttonApin)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonBpin, gpioGet, (void*)(gpio_num_t)buttonBpin)); // attach the interrupt service routine to the GPIO pin for index


  xTaskCreatePinnedToCore(
    readUI,
    "readUI",
    8000,
    NULL,
    16,
    NULL,
    0
  );


}

static spi_device_handle_t shutter = 0;



void as5047Config()
{
    // static gpio_config_t index_conf = {
    //     .pin_bit_mask = 1ULL << EncI,
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };
    // gpio_config(&index_conf);

    // static gpio_config_t encA_conf = {
    //     .pin_bit_mask = 1ULL << EncA, // we can set both encoder pins to trigger the same interrupt since we read both pin states in the ISR and can determine the direction and amount of rotation based on those states
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE // we want to trigger on both edges to ensure we catch every single change in the encoder state, which allows us to have more accurate readings of the encoder position and speed, which is crucial for the performance of the system. if we only triggered on one edge, we would only be able to detect half of the changes in the encoder state, which would lead to less accurate readings and potentially cause issues with the motor control and shutter timing.
    // };
    // gpio_config(&encA_conf);

    // static gpio_config_t encB_conf = {
    //     .pin_bit_mask = 1ULL << EncB, // we can set both encoder pins to trigger the same interrupt since we read both pin states in the ISR and can determine the direction and amount of rotation based on those states
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE // we want to trigger on both edges to ensure we catch every single change in the encoder state, which allows us to have more accurate readings of the encoder position and speed, which is crucial for the performance of the system. if we only triggered on one edge, we would only be able to detect half of the changes in the encoder state, which would lead to less accurate readings and potentially cause issues with the motor control and shutter timing.
    // };
    // gpio_config(&encB_conf);

    
    // encoderMutex = xSemaphoreCreateMutex();
    // xSemaphoreGive(encoderMutex); // give the semaphore so that it's available for the first interrupt to take. after that, the interrupt will give it back after it's done updating shared variables, and then other interrupts can take it again when they need to update shared variables. this ensures that only one interrupt can update shared variables at a time, which prevents
    // spiRead = xSemaphoreCreateMutex();
    // xSemaphoreGive(spiRead); // give the semaphore so that it's available for the first SPI read. after that, any task that wants to read from the AS5047 will take this semaphore before reading and give it back after it's done, which ensures that only one task can read from the AS5047 at a time and prevents conflicts on the SPI bus.
    // encoderRead = xSemaphoreCreateBinary();


    // spi_bus_config_t encoderBus = {
    //     .mosi_io_num = (gpio_num_t)EncMOSI,
    //     .miso_io_num = (gpio_num_t)EncMISO,
    //     .sclk_io_num = (gpio_num_t)EncCLK,
    //     .isr_cpu_id = ESP_INTR_CPU_AFFINITY_0,
    // };

    // spi_bus_initialize(SPI3_HOST, &encoderBus, SPI_DMA_CH_AUTO);

    // spi_device_interface_config_t encoderInterface = {
    //     .command_bits = 16,
    //     .address_bits = 14,
    //     // .dummy_bits = 1,
    //     .mode = 1,
    //     .clock_source = SPI_CLK_SRC_APB,
    //     .duty_cycle_pos = 0, //half
    //     .clock_speed_hz = SPI_MASTER_FREQ_10M,
    //     .spics_io_num = (gpio_num_t)EncCSN,
    //     .queue_size = 2,
    // };

    // spi_bus_add_device(SPI3_HOST, &encoderInterface, &shutter);
    // // typedef union 
    // // {
    
    //     //using settings registers from AS5X47 library, so do not untag the lib!!
    //     Settings1 setting1;
    //     setting1.values.dir = encoderDir;
    // setting1.values.daecdis = 1;
    // spi_transaction_t settings1 = {
    //     .cmd = SETTINGS1_REG,
    //     .addr = (uint16_t)setting1.raw,
    //     .length = 16,
    // };

    //     Settings2 setting2;

    // spi_transaction_t settings2 = {
    //     .cmd = SETTINGS2_REG,
    //     .addr = (uint16_t)setting2.raw,
    //     .length = 16,
    // };

    // Mag cordicVal;

    // spi_transaction_t getCordic = {
    //     .cmd = ANGLE_REG,
    //     .addr = cordicVal.raw,
    //     .length = 16,
    //     .rx_buffer = &shutterBuffer

    // };



    // abOld = encCount = encCountOld = 0;
    // // Set rotation direction (see AS5047 datasheet page 17)
    // Settings1 settings1;


    // as5047.writeSettings1(settings1);
    // // Set ABI output resolution (see AS5047 datasheet page 19)
    // // (pulses per rev: 5 = 50 pulses, 6 = 25 pulses, 7 = 8 pulses)
    // Settings2 settings2;
    // settings2.values.abires = 2; // 25 pulses per rev = 100 transitions. This is what we want.
    // as5047.writeSettings2(settings2);
    // // Disable ABI output when magnet error (low or high) exists (see AS5047 datasheet page 24)
    // Zposl zposl;
    // Zposm zposm;
    // zposl.values.compLerrorEn = 1;
    // zposl.values.compHerrorEn = 1;
    // as5047.writeZeroPosition(zposm, zposl);

}

void mathConfig() {
      for (int thisReading = 0; thisReading < FPSnumReadings; thisReading++) {
    FPSAvgArray[thisReading] = 0;
  }

}

void motorConfig() {
    //   Motor PWM setup

      static mcpwm_timer_config_t motPWM = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = motPWMRes,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000
      };

      static mcpwm_timer_handle_t motPWMTimer = NULL;
      mcpwm_new_timer(&motPWM, &motPWMTimer);

      static mcpwm_operator_config_t motPWMCFG = {
        .group_id = 0,
      };

      static mcpwm_oper_handle_t motOper = NULL;
      mcpwm_new_operator(&motPWMCFG, &motOper);
      mcpwm_operator_connect_timer(motOper, motPWMTimer);

      static mcpwm_generator_config_t motGen = {
        .gen_gpio_num = escPin,
      };
      static mcpwm_gen_handle_t motGenerator = NULL;
      mcpwm_new_generator(motOper, &motGen, &motGenerator);


//   ledcSetup(motPWMChannel, motPWMFreq, motPWMRes);  // configure motor PWM function using LEDC channel
//   ledcAttachPin(escPin, motPWMChannel);             // attach the LEDC channel to the GPIO to be controlled

//   it's important to send the ESC a "0" speed signal (1500us) whenever the motor is stopped. Otherwise the ESC goes into "failsafe" mode thinking that our RC car has lost contact with the TX!


//   ledcWrite(motPWMChannel, (1 << motPWMRes) * 1500 / motPWMPeriod);  // duty = # of values at current res * US / pulse period
//   Serial.print("Sending neutral signal to ESC...");
//   delay(4000);
//   Serial.println("Done");
//     ledcWrite(motPWMChannel, (1 << motPWMRes) * 1375 / motPWMPeriod);  // duty = # of values at current res * US / pulse period

}


void mechanicalShutterConfig() {
      // Motor PWM setup for mechanical shutter control (same as motorConfig but with different channel and pin)
        ledc_timer_config_t led_Timer = {

    };

    ledc_channel_config_t led_Channel = {

    };

    ledc_timer_config(&led_Timer);
    ledc_channel_config(&led_Channel);
    //ledupdate function needs to happen earlier.
}

void uartConfig() {



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
ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &commanderlink));
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, auxTransmitter, auxReceiver, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); //last two are for handshake pins. didnt know these exist but might be useful since the run is internal and thus i can do whatever i want
// ESP_ERROR_CHECK(uart_isr_free(UART_NUM_2));
// ESP_ERROR_CHECK(uart_isr_register(UART_NUM_2,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));
// ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_2));
Serial2.flush();
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
      Serial.begin(57600);  // Setup Serial Monitor
      Serial.flush();
  Serial.println("-----------------------------");
  Serial.println("SPECTRAL Projector Controller");
  Serial.println("-----------------------------");


}

void lightSetup() {
    if(LedInvert) {
        //above function inverts the led pin at a lower(?) level. this should make writing lamp functions easier
    }

    static ledc_timer_config_t led_Timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 19000
    };

    static ledc_channel_config_t led_Channel = {
        .gpio_num = ledPin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,

    };

    ledc_timer_config(&led_Timer);

    led_Channel.flags.output_invert = true; 

    ledc_channel_config(&led_Channel);


    // ledcSetup(ledChannel, ledBrightFreq, ledBrightRes);   // configure LED PWM function using LEDC channel
    // ledcAttachPin(ledPin, ledChannel); // attach the LEDC channel to


    // ledcWrite(ledChannel, 0);
}

