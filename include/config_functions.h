// Encoder enc(EncA, EncB, FULLQUAD, 0);
#define FPSnumReadings 3           // how many FPS readings to average together
float FPSAvgArray[FPSnumReadings]; // the FPS readings array
int FPSAvgReadIndex = 0;           // the index of the current reading
float FPSAvgTotal = 0;             // total of all readings

// Simple Kalman Filter Library Setup
// see tuning advice here: https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html
// and this playlist: https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
float kalmanMEA = 2;   // "measurement noise estimate" (larger = we assmume there is more noise/jitter in input)
float kalmanQ = 0.010; // "Process Noise" AKA "gain" (smaller = more smoothing but more latency)

// prepare to intialize interrupt tasks
TaskHandle_t pcISR;
TaskHandle_t ledCshutter;

// configure timers
elapsedMicros framePeriod, countPeriod;
elapsedMillis timerUI, serialTimerUI, motSingleRunTimer, motModeMidiTimer, midiParseTimer; // MS since last time we checked/updated the user interface

void createObj()
{
    q_shutterBlade = xQueueCreate(2, sizeof(int));
    q_shutterAngle = xQueueCreate(2, sizeof(int)); // shutter angle is never a float in the program which is why i changed it
    q_motSpeed = xQueueCreate(36, sizeof(int));
    q_shutterMap = xQueueCreate(8, sizeof(int));
    q_spiRead = xQueueCreate(10, sizeof(int));             // queue used as a binary semaphore to manage access to SPI reads from AS5047 sensor. we use a queue instead of a semaphore because we want the option to add more data to this queue in the future if needed without having to change the type of synchronization primitive we're using. currently we just send an int (with value 0) to this queue whenever a task wants to read from the AS5047, and then receive from the queue after the read is done to allow other tasks to read from the AS5047.
    q_actualShutterMap = xQueueCreate(100, sizeof(char));  // this queue is used to send the actual shutter map (as opposed to the desired shutter map) to the LED task so that it can update the LED brightness accordingly in real time based on how many blades are actually open
    q_intr1_hasRun = xQueueCreate(1, sizeof(int));         // this queue is used to send a value to the shutter task to indicate that the LED ISR has run at least once, which we use as a signal to start sending the shutter map to the LED task via its queue
    q_intr2_hasRun = xQueueCreate(1, sizeof(int));         // this queue is used to send a value to the shutter task to indicate that the encoder pin change ISR has run at least once, which we use as a signal to start sending the actual shutter map position to the LED task via its queue so that the LED task can update the LED brightness accordingly in real time based on how many blades are actually open
    q_debugShutterPosition = xQueueCreate(1, sizeof(int)); // this queue is used to send the current shutter position to the debug task so that it can be printed to the serial monitor for debugging purposes
    q_peekLED = xQueueCreate(3, sizeof(int));
    led_iswriting = xSemaphoreCreateMutex(); // create a mutex to manage access to the LED for writing. this is important because we have multiple tasks and ISRs that want to write to the LED, and we need to make sure that only one of them can write to the LED at a time to prevent conflicts and ensure that the LED state is updated correctly based on the most recent data.
    xSemaphoreGive(led_iswriting);           // give the semaphore so that it's available for the first task to take. after that, any task that wants to write to the LED will take this semaphore before writing and give it back after it's done, which ensures that only one task can write to the LED at a time and prevents conflicts between tasks and ISRs when updating the LED state.
    motPot = xQueueCreate(1, sizeof(int *));
    ledPot = xQueueCreate(8, sizeof(int *));
    controlLock = xSemaphoreCreateMutex();
    xSemaphoreGive(controlLock);              // give the semaphore so that it's available for the first task to take. after that, any task that wants to update shared variables will take this semaphore before updating and give it back after it's done, which ensures that only one task can update shared variables at a time and prevents conflicts between tasks.
    shutterMapping = xSemaphoreCreateMutex(); // create the shutter mapping semaphore after the delay to ensure that it is created after the tasks that use it are created, which prevents potential issues with tasks trying to take a semaphore that hasn't been created yet. this is important because the shutter mapping semaphore is used in
    q_ledBright = xQueueCreate(2, sizeof(int));
    ledCl = xSemaphoreCreateBinary();
    physinput = xSemaphoreCreateBinary();
    ioQ = xQueueCreate(1, sizeof(uint32_t));
    q_motRunFwd = xQueueCreate(1, sizeof(uint8_t));
    q_motRunBack = xQueueCreate(1, sizeof(uint8_t));
    q_safemode = xQueueCreate(2, sizeof(uint8_t));
    q_buttonF = xQueueCreate(2, sizeof(uint8_t));
    q_buttonR = xQueueCreate(2, sizeof(uint8_t));
    q_actor = xQueueCreate(2, sizeof(char));
    outCommanderQueue = xQueueCreate(40, sizeof(char));
    tx = xSemaphoreCreateMutex();
    runMsg = xQueueCreate(1, sizeof(char));
    customBrightness = xQueueCreate(3, sizeof(uint16_t));
    q_whichbutton = xQueueCreate(2, sizeof(char));
    q_singleFrame = xQueueCreate(8, sizeof(char));
    singleFraming = xSemaphoreCreateMutex();
    xSemaphoreGive(singleFraming);
}

void createTasks()
{

    xTaskCreatePinnedToCore(
        serialReadTask,
        "serialReadTask",
        8000,
        NULL,
        12,
        &externalControlParse,
        1);
    vTaskSuspend(externalControlParse);

    xTaskCreatePinnedToCore(
        calcFPS,
        "calcFPS",
        1500,
        NULL,
        12,
        &FPSactor,
        1);
    vTaskSuspend(FPSactor);

    xTaskCreatePinnedToCore(
        updateMotor,
        "updateMotor",
        4000,
        NULL,
        14,
        &motorSlewRead,
        1);
    vTaskSuspend(motorSlewRead);
}

void gpioConfig()
{

    vTaskDelay(500 / portTICK_PERIOD_MS); // small delay to ensure everything is set up before we start creating tasks

    gpio_config_t switch_conf = {
        .pin_bit_mask = (1ULL << buttonBpin) | (1ULL << buttonApin) | (1ULL << motDirFwdSwitch) | (1ULL << motDirBckSwitch) | (1ULL << safeSwitch),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_install_isr_service(0);

    gpio_config(&switch_conf);

    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirFwdSwitch, switchGet, (void *)(gpio_num_t)motDirFwdSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)motDirBckSwitch, switchGet, (void *)(gpio_num_t)motDirBckSwitch)); // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)safeSwitch, switchGet, (void *)(gpio_num_t)safeSwitch));           // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonApin, switchGet, (void *)(gpio_num_t)buttonApin));           // attach the interrupt service routine to the GPIO pin for index
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)buttonBpin, switchGet, (void *)(gpio_num_t)buttonBpin));           // attach the interrupt service routine to the GPIO pin for index

    xTaskCreatePinnedToCore(
        parseIO,
        "Parse IO",
        8000,
        NULL,
        configMAX_PRIORITIES - 7,
        &ioTASKHANDLE,
        1);
    vTaskSuspend(ioTASKHANDLE);

    xTaskCreatePinnedToCore(
        readUI,
        "readUI",
        4000,
        NULL,
        16,
        &readControls,
        0);
    vTaskSuspend(readControls);
}

void as5047Config()
{
    encoderRead = xSemaphoreCreateBinary();

    configEncoderSPI(1);
    // argument passes the core number the readings happen on//
}

adc_oneshot_unit_handle_t adc_handle;

void mathConfig()
{

    // Initialize ADC Oneshot Mode Driver on the ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, motPotPin, &config);
    adc_oneshot_config_channel(adc_handle, ledPotPin, &config);
    adc_oneshot_config_channel(adc_handle, shutBladesPotPin, &config);
    adc_oneshot_config_channel(adc_handle, shutAnglePotPin, &config);

    xTaskCreatePinnedToCore(
        readShutterControls,
        "read shutter controls",
        2000,
        NULL,
        15,
        &shutterPotTranslate,
        1);

    vTaskSuspend(shutterPotTranslate);

    // motPotKalman -> Init();
    // ledPotKalman -> Init();
    // shutBladesPotKalman -> Init();
    // shutAnglePotKalman -> Init();

    // motSlewPotKalman -> Init();
    // motSlewVal -> UpdateRefMeasurement(NULL, motSlewPotPin, kalmanMEA, kalmanQ);
    // ledSlewPotKalman -> Init();
    // ledSlewVal -> UpdateRefMeasurement(NULL, ledSlewPotPin, kalmanMEA, kalmanQ);

    for (int thisReading = 0; thisReading < FPSnumReadings; thisReading++)
    {
        FPSAvgArray[thisReading] = 0;
    }
}

void motorConfig()
{
    //   Motor PWM setup

    mcpwm_timer_config_t motPWM = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD};

    mcpwm_new_timer(&motPWM, &motPWMTimer);

    mcpwm_operator_config_t motPWMCFG = {
        .group_id = 0,
    };

    mcpwm_new_operator(&motPWMCFG, &motOper);
    mcpwm_operator_connect_timer(motOper, motPWMTimer);

    comparator_config.flags.update_cmp_on_tez = true;

    mcpwm_new_comparator(motOper, &comparator_config, &comparator);

    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

    mcpwm_generator_config_t motGen = {
        .gen_gpio_num = escPin,
    };
    mcpwm_new_generator(motOper, &motGen, &motGenerator);

    mcpwm_generator_set_action_on_timer_event(motGenerator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(motGenerator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(motPWMTimer);
    mcpwm_timer_start_stop(motPWMTimer, MCPWM_TIMER_START_NO_STOP);

    vTaskDelay(pdMS_TO_TICKS(500));

    xTaskCreatePinnedToCore(
        motConstHandler,
        "motor continuous movement driver",
        8000,
        NULL,
        16,
        &motContinuousHandle,
        1);

    vTaskSuspend(motContinuousHandle);

    xTaskCreatePinnedToCore(
        singleFrameHandler,
        "Single Frame Handler",
        10000,
        NULL,
        16,
        &singleFrame,
        1);

    vTaskSuspend(singleFrame);

    char strMsg = 'x';
    xQueueSend(runMsg, &strMsg, portMAX_DELAY);
    
ledSwitch = 0;
    send_LEDC('x');

}

void mechanicalShutterConfig()
{
    // Motor PWM setup for mechanical shutter control (same as motorConfig but with different channel and pin)
    ledc_timer_config_t led_Timer = {

    };

    ledc_channel_config_t led_Channel = {

    };

    ledc_timer_config(&led_Timer);
    ledc_channel_config(&led_Channel);
}

void uartConfig()
{

    uart_config_t commanderlink = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 2048, 512, 0, NULL, 0)); // set rx buffer size to 2048 and no tx buffer since we're not using it. also no queue since we'll be using our own FreeRTOS queue
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &commanderlink));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, auxTransmitter, auxReceiver, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // last two are for handshake pins. didnt know these exist but might be useful since the run is internal and thus i can do whatever i want

    // xTaskCreatePinnedToCore(
    //     serial2RX,
    //     "serial2RX",
    //     3750,
    //     NULL,
    //     14, // this should be sort of high. note to self 2 check and adj +stack size with goated freertos debug toolz
    //     &internalSerialRX,
    //     1);
    // vTaskSuspend(internalSerialRX);

    // xTaskCreatePinnedToCore(
    //     serial2tx,
    //     "serial2tx",
    //     3750,
    //     NULL,
    //     15,
    //     &internalSerialTX,
    //     1);
    // vTaskSuspend(internalSerialTX);
}

void lightSetup()
{

    ledc_timer_config_t led_Timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 8000,
        .clk_cfg = LEDC_USE_APB_CLK,

    };

    ledc_channel_config_t led_Channel = {
        .gpio_num = ledPin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,

    };

    ledc_timer_config(&led_Timer);
    ledc_bind_channel_timer(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_TIMER_1);
    if (LedInvert)
    {
        led_Channel.flags.output_invert = true;
    }

    ledc_channel_config(&led_Channel);

    xTaskCreatePinnedToCore(
        readEncoder,
        "readEncoder",
        8000,
        NULL,
        configMAX_PRIORITIES - 2,
        &ledDraw,
        0);
    // vTaskSuspend(ledDraw);
}

void ledCoolingSetup() {
    ledc_timer_config_t fan = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_3,
        .freq_hz = 50000,
        .clk_cfg = LEDC_USE_APB_CLK,
    };

        ledc_timer_config(&fan);


    ledc_channel_config_t fan_channel1 = {
        .gpio_num = exhaustFan,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_3,
        .duty = 0,
    };



    ledc_channel_config_t fan_channel2 = {
        .gpio_num = twelveVoltFans,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_TIMER_3,
        .duty = 0,
    };
        ledc_bind_channel_timer(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, LEDC_TIMER_3);
                ledc_bind_channel_timer(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5, LEDC_TIMER_3);


            ledc_channel_config(&fan_channel1);
        ledc_channel_config(&fan_channel2);


    xTaskCreatePinnedToCore(
        cooling,
        "fan control",
        2000,
        NULL,
        13,
        &fancontrol,
        1
    );
}
