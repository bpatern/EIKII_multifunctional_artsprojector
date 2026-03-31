
static IRAM_ATTR int shutterVal = 1;
static IRAM_ATTR int shutterBl = 2;

static IRAM_ATTR int shutterAng;
static IRAM_ATTR int shutterBlOld;
static IRAM_ATTR int shutterAngOld;
static IRAM_ATTR float shAngF;
static IRAM_ATTR int ang;
static IRAM_ATTR int wrCt;
static IRAM_ATTR int remap;



static void IRAM_ATTR shutterRead(spi_transaction_t *val) 
{
        static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(encoderRead, &xHigherPriorityTaskWoken);
        if(&xHigherPriorityTaskWoken)
            {
                portYIELD_FROM_ISR();
            }


}

    static spi_transaction_t sb;

#define TIMER_DIVIDER 80
#define tickPeriod 10

    static gptimer_handle_t ledtick = NULL;


bool IRAM_ATTR ledClock(gptimer_handle_t, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(ledCl, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

void IRAM_ATTR processVal(void *pvParameters)
{
    for(;;)
    {
        // xSemaphoreTake(encoderRead, portMAX_DELAY);
        // rx_buffer;

        // Serial.println(sb);

    }
}

void IRAM_ATTR readEncoder(void *pvParameters)
{

    ledCl = xSemaphoreCreateBinary();
    const bool a_r = true;

        vTaskDelay(250 / portTICK_PERIOD_MS); // small delay to ensure everything is set up before we start creating tasks


   static gptimer_config_t clocktimer = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0
    };
        gptimer_new_timer(&clocktimer, &ledtick);


    static gptimer_event_callbacks_t clockCB = {
        .on_alarm = ledClock,
    };
        gptimer_register_event_callbacks(ledtick, &clockCB, NULL);


    static gptimer_alarm_config_t ledAlarm = {
        .alarm_count = 5,
        .reload_count = 0,
    };
    ledAlarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(ledtick, &ledAlarm);

    gptimer_enable(ledtick);
    gptimer_start(ledtick);



    // timer_config_t clock = {
    //     .alarm_en = TIMER_ALARM_EN,
    //     .counter_en = TIMER_PAUSE,
    //     .counter_dir = TIMER_COUNT_UP,
    //     .auto_reload = (timer_autoreload_t)a_r,
    //     .divider = TIMER_DIVIDER}; // default clock source is APB

    // timer_init(TIMER_GROUP_1, TIMER_1, &clock); // freertos runs using group 0?
    // timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);
    // timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, tickPeriod);
    // timer_enable_intr(TIMER_GROUP_1, TIMER_1);
    // timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, ledClock, NULL, 0);
    // timer_start(TIMER_GROUP_1, TIMER_1);

    // // clock_info_t *timer_info = calloc(1, sizeof(clock_info_t));

    // timer_config_t runPeriod = {
    //     .alarm_en = TIMER_ALARM_EN,
    //     .counter_en = TIMER_PAUSE,
    //     .counter_dir = TIMER_COUNT_UP,
    //     .auto_reload = (timer_autoreload_t)a_r,
    //     .divider = TIMER_DIVIDER}; // default clock source is APB
    // timer_init(TIMER_GROUP_1, TIMER_0, &runPeriod);

    // static gptimer_config_t watchdawgTimer = {
    //     .clk_src = GPTIMER_CLK_SRC_APB,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000000,
    //     .intr_priority = 0
    // };
    //  gptimer_new_timer(&watchdawgTimer, &watchdawg);
    //         gptimer_enable(watchdawg);


    // timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);



    for (;;)
    {

        xSemaphoreTake(ledCl, portMAX_DELAY);
        // Serial.println("X");
        // gptimer_start(watchdawg);
        // xQueueReceive(q_shutterBlade, &shutterBl, 1);  // receive shutter blade value from UI task via queue
        // xQueueReceive(q_shutterAngle, &shutterAng, 1); // receive shutter angle value from UI task via queue

        if (shutterBlOld == shutterBl && shutterAngOld == shutterAng)
        {
        }
        else
        {
            // remap = 1;
            shAngF = (float)(100.00 - (float)shutterAng) / 100.00;

            for (int myBlade = 0; myBlade < shutterBl; myBlade++)
            {
                int countOffset = myBlade * (150 / shutterBl);
                for (int myCount = 0; myCount < 150 / shutterBl; myCount++)
                {
                    if (myCount < (150.00 / shutterBl * shAngF))
                    {
                        shutterMap[myCount + countOffset] = 1;
                    }
                    else
                    {
                        shutterMap[myCount + countOffset] = 0;
                    }
                }
            }
        }

        // ang = map(as5047.readAngle(), 0, 360, 0, 150);
            ang = 0;
        if (shutterMap[ang] == 1)
        {
            shutterVal = 1;
        }
        else if (shutterMap[ang] == 0)
        {
            shutterVal = 0;
        }

        xQueueSend(q_shutterMap, &shutterVal, 1); // send updated shutter map to shutter task via queue
        send_LEDC();

        shutterBlOld = shutterBl;
        shutterAngOld = shutterAng;
    }
}

void fixCount()
{
    // encCount = map(as5047.readAngle(), 0, 360, 0, 100);
    Serial.println("   (Updated count via SPI)");
}

// fill shutterMap array with boolean values to control LED state at each position of shutter rotation
void updateShutterMap(void *parameter)

// i am 70% sure the shutter can be represented as a math function rather than writing to an array
// currently the array writes over the shutter array constantly, and doesnt always finish before a new shutter led position is needed
{ // move to core 0 with related interrupts

    static int shutBladesVal = 0;
    static int shutAngleVal = 0;
    static bool *shutterMap[countsPerFrame]; // array to hold the on/off state of the shutter for each count position

    for (;;)
    {
        // if (xSemaphoreTake(shutterMapping, 25) == pdTRUE)
        // {
        // { // take the semaphore to ensure exclusive access to shared variables
        // Serial.println("Updating Shutter Map");
#if (enableSlewPots)
        motSlewVal = motSlewPotKalman.updateEstimate(analogRead(motSlewPotPin));
        // delay(2);
        ledSlewVal = ledSlewPotKalman.updateEstimate(analogRead(ledSlewPotPin));
        // delay(2);
#endif

#if (enableShutterPots && enableShutter)
        if (musicMode == 0)
        {
            shutBladesPotVal = shutBladesPotKalman.updateEstimate(analogRead(shutBladesPotPin));
            if (shutBladesPotVal < 800)
            {
                shutBladesVal = 1;
            }
            else if (shutBladesPotVal < 2500)
            {
                shutBladesVal = 2;
            }
            else
            {
                shutBladesVal = 3;
            }
            shutAnglePotVal = shutAnglePotKalman.updateEstimate(analogRead(shutAnglePotPin));
            shutAngleVal = map(shutAnglePotVal, 0, 4090, 1, 100); // map ADC input to range of shutter angle

            // delay(2);
        }
        else if (musicMode == 1)
        {
            shutBladesPotVal = map(CC2ProjBlades, 0, 100, 0, 4095);
            // inject values during Music Mode
        }
        // delay(2);
        // using custom scaling for shutBladesPotVal to make control feel right

        //   shutAngleVal = constrain(shutAngleVal, 1, 100);

        xQueueSend(q_shutterBlade, &shutBladesVal, 20); // send shutter blade pot value to shutter task via queue
        xQueueSend(q_shutterAngle, &shutAngleVal, 20);  // send shutter angle pot value to shutter task via queue
                                                        // clip values
#endif
                                                       // // xQueueReceive(q_shutterBlade, &sB, portMAX_DELAY);
                                                       // // xQueueReceive(q_shutterAngle, &sA, portMAX_DELAY);
                                                       // Serial.println("Update ShutterMap");
                                                       // //  shutterBlades: number of virtual shutter blades (must be > 0)
                                                       // //  shutterAngle: ratio between on/off for each shutter blade segment (0.5 = 180d)
                                                       // if (shutBladesVal < 1)
                                                       // {
                                                       //     shutBladesVal = 1;                       // it would break if set to 0
                                                       //     shutAngleVal = constrain(shutAngleVal, 0.0, 1.0); // make sure it's 0-1
                                                       // }
                                                       // for (int myBlade = 0; myBlade < shutBladesVal; myBlade++)
                                                       // {
                                                       //     int countOffset = myBlade * (countsPerFrame / shutBladesVal);
                                                       //     for (int myCount = 0; myCount < countsPerFrame / shutBladesVal; myCount++)
                                                       //     {
                                                       // if (myCount < countsPerFrame / shutBladesVal * (1.0 - shutAngleVal))
        //         {
        //             shutterMap[myCount + countOffset] = (bool*)false;
        //             // xQueueSend(q_shutterMap, &"0", sizeof(shutterMap)); // send updated shutter map to shutter task via queue
        //         }
        //         else
        //         {
        //             shutterMap[myCount + countOffset] = (bool*)true;
        //             // xQueueSend(q_shutterMap, &"1", sizeof(shutterMap)); // send updated shutter map to shutter task via queue
        //         }
        //     }
        // }
        // xQueueSend(q_shutterMap, &shutterMap[countsPerFrame], sizeof(shutterMap)); // send updated shutter map to shutter task via queue

        // xSemaphoreGive(shutterMapping); // give the semaphore back after updating shared variables
        // }
    }
}




// send info to the LEDC peripheral to update LED PWM (abstracted here because it's called from loop or ISR)

// check for encoder magnet proximity
// void as5047MagCheck(void *pvParameters)
// {
//     for (;;)
//     {

//         // read magnet AGC data from sensor registers
//         readDataFrame = as5047.readRegister(DIAGAGC_REG);
//         Diaagc diaagc;
//         diaagc.raw = readDataFrame.values.data;
//         // Serial.println(diaagc.values.magl);

//         // check result for magnet errors and update global var
//         if (diaagc.values.magh || diaagc.values.magl)
//         {
//             as5047MagOK = 0;
//         }
//         else
//         {
//             as5047MagOK = 1;
//         }

//         // take action if global var has changed
//         if (as5047MagOK_old != as5047MagOK)
//         {
//             if (as5047MagOK)
//             {
//                 Serial.println("Magnet OK");
//             }
//             else
//             {
//                 Serial.println("Magnet ERROR");
//             }
//             as5047MagOK_old = as5047MagOK;
//         }
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }