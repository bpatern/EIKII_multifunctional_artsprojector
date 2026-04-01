
static IRAM_ATTR int shutterVal = 1;
static IRAM_ATTR int shutterBl = 2;

static IRAM_ATTR int shutterAng;
static IRAM_ATTR int shutterBlOld;
static IRAM_ATTR int shutterAngOld;
static IRAM_ATTR float shAngF;
    static int *ang1 = &ang;

static IRAM_ATTR int wrCt;
static IRAM_ATTR int remap;

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

void IRAM_ATTR readEncoder(void *pvParameters)
{

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
        .alarm_count = 1000,
        .reload_count = 0,
    };
    ledAlarm.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(ledtick, &ledAlarm);

    gptimer_enable(ledtick);


/* 
so the below function is essentially a frame buffer, because really the shutter is like a screendrawing function. it must have a resolution assigned. The resolution
can be up to 14 bits (16384) and in theory the chosen scaled resolution will contribute to the accuracy of the shutter blade open/closed transitions.
the three blade shutter mode is the hardest to deal with because it doesnt divide evenly into 16384, let alone 100 or 150. 
Now, the led "draws" at a substantially higher rate -- higher than the gpio interrupts could afford with the original settings. 
I believe we can use this for quicker draws. Now, i will have to consider the speed of the 
*/

    // int *shutterMap = (int *)malloc(sizeof(int) * countsPerFrame);
    static bool shutterMap[1024];
    // memset(shutterMap, false, countsPerFrame*sizeof(bool));

        gptimer_start(ledtick);


    for (;;)
    {

        xSemaphoreTake(ledCl, portMAX_DELAY);
        xQueueReceive(q_shutterBlade, &shutterBl, 1);  // receive shutter blade value from UI task via queue
        xQueueReceive(q_shutterAngle, &shutterAng, 1); // receive shutter angle value from UI task via queue

        if (shutterBlOld == shutterBl && shutterAngOld == shutterAng)
        {
        }
        else
        {
            shAngF = (100.0f - (float)shutterAng) * (1/100.0f);
            if (shutterMap == NULL) {
                Serial.println("memory error");
            }


            for (int myBlade = 0; myBlade < shutterBl; myBlade++)
            {
                int countOffset = myBlade * (1024 / shutterBl);
                for (int myCount = 0; myCount < 1024 / shutterBl; myCount++)
                {
                    if (myCount < (1024 / shutterBl * shAngF))
                    {
                        shutterMap[myCount + countOffset] = 0;
                    }
                    else
                    {
                        shutterMap[myCount + countOffset] = 1;
                    }
                }
            }
        }
        
        ang = map(readAngleBuffer(true), 0, 16384, 0, 1024);

        if (shutterMap[ang] == true)
        {
            shutterVal = 1;

        }
        else if (shutterMap[ang] == false)
        {
            shutterVal = 0;

        }

        xQueueSend(q_shutterMap, &shutterVal, 1); // send updated shutter map to shutter task via queue
        send_LEDC();

        shutterBlOld = shutterBl;
        shutterAngOld = shutterAng;

    }
}