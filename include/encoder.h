
IRAM_ATTR uint32_t shutterBl = 2;
uint32_t *shutterBlPtr = &shutterBl;


IRAM_ATTR int shutterAng = 25;
int *shutterAngPtr = &shutterAng;

#define TIMER_DIVIDER 80
#define tickPeriod 10



bool IRAM_ATTR ledClock(gptimer_handle_t, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gptimer_set_raw_count(ledtick, 0);

    xSemaphoreGiveFromISR(ledCl, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

void IRAM_ATTR readEncoder(void *pvParameters)
{
    vTaskDelay(250 / portTICK_PERIOD_MS); // small delay to ensure everything is set up before we start creating tasks


    gptimer_config_t clocktimer = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1*1000*1000,
        .intr_priority = 0
    };


    gptimer_event_callbacks_t clockCB = {
        .on_alarm = ledClock,
    };

    gptimer_alarm_config_t ledAlarm = {
        .alarm_count = 4800,
        .reload_count = 0,
    };

    ledAlarm.flags.auto_reload_on_alarm = true;

    gptimer_new_timer(&clocktimer, &ledtick);
    gptimer_register_event_callbacks(ledtick, &clockCB, NULL);
    gptimer_enable(ledtick);
    gptimer_set_raw_count(ledtick, 0);

    gptimer_set_alarm_action(ledtick, &ledAlarm);

/* 
so the below function is essentially a frame buffer, because really the shutter is like a screendrawing function. it must have a resolution assigned. The resolution
can be up to 14 bits (16384) and in theory the chosen scaled resolution will contribute to the accuracy of the shutter blade open/closed transitions.
the three blade shutter mode is the hardest to deal with because it doesnt divide evenly into 16384, let alone 100 or 150. 
Now, the led "draws" at a substantially higher rate -- higher than the gpio interrupts could afford with the original settings. 
I believe we can use this for quicker draws. Now, i will have to consider the speed of the 
*/
    static int shutterBlOld;
    static int shutterAngOld;
    static uint32_t shutterVal = 1;



    for (;;)
    {
        

        if (ledSwitch == 1) {
        xSemaphoreTake(ledCl, portMAX_DELAY);

        xQueueReceive(q_shutterBlade, &shutterBl, 1);  // receive shutter blade value from UI task via queue
        xQueueReceive(q_shutterAngle, &shutterAng, 1); // receive shutter angle value from UI task via queue

        if (shutterBlOld == shutterBl && shutterAngOld == shutterAng)
        {
        }
        else
        {
            *shAngFptr = (100.0 - shutterAng) * (1/100.0);

            for (int myBlade = 0; myBlade < *shutterBlPtr; myBlade++)
            {
                int countOffset = myBlade * (1024 / *shutterBlPtr);
                for (int myCount = 0; myCount < 1024 / *shutterBlPtr; myCount++)
                {
                    if (myCount < (1024 / *shutterBlPtr * *shAngFptr))
                    {
                        shutterMap[(myCount + countOffset)] = 0;
                    }
                    else
                    {
                        shutterMap[myCount + countOffset] = 1;
                    }
                }
            }
        }
        shutterBlOld = *shutterBlPtr;
        shutterAngOld = *shutterAngPtr;



        ang = map(readAngle_raw(ANGLE_REG), 0, 16384, 0, 1024);

        if (shutterMap[ang] == 1)
        {
            shutterVal = 1;

        }
        else if (shutterMap[ang] == 0)
        {
            shutterVal = 0;

        }

        xQueueSend(q_shutterMap, &shutterVal, 1); // send updated shutter map to shutter task via queue

        send_LEDC('l');
    } else if (ledSwitch == 0) {
                ang = map(readAngle_raw(ANGLE_REG), 0, 16384, 0, 1024);

    }
    }
}