/*
continuous movement commands are passed to the motor commander
single frame movements also pass through the commander which handles
temporarily suspending and then enabling the single frame task.


*/

static volatile bool motorIsRunning = false;
static volatile bool *motorStatus = &motorIsRunning;
static volatile char whichbutton = 'a';

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO 0                   // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static int motPotReading;
static char act;
static bool SF;
    static char intDirection;
            static int whatio;
            static char rAction;

static void motConstHandler(void *pvParams)
{
    static int brake = 0;
static int ioPool;
    for (;;)
    {
        // ESP_LOGI("motCont", "intDir %c", intDirection);

        xQueueReceive(motPot, &motPotReading, 1);
        xQueueReceive(runMsg, &rAction, 5);


        int angle = map(motPotReading, 0, 4095, 0, 30);

        intDirection = rAction;

        // if(ioPool == motDirFwdSwitch)
        // {
        //     if (rAction == 1)
        //     {
        //         intDirection = 'f';
        //     } else if (rAction == 0)
        //     {
        //         intDirection = 'b';
        //     }

        // } else if (ioPool == motDirBckSwitch)
        // {
        //     if (rAction == 0)
        //     {
        //         intDirection = 'r';
        //     } else if (rAction == 1)
        //     {
        //         intDirection = 'b';
        //     }

        // } else if (ioPool == 99)
        // {
        //     if (rAction == 1)
        //     {
        //         intDirection = 'x';
        //     }
        // }


        if (intDirection == 'f')
        {
                        vTaskResume(ledDraw);

            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-angle));
        }
        else if (intDirection == 'r')
        {
                                    vTaskResume(ledDraw);

            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(14));
        } else if (intDirection == 'b')
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-50));
            vTaskDelay(30);

            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

            vTaskSuspend(ledDraw);
            send_LEDC('x');
            rAction = 9;
            intDirection = 'x';
        }else if (intDirection == 'l')
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(50));
            vTaskDelay(30);

            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

            vTaskSuspend(ledDraw);
            send_LEDC('x');
            rAction = 9;
            intDirection = 'x';
        }
        else if (intDirection == 'x')
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
            vTaskSuspend(ledDraw);

            ESP_LOGI("motCont", "brake");
        }
    }
}
static uint8_t buttonF = 0;
static uint8_t buttonR = 0;
static uint8_t buttonFold = 0;
static uint8_t buttonRold = 0;
static uint8_t singleFrameSM;
static uint8_t R_singleFrameSM;

static char butn;

static void singleFrameHandler(void *pvParams)
{

    for (;;)
    {
        if(motorIsRunning == false)
        {
        // ang = map(readAngle_raw(0x3FFE), 0, 16384, 0, 1024);
        }

        xQueueReceive(q_whichbutton, &butn, 2);
        if (butn == 'a')
        {
            // if (motorIsRunning == true) {
            //             vTaskSuspend(ledDraw);
            // }

                xQueueReceive(q_buttonF, &buttonF, 2);

                //     if (buttonF != buttonFold)
                // {
                if (buttonF == 1)
                {
                    if (xSemaphoreTake(singleFraming, 2) == pdTRUE && singleFrameSM == 0)
                    {
                        vTaskSuspend(motContinuousHandle);

                        vTaskResume(ledDraw);
                        singleFrameSM = 2;
                    }
                    else 
                    {

                    }
                    // buttonF = 1;
                }
                else if (buttonF == 0)
                {
                }
                buttonF = 9;
                // buttonFold = buttonF;
            
        }
        else if (butn == 'b')
        {

                xQueueReceive(q_buttonR, &buttonR, 2);


                if (buttonR == 1)
                {
                    if (xSemaphoreTake(singleFraming, 2) == pdTRUE && R_singleFrameSM == 0)
                    {
                                                vTaskSuspend(motContinuousHandle);

                        vTaskResume(ledDraw);
                        R_singleFrameSM = 2;
                    } else
                    {

                    }
                }
                else if (buttonR == 0)
                {
                }
                buttonR = 9;
            
        }

        if (singleFrameSM == 2)
        {
            send_LEDC('c', 1, 2048);
            singleFrameSM = 3;
        }
        else if (singleFrameSM == 3)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(15));
                        vTaskDelay(10);
            if (*angle > 600 && *angle < 800)
            {
                singleFrameSM = 4;
            }
        }
        else if (singleFrameSM == 4)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-50));
            vTaskDelay(30);
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
            vTaskDelay(20);
            vTaskSuspend(ledDraw);
            singleFrameSM = 5;
        }
        else if (singleFrameSM == 5)
        {
            if (buttonApinState == 1)
            {
                send_LEDC('c', 1, 1700);
            }
            else if (buttonApinState == 0)
            {
                send_LEDC('x');
                xSemaphoreGive(singleFraming);
                        vTaskResume(motContinuousHandle);
                singleFrameSM = 0;
            }
        }

        if (R_singleFrameSM == 2)
        {
            send_LEDC('c', 1, 2048);
            R_singleFrameSM = 3;
        }
        else if (R_singleFrameSM == 3)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-15));
            vTaskDelay(10);
            if (*angle > 600 && *angle < 700)
            {
                R_singleFrameSM = 4;
            }
        }
        else if (R_singleFrameSM == 4)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(50));
            vTaskDelay(30);
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
            vTaskDelay(20);
            vTaskSuspend(ledDraw);
            R_singleFrameSM = 5;
        }
        else if (R_singleFrameSM == 5)
        {
            if (buttonBpinState == 1)
            {
                send_LEDC('c', 1, 1700);
            }
            else if (buttonBpinState == 0)
            {
                send_LEDC('x');
                xSemaphoreGive(singleFraming);

                        vTaskResume(motContinuousHandle);
                R_singleFrameSM = 0;
            }
        }
    }
}