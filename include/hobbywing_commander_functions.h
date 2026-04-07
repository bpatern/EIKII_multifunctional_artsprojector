/*
continuous movement commands are passed to the motor commander
single frame movements also pass through the commander which handles
temporarily suspending and then enabling the single frame task.
*/
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO 0                   // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

int motPotReading;
char act;
bool SF;
char intDirection;
int Mangle;


inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}



void motConstHandler(void *pvParams)
{
    static int brake = 0;
    static int ioPool;
    static char rAction;
    static int isPaused = 0;
    static int dir;
    static char rActionWhenEntered;
    static char rActionNew;

    for (;;)
    {

        if (xQueueReceive(runMsg, &rAction, 1) == pdTRUE)
        {

            intDirection = rAction;
        }

        Mangle = map(motPotVal, 0, 4095, 0, 30);

        isPaused = motSwitch;

        if (motSwitch == 0)
        {

            if (intDirection == 'f')
            {
                ledSwitch = 1;
                dir = 1;
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-Mangle));
            }
            else if (intDirection == 'r')
            {
                ledSwitch = 1;

                dir = -1;
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(Mangle));
            }
            else if (intDirection == 'b')
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-50));
                vTaskDelay(30);
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                ledSwitch = 0;

                send_LEDC('x');
                rAction = 9;
                dir = 0;
                intDirection = 'x';
            }
            else if (intDirection == 'l')
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(50));
                vTaskDelay(30);
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                ledSwitch = 0;

                send_LEDC('x');
                rAction = 9;
                dir = 0;
                intDirection = 'x';
            }
            else if (intDirection == 'x')
            {

                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

                intDirection = 'p';
            }
            else if (intDirection == 'p')
            {
            }
        }
        else if (motSwitch == 1)
        {
        }

        vTaskDelay(10);
    }
}

void singleFrameHandler(void *pvParams)
{
    static uint8_t buttonF = 0;
    static uint8_t buttonR = 0;
    static uint8_t singleFrameSM;
    static uint8_t R_singleFrameSM;
    static uint8_t extCommand = '&';
    static uint8_t isExternalCommand = 0;

    for (;;)
    {

        if (xQueueReceive(q_singleFrame, &extCommand, 1) == pdTRUE)
        {
            isExternalCommand = 1;
            if (extCommand == 'f')
            {
                if (R_singleFrameSM == 5)
                    {
                        R_singleFrameSM = 0;
                    }
                    ESP_LOGI("motCont", "A");
                    ledSwitch = 0;

                    // xTaskNotifyGive(motContinuousHandle);
                    motSwitch = 1;
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                    singleFrameSM = 2;
            } else if (extCommand == 'r')
            {
                if (singleFrameSM == 5)
                    {
                        singleFrameSM = 0;
                    }
                    ESP_LOGI("motCont", "A");
                    ledSwitch = 0;

                    motSwitch = 1;
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                    R_singleFrameSM = 2;
            }


        }

        if (isExternalCommand == 0)
        {
        if (buttonApinState == 0)
        {
            if (singleFrameSM > 0 && singleFrameSM != 5)
            {
                singleFrameSM = 5;
            }
        }
        if (buttonBpinState == 0)
        {
            if (R_singleFrameSM > 0 && R_singleFrameSM != 5)
            {
                R_singleFrameSM = 5;
            }
        }
    }
        if (singleFrameSM == 0)
        {

            if (buttonApinState == 1)
            {
                if (xSemaphoreTake(singleFraming, 2) == pdTRUE && singleFrameSM == 0 && (R_singleFrameSM == 0 || R_singleFrameSM == 5))
                {
                    if (R_singleFrameSM == 5)
                    {
                        R_singleFrameSM = 0;
                    }
                    ESP_LOGI("motCont", "A");
                    ledSwitch = 0;

                    // xTaskNotifyGive(motContinuousHandle);
                    motSwitch = 1;
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                    singleFrameSM = 2;
                }
            }
            buttonF = 9;
            vTaskDelay(20);
        }

        if (singleFrameSM > 1)
        {
        }

        if (singleFrameSM == 2)
        {
            ESP_LOGI("motCont", "B");
            send_LEDC('c', 1, 2048);

            if (ang > 600 && ang < 800)
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-15));
                vTaskDelay(100);
            }

            singleFrameSM = 3;
        }
        else if (singleFrameSM == 3)
        {
            ESP_LOGI("motCont", "C");

            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-12));
            vTaskDelay(10);

            if (ang > 600 && ang < 800)
            {
                singleFrameSM = 4;
            }
        }
        else if (singleFrameSM == 4)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(80));
            vTaskDelay(30);
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
            vTaskDelay(20);
            singleFrameSM = 5;
        }
        else if (singleFrameSM == 5)
        {
            if (buttonApinState == 1)
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

                send_LEDC('c', 1, 1700);
            }
            else if (buttonApinState == 0)
            {
                xSemaphoreGive(singleFraming);
                if(extCommand != '&'){extCommand = '&'; isExternalCommand = 0;}


                ledSwitch = 0;

                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

                send_LEDC('x');

                motSwitch = 0;
                singleFrameSM = 0;
                R_singleFrameSM = 0;
            }
        }

        if (R_singleFrameSM == 0)
        {

            if (buttonBpinState == 1)
            {
                if (xSemaphoreTake(singleFraming, 2) == pdTRUE && R_singleFrameSM == 0 && (singleFrameSM == 0 || singleFrameSM == 5))
                {
                    if (singleFrameSM == 5)
                    {
                        singleFrameSM = 0;
                    }
                    ESP_LOGI("motCont", "A");
                    ledSwitch = 0;

                    motSwitch = 1;
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
                    R_singleFrameSM = 2;
                }
            }
            buttonR = 9;
            vTaskDelay(20);
        }

        if (R_singleFrameSM == 2)
        {
            send_LEDC('c', 1, 2048);

            if (ang > 600 && ang < 800)
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(15));
                vTaskDelay(100);
            }
            R_singleFrameSM = 3;
        }
        else if (R_singleFrameSM == 3)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(13));
            // vTaskDelay(10);

            if (ang > 600 && ang < 800)
            {
                R_singleFrameSM = 4;
            }
        }
        else if (R_singleFrameSM == 4)
        {
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-70));
            vTaskDelay(30);
            mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
            vTaskDelay(20);
            R_singleFrameSM = 5;
        }
        else if (R_singleFrameSM == 5)
        {
            if (buttonBpinState == 1)
            {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

                send_LEDC('c', 1, 1700);
            }
            else if (buttonBpinState == 0)
            {
                xSemaphoreGive(singleFraming);
                if(extCommand != '&'){extCommand = '&'; isExternalCommand = 0;}

                ledSwitch = 0;

                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));

                send_LEDC('x');

                motSwitch = 0;
                R_singleFrameSM = 0;
                singleFrameSM = 0;
            }
        }
    }
}