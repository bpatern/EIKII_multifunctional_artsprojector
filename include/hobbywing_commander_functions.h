#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             0        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
static int motPotReading;
static void motorCommander(char action, int isStill) {



    if (action == 'i')
    {
        mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
        motor_isRunning = xSemaphoreCreateMutex();
        runMsg = xQueueCreate(2, sizeof(char));
    } else if (action == 'f') 
    {
        if (isStill == 0)
        {
        } else if (isStill == 1)
        {

        }


    } else if (action == 'r')
    {
        if (isStill == 0)
        {
        } else if (isStill == 1)
        {

        }
    } else if (action == 'x') {
                mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0));
    }
    vTaskResume(motHandle);
    xQueueSend(runMsg, &action, portMAX_DELAY);
}
 
static void motHandler(void *pvParams) {

    static char rAction;
    for(;;) {

            xQueueReceive(motPot, &motPotReading, 1);
            xQueueReceive(runMsg, &rAction, 5);
            int angle = map(motPotReading, 0, 4095, 0, 20);
                //other tasks can be put onto the queue! 
                if(rAction == 'f')
                {
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle));
                } else if (rAction == 'r')
                {
                    mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(-angle));
                } else if (rAction == 'x')
                {
                    vTaskSuspend(NULL);
                }



    }

}