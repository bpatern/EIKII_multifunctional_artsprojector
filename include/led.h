void IRAM_ATTR send_LEDC(char fromwhere, uint8_t map = 0, uint16_t brightness = 0)
{
    static uint32_t openCl;
    static IRAM_ATTR uint32_t openClOld;
    static IRAM_ATTR int ledBrightOld;
    static IRAM_ATTR int ledBright;



    xQueueReceive(ledPot, &ledBright, 1); // receive LED brightness value from LED task via queue
    xQueueReceive(q_shutterMap, &openCl, 1); // receive shutter blade value from UI task via queue
    
    // Serial.println(ledBright);
    if (fromwhere == 'l')
    {
        if (openCl != openClOld)
        {
        if (openCl == 0)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        }
        else if (openCl == 1)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledBright);
        }
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    gptimer_set_raw_count(ledtick, 4800);
    

    }
    else
    {
        if (fromwhere == 'x')
        {
            openCl = 0;
        }
        else if (fromwhere == 'b')
        {
            xQueueReceive(customBrightness, &ledBright, 1);
        }
        else if (fromwhere == 'c')
        {
            openCl = map;           // at the point of this call, is the shutter open or closed? (does not factor in encoder position unless thats part of ur function!!)
            ledBright = brightness; // exposes low level brightness control. i would recommend using the custombrightness queue instead
        }

        if (openCl == 0)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);

        }
        else if (openCl == 1)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledBright);


        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

    openClOld = openCl;



}

void cooling(void *pvparemeters) {

    for(;;)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 572);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 150);


        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);


        vTaskDelay(200);
    }

}