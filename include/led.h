

static int actualShutterMapPosition;

static int shutterSize = 150;
static int shutterSegments = 3;
static int segmentSize = 40;

static int shutterDiv;
static int shutterOn;

static int ledWriteCount;
static IRAM_ATTR int ledBright;
static IRAM_ATTR int *ledBrightptr = &ledBright;
static IRAM_ATTR uint32_t openClOld;

static IRAM_ATTR int ledBrightOld;

// static IRAM_ATTR int openCl;
static IRAM_ATTR uint64_t ledWrite_period;

static gptimer_handle_t watchdawg = NULL;



void IRAM_ATTR send_LEDC(char fromwhere, uint8_t map = 0, uint16_t brightness = 0)
{
    static uint32_t openCl;
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
            // ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        }
        else if (openCl == 1)
        {
                    // ledc_timer_rst(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1);
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledBright);
            // ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 16000, 0);        
        
        }
        }
    

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
    }

    openClOld = openCl;

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    gptimer_set_raw_count(ledtick, 4800);

            // gptimer_get_raw_count(ledtick, &ledtimer);




}