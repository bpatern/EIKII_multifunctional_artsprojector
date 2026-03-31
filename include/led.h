

static char shutMapIRQ[150]; // local variable to hold the shutter map value received from the queue, which will be used to update the LED state in real time in the ISR based on how many blades are actually open at any given moment
static int actualShutterMapPosition;

static int shutterSize = 150;
static int shutterSegments = 3;
static int segmentSize = 40;

static int shutterDiv;
static int shutterOn;

static int ledWriteCount;
static IRAM_ATTR int ledBright;
static IRAM_ATTR int openCl;
static IRAM_ATTR uint64_t ledWrite_period;

    static gptimer_handle_t watchdawg = NULL;

void IRAM_ATTR send_LEDC()
{

    xQueueReceive(ledPot, &ledBright, 1); // receive LED brightness value from LED task via queue

    xQueueReceive(q_shutterMap, &openCl, 1); // receive shutter blade value from UI task via queue


        if (openCl == 0 )
        {
            // ledcWrite(ledChannel, 4097);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        }
        else if (openCl == 1)
        {
            // ledcWrite(ledChannel, ledBright);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ledBright);

        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        

        // gptimer_stop(watchdawg);
        // gptimer_get_raw_count(watchdawg, &ledWrite_period);
        // gptimer_set_raw_count(watchdawg, 0);
        // Serial.println(ledWrite_period);

}