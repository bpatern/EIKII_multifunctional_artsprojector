

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


void IRAM_ATTR send_LEDC()
{

    xQueueReceive(ledPot, &ledBright, 1); // receive LED brightness value from LED task via queue

    xQueueReceive(q_shutterMap, &openCl, 1); // receive shutter blade value from UI task via queue


        if (openCl == 0 )
        {
            ledcWrite(ledChannel, 4097);
        }
        else if (openCl == 1)
        {
            ledcWrite(ledChannel, ledBright);
        }
    
    timer_pause(TIMER_GROUP_1, TIMER_0);
    timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &ledWrite_period);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);

}