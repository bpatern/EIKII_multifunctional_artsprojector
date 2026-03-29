void externalcontrol();
void midiControlTakeover();
void recvWithStartEndMarkers();
void midiParser();
void uartConfig();
void serialReadTask(void *pvparemeters);
void ESC_send_ACK();
void ESC_ser_write(unsigned char x);

void createTasks();
void serial2RX(void * parameter);
void serial2tx(void * parameter);
void sendConfirmation(char type);
void updateMotor(void *pvParameters);
void updateLed(void *pvParameters);
void as5047MagCheck(void *pvParameters);
void calcFPS(void *pvParameters);
void readUI(void *pvParameters);
void readEncoder(void *pvParameters);
void ESCprogram();
void fixCount();
void shutterQueue(byte shutterBlades, float shutterAngle);
void updateShutterMap(void *parameter);
void pressed(Button2& btn);
void released(Button2& btn);
void change(Button2& btn);
void updateStatusLED(int x, int R, int G, int B);
// prototypes for ISR functions that will be defined in later code
// (prototype must be declared _before_ we attach interrupt because ESP32 requires "IRAM_ATTR" flag which breaks typical Arduino behavior)
void IRAM_ATTR pinChangeISR(void *arg);
void IRAM_ATTR indexISR(void *arg);
void IRAM_ATTR send_LEDC();
void debugTask(void *pvParameters);

double mapf(double x, double in_min, double in_max, double out_min, double out_max);
float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve);

