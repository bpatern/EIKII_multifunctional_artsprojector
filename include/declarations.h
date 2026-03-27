void externalcontrol();
void midiControlTakeover();
void recvWithStartEndMarkers();
void midiParser();
void uartConfig();
void serialReadTask(void *pvparemeters);

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
void updateShutterMap(byte shutterBlades, float shutterAngle);
void pressed(Button2& btn);
void released(Button2& btn);
void change(Button2& btn);
void updateStatusLED(int x, int R, int G, int B);
// prototypes for ISR functions that will be defined in later code
// (prototype must be declared _before_ we attach interrupt because ESP32 requires "IRAM_ATTR" flag which breaks typical Arduino behavior)
void IRAM_ATTR pinChangeISR();
void IRAM_ATTR send_LEDC();
