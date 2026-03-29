since the arduino ide has more forgiveness for the placement of functions throughout the program, this program will need restructuring to comply with what a normal C++ compiler wants. I'd prefer to leave things mostly as is as I branch out the bulk of this functionality into the toolkit library, since some behaviors are tighter in my beaulieu camera motor code.

i've moved a lot around. biggest change i had to make was making certain variables global like those at the beginning of CalcFPS. if i didnt make this change i would get crashing if the projector was running for a Long Time. debug stated it was fetching a value before the value was even written. this is probably a result of additional tasks needing to happen during the main loop. splitting operations out to multicore would maybe help, or using the ESP32-s3 with additional SRAM

// ------------------------------------------
// SPECTRAL Wandering Sounds + Images Seminar
// ------------------------------------------
// Projector control software for ESP32 micro-controller
// On some 38pin ESP32 boards you need to press GPIO 0 [right] button each time you upload code. If so, solder 10uF cap between EN and GND to enable auto reset.
// Controlling Hobbywing "Quickrun Fusion SE" motor/ESC combo using ESP32 LEDC peripheral (using 16bit timer for extra resolution)
// Note: Hobbywing motor/esc needs to be programmed via "programming card" to disable braking, otherwise it will try to hold position when speed = 0
// (If you hack the ESC wiring according to our docs, you can program it with the ESCprogram() function instead of a programming card)
// "Digital shutter" is accomplished via AS5047 magnetic rotary encoder, setup via SPI registers and monitored via ABI quadrature pulses
// LED is dimmed via CC LED driver board with PWM input, driven by ESP32 LEDC (which causes slight PWM blinking artifacts at low brightness levels)
// Future option exists for current-controlled dimming (set LedDimMode=0): Perhaps a log taper digipot controlled via SPI? Probably can't dim below 20% though.


TODO LIST:
move "loop" code into freertos tasks
how do I make the creation of such into a function?
id assume with FREERTOS that one wouldn't need to put anything into loop. 

additionally, it would probably be a better use of time to get other board types working
teensy, newer esp32s3, rp2350B, stm32 variants
 as well as bringing the benefits of the existing functionality to other use cases, 
 remote control, feedback, single framing, user interface should be portable to most projector projects


// Include the libraries
// NOTE: In 2023 this code was developed using the ESP32 Arduino core v2.0.9.
// When ESP32 Arduino core v3.x is released there will be breaking changes because LEDC setup will be different!

as of 2026 i am still using this core running in platformIO. 
it seems like there are some "things" with using the esp32 in arduino mode

id like to transition to using new ESP-IDF libraries possibly maybe
i hesitate to do the above because its easier to get going with the code in the Arduino IDE, which is going to be most familiar to ppl.
and the closer functionalities get to using ESP specific stuff the more reliant the code becomes on those architectures/languages

but technically arduino functions live on top of ESP-IDF in arduino mode. currently switching to lower level stuffz should help with the cycle speed/operation. You mostly wont see the diff as just a projector but trying to get it to respond properly while avoiding control interrupts during movement is hard. tasking functionality of RTOS should make things smoother and more reliable in installation/studio use cases
