/* 
   main.pp: Generic main() for caling Arduino-style setup(), loop()

   Also provides delay(), micros() routines

   Copyright (c) 2018 Simon D. Levy

   This file is part of MPU9250.

   Library may be used freely and without limit with attribution.
*/

#include <unistd.h>
#include <time.h>

extern void setup(), loop();

void delay(unsigned int msec)
{
    usleep(msec*1000);
}

unsigned int micros(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return 1000000*ts.tv_sec + ts.tv_nsec/1000;
}

int main(int argc, char ** argv)
{
    setup();

    while (true) {
        loop();
    }
}
