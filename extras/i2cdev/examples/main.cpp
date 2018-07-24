/* 
   main.pp: Generic main() for caling Arduino-style setup(), loop()

   Also provides delay() routine

   Copyright (c) 2018 Simon D. Levy

   This file is part of MPU9250.

   Library may be used freely and without limit with attribution.
*/

#include <unistd.h>

extern void setup(), loop();

void delay(unsigned int msec)
{
    usleep(msec*1000);
}

int main(int argc, char ** argv)
{
    setup();

    while (true) {
        loop();
    }
}
