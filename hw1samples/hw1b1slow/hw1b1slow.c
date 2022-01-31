/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 Sample Program 1
* Revision V2.1
* On 2/4/2018
* On 1/14/2022
* By Kyusun Choi
* 
***************************************************/

/* Homework 2 Sample Program 1
 * Slow LED blinking program example in C for 
 * Raspberry Pi 4 computer with 
 * red   LED on GPIO12 (with 500 ohm resistor in series)
 * 
 * LED turn-on and LED turn-off in 2 second cycle, repeat
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"


int main( void )
{
  volatile struct io_peripherals *io;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO18 - red   LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO12
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;  //GPIO23
    printf( "hit 'ctl c' to quit\n");

    while (1)
    {
      GPIO_SET( &(io->gpio), 12);

      sleep(1);

      GPIO_CLR( &(io->gpio), 12);

      sleep(1);

    }

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
