/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 Sample Program 1
* 1/19/2021 
* 
* By Kevin Kelly
* 
***************************************************/

/* Homework 2 Sample Program 1
 * Slow LED blinking program example in C for 
 * Raspberry Pi 4 computer with 
 * red    LED on GPIO12 (with 430 ohm resistor in series)
 * green  LED on GPIO13 (with 470 ohm resistor in series)
 * blue   LED on GPIO23 (with 470 ohm resistor in series)
 * yellow LED on GPIO24 (with 470 ohm resistor in series)
 * 
 * Turn on each LED in sequence, switching every 0.25 seconds
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

    /* set the pin function to OUTPUT for GPIO12 - red    LED light */
    /* set the pin function to OUTPUT for GPIO12 - green  LED light */
    /* set the pin function to OUTPUT for GPIO12 - blue   LED light */
    /* set the pin function to OUTPUT for GPIO12 - yellow LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO12
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;  //GPIO13
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;  //GPIO23
    io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;  //GPIO24
    
    printf( "hit 'ctl c' to quit\n");

    // initial state
    GPIO_CLR( &(io->gpio), 12);
    GPIO_CLR( &(io->gpio), 13);
    GPIO_CLR( &(io->gpio), 23);
    GPIO_CLR( &(io->gpio), 24);


    while (1)
    {
      GPIO_SET( &(io->gpio), 12);
      GPIO_CLR( &(io->gpio), 13);
      GPIO_CLR( &(io->gpio), 23);
      GPIO_CLR( &(io->gpio), 24);

      usleep(250*1000);

      GPIO_CLR( &(io->gpio), 12);
      GPIO_SET( &(io->gpio), 13);
      GPIO_CLR( &(io->gpio), 23);
      GPIO_CLR( &(io->gpio), 24);

      usleep(250*1000);
      
      GPIO_CLR( &(io->gpio), 12);
      GPIO_CLR( &(io->gpio), 13);
      GPIO_SET( &(io->gpio), 23);
      GPIO_CLR( &(io->gpio), 24);

      usleep(250*1000);
      
      GPIO_CLR( &(io->gpio), 12);
      GPIO_CLR( &(io->gpio), 13);
      GPIO_CLR( &(io->gpio), 23);
      GPIO_SET( &(io->gpio), 24);

      usleep(250*1000);

    }

  }
  else
  {
    ; /* warning message already issued */
  }
  
  // "get those lights off!"
  GPIO_CLR( &(io->gpio), 12);
  GPIO_CLR( &(io->gpio), 13);
  GPIO_CLR( &(io->gpio), 23);
  GPIO_CLR( &(io->gpio), 24);
  
  /* clean the GPIO pins */
  io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
  io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

  return 0;
}
