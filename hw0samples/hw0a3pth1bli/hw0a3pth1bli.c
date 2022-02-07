/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a3pth1bli 
* 
*   - Red   LED at GPIO12 Blink 5Hz
*   - Green LED at GPIO13 Blink 5Hz
*   - ending by 'ctl c' key hits
* 
* Revision V1.1   On 1/22/2022
* By Kyusun Choi
* 
* Red   LED on GPIO12 (with 500 ohm resistor in series)
* Green LED on GPIO13 (with 500 ohm resistor in series)
* 
***************************************************/


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "wait_period.h"


int main( void )
{
  volatile struct io_peripherals *io;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red   LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
    
    printf( "\n Press 'ctl c' to quit.\n");
    

    while (1)
    {
      GPIO_SET( &(io->gpio), 12);  // on
      GPIO_CLR( &(io->gpio), 13);  // off

      usleep(100*1000);    /* numbers in micro seconds (100msec.)  */

      GPIO_CLR( &(io->gpio), 12);  // off
      GPIO_SET( &(io->gpio), 13);  // on

      usleep(100*1000);    /* numbers in micro seconds (100msec.)  */
    }
    
    // Loop forever, Following will not be run

    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
