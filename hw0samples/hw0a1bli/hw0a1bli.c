/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a1bli - Red LED at GPIO 12 Blinking, starting example
* Revision V1.1   On 1/18/2022
* By Kyusun Choi
* 
* Red LED on GPIO 12 (with 500 ohm resistor in series)
* 
* LED turn-on and LED turn-off, in 2 second cycle, blinking at 0.5Hz rate.
* Program ending - type 'Ct'l c' to quit this program.
* 
***************************************************/

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
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
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    
    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
  
    printf( "hit 'ctl c' to quit\n");

    while (1)                         // repeat forever

    {
      GPIO_SET( &(io->gpio), 12);     // Turn on red LED
      sleep(1);                       // sleep 1 sec.
      GPIO_CLR( &(io->gpio), 12);     // Turn off red LED
      sleep(1);                       // sleep 1 sec.
    }

  }

  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
