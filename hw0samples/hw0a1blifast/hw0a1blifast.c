/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a1blifast - Red   LED at GPIO12 Blinking
*                             - Green LED at GPIO13 Blinking
* Revision V1.1   On 1/18/2022
* By Kyusun Choi
* 
* Red   LED on GPIO12 (with 500 ohm resistor in series)
* Green LED on GPIO13 (with 500 ohm resistor in series)
* Switch on GPIO26 (with 10K ohm pull-up resistor to 3.3V)
* 
* LED turn-on and LEDs turn-off, in 0.2 second cycle, blinking at 5Hz rate.
* Program ending - press (GPIO 26 pin low with 0V) to quit this program, or
*                - type 'Ct'l c' to quit this program.
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

    /* set the pin function to OUTPUT for GPIO12 - Red   LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    /* set the pin function to OUTPUT for GPIO13 - Green LED light   */
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    
    /* set the pin function to INPUT for GPIO26 - for a switch input */
    io->gpio.GPFSEL2.field.FSEL6 = GPFSEL_INPUT;
    
    
    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
  
    printf( "Press Switch 1 to quit\n");

    while (GPIO_READ(&(io->gpio), 26) != 0)
     /* Do NOT use (GPIO_READ(&(io->gpio), 26) == 1) test case    */
     /* function GPIO_READ(&(io->gpio), 26) will return           */
     /* any of 1, 2, 4, 8, 16, ...   value                        */

    {
      GPIO_SET( &(io->gpio), 12);     // Turn on  Red   LED
      GPIO_CLR( &(io->gpio), 13);     // Turn off Green LED
      usleep(100*1000);               // sleep 0.1 sec.
      GPIO_CLR( &(io->gpio), 12);     // Turn off Red   LED
      GPIO_SET( &(io->gpio), 13);     // Turn on  Green LED
      usleep(100*1000);               // sleep 0.1 sec.
    }
    
    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

    printf( "\n Now Switch 1 pressed (input GPIO26 pin switched), quiting ... \n");

  }

  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
