/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 Sample Program 2
* Revision V1.1
* Revision V2.1
* On 2/4/2018
* On 1/14/2022
* By Kyusun Choi
* 
***************************************************/

/* Homework 1 Sample Program 2
 * Fast LED blinking program example in C for 
 * Raspberry Pi 4 computer with OUTPUT GPIOs:
 * red   LED on GPIO12 (with 500 ohm resistor in series)
 * green LED on GPIO13 (with 500 ohm resistor in series)
 *   also configure INPUT GPIOs:
 * Switch 1 on GPIO26 (with 10K ohm resistor in series)
 * Switch 2 on GPIO27 (with 10K ohm resistor in series)
 * 
 * If switch not pressed (keep GPIO26 pin high with 3.3V and 10Kohm pull-up)
 *   LED turn-on and LED turn-off in 0.2 second cycle, repeat - blinking fast
 * If switch pressed (keep GPIO26 pin low with 0V)
 *   quit this program
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

    /* set the pin function to OUTPUT for GPIO12 - red   LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light   */

    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    //io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    //io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;

    /* set the pin function to INPUT for GPIO26 - for a switch input */
    /* set the pin function to INPUT for GPIO27 - just for an example     */
    io->gpio.GPFSEL2.field.FSEL6 = GPFSEL_INPUT;

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);

    printf( "\n keep GPIO26 high (3.3V with 10Kohm pull-up) to work\n");
    printf( " switch GPIO26 to low (0.0V) to quit\n");

    while (GPIO_READ(&(io->gpio), 26) != 0)
     /* Do NOT use (GPIO_READ(&(io->gpio), 26) == 1) test case    */
     /* function GPIO_READ(&(io->gpio), 26) will return           */
     /* any of 1, 2, 4, 8, 16, ...                                */

    {
      GPIO_SET( &(io->gpio), 12);  // on
      GPIO_CLR( &(io->gpio), 13);  // off

      usleep(100*1000);    /* numbers in micro seconds (100msec.)  */

      GPIO_CLR( &(io->gpio), 12);  // off
      GPIO_SET( &(io->gpio), 13);  // on

      usleep(100*1000);    /* numbers in micro seconds (100msec.)  */
    }

    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

    printf( "\n Now input GPIO26 pin switched, quiting ... \n");

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
