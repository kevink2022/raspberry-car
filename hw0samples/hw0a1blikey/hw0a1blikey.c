/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a1blikey - Red   LED at GPIO12 Blinking
*                            - Green LED at GPIO13 Blinking
*                            - by keyboard hits
* Revision V1.1   On 1/18/2022
* By Kyusun Choi
* 
* Red   LED on GPIO12 (with 500 ohm resistor in series)
* Green LED on GPIO13 (with 500 ohm resistor in series)
* 
* 'r' => red   LED on/off, hit 'r' key to toggle red   LED on/off
* 'g' => green LED on/off, hit 'g' key to toggle green LED on/off
*  hit any other key to quit
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

int get_pressed_key(void)
{
  int             ch;

  ch = getchar();

  return ch;
}


int main( void )
{
  volatile struct io_peripherals *io;
  bool  done = false;
  
  io = import_registers();

  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - Red   LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    /* set the pin function to OUTPUT for GPIO13 - Green LED light   */
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;
    
    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
  
    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " any other character will exit\n");
    

    do
    {
      switch (get_pressed_key())
      {
        case 'r':
          if (GPIO_READ(&(io->gpio), 12) == 0)
          {
            GPIO_SET( &(io->gpio), 12);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 12);
          }
          break;

        case 'g':
          if (GPIO_READ(&(io->gpio), 13) == 0)
          {
            GPIO_SET( &(io->gpio), 13);
          }
          else
          {
            GPIO_CLR( &(io->gpio), 13);
          }
          break;

        case 10:               /* it is Line Feed, New Line, or Enter Key character */
          usleep(100*1000);    /* numbers in micro seconds (100msec.)  */
          break;

        default:
          done = true;
          break;
      }
    } while (!done);


    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

    printf( "\n Key hit is not 'r' or 'g' key, now quiting ... \n");
    
  }

  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
