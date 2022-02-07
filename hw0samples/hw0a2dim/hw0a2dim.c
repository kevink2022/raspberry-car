/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a2dim - Red   LED at GPIO12 Off
*                         - Green LED at GPIO13 Dimmed to 5% light level
*                         - ending by 'ctl c' key hits
* 
* Revision V1.1   On 1/22/2022
* By Kyusun Choi
* 
* Red   LED on GPIO12 (with 500 ohm resistor in series)
* Green LED on GPIO13 (with 500 ohm resistor in series)
* 
* Red   LED at GPIO12 Off
* Green LED at GPIO13 Dimmed to 5% light level at program start
*   - ending by 'ctl c' key hits
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


void DimLevUnit(int Level, volatile struct io_peripherals *io)
{
      int  ONcount, OFFcount;

      ONcount  =     Level;
      OFFcount = 100-Level;

      /* create the output pin signal duty cycle, same as Level */
      GPIO_SET( &(io->gpio), 13 );   /* Turn ON  LED at GPIO 13 */

      while (ONcount > 0)
      {
        usleep( 100 );
        ONcount = ONcount -1;
      }

      GPIO_CLR( &(io->gpio), 13 );  /* Turn OFF LED at GPIO 13 */

      while (OFFcount > 0)
      {
        usleep( 100 );
        OFFcount = OFFcount -1;
      }
}


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
    
    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);
  
    printf( "\n Press 'ctl c' to quit.\n");
    
    while (1)
    {
      DimLevUnit(5, io);  /* dim green LED to 5% light level, and repeat */
    }
    
    /* Waiting for 'ctl c', below lines will not be done.  */
    
    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

    printf( "\n 'ctl c' key hit, now quiting ... \n");
    
  }

  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
