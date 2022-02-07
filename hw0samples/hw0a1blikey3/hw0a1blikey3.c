/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a1blikey3 - Red   LED at GPIO12 Blinking
*                             - Green LED at GPIO13 Blinking
*                             - by single keyboard hit (without Enter Key hit)
*                             - and set STDIO mode to NONBLOCK
* Revision V1.1   On 2/6/2022
* By Kyusun Choi
* 
* Red   LED on GPIO12 (with 500 ohm resistor in series)
* Green LED on GPIO13 (with 500 ohm resistor in series)
* 
* red   LED and green LED alternate blinking, 
*  hit 'q' key to quit
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
  struct termios  original_attributes;
  struct termios  modified_attributes;
  long   oldf, newf;
  int    ch;

  /* Make input available immediately, without 'enter' key,       */
  /* no input processing, disable line editing, noncanonical mode */
  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  /* Save the mode of STDIN, then      <use fcntl>   */
  /*   change the mode to NONBLOCK, then get char,   */
  /*   and then restore the STDIN mode back.         */
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  newf = oldf | O_NONBLOCK;
  fcntl(STDIN_FILENO, F_SETFL, newf);
  ch = getchar();                       //get the character
  fcntl(STDIN_FILENO, F_SETFL, oldf);   //restore the STDIN mode back

  //reset the input to the orginal settings
  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  //printf( "Key= %d\n", ch);

  return ch;
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
  
    printf( " press 'q' to quit\n");
    
    while ( (int)('q') != get_pressed_key() )
    {
      GPIO_SET( &(io->gpio), 12);     // Turn on  Red   LED
      GPIO_CLR( &(io->gpio), 13);     // Turn off Green LED
      usleep(50*1000);                // sleep 0.1 sec.
      GPIO_CLR( &(io->gpio), 12);     // Turn off Red   LED
      GPIO_SET( &(io->gpio), 13);     // Turn on  Green LED
      usleep(50*1000);                // sleep 0.1 sec.
    }


    /* clean the GPIO pins */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

    printf( "\n now quiting ... \n");
    
  }

  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
