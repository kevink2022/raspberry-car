/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a1blikey2 - Red   LED at GPIO12 Blinking
*                             - Green LED at GPIO13 Blinking
*                             - by single keyboard hit (without Enter Key hit)
*           - Check STDIN before reading, get char if available, 
*           - this is effectively NONBLOCK-ing read.
*           - include <sys/select.h> and <sys/time.h>
* Revision V1.1   On 1/18/2022
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
#include <sys/select.h>
#include <sys/time.h>
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
  int             ch = 0;

  /* Make input available immediately, without 'enter' key,       */
  /* no input processing, disable line editing, noncanonical mode */
  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  /* Check STDIN before reading, get char if available */
  /*   if not available, char = 0                      */
  /*   use 'select' to check, use 'read' to get char   */
  fd_set rd;
  struct timeval tv;
  int err;
  FD_ZERO(&rd);
  FD_SET(0, &rd);
  tv.tv_sec=0;
  tv.tv_usec=1000;   /* timeout of 1ms */
//  tv.tv_usec=100;
  err = select(1, &rd, NULL, NULL, &tv);  //check STDIO 
  if (err > 0) {
    read(STDIN_FILENO, &ch, 1);           //read STDIO 
  }

  //reset the input to the orginal settings
  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

//  printf( "Key= %d\n", ch);

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
