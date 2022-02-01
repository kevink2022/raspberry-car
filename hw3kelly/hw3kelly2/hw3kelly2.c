/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 1 Sample Program 3
* 1/19/2021
* 
* By Kevin Kelly
* 
***************************************************/

/* Homework 1 Live Input 
 * simple LED on/off program example in C for 
 * Raspberry Pi 4 computer with 
 * red    LED on GPIO12 (with 430 ohm resistor in series)
 * green  LED on GPIO13 (with 470 ohm resistor in series)
 * blue   LED on GPIO23 (with 470 ohm resistor in series)
 * yellow LED on GPIO24 (with 470 ohm resistor in series)
 * 
 * 'r' => red    LED on, all other LED off
 * 'g' => green  LED on, all other LED off
 * 'b' => blue   LED on, all other LED off
 * 'y' => yellow LED on, all other LED off
 * 'a' => all    LED on
 * 'c' => all    LED off
 * 'q' => quit program
 * hit any other key to quit
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


int get_pressed_key(void)
{
  int ch;

  ch = getchar();

  return ch;
}


int main( void )
{
  volatile struct io_peripherals *io;
  bool  done;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    

    /* set the pin function to INPUT for GPIO27 - just for an example   */
    // io->gpio.GPFSEL2.field.FSEL6 = GPFSEL_INPUT; <-- unused

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 22);
    

    
    // print directions
    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " press 'b' to toggle the blue LED\n");
    printf( " press 'y' to toggle the yellow LED\n");
    printf( " press 'c' to turn off all LED\n");
    printf( " press 'q' to quit the program\n\n");

    
    bool running = true;
    int input;
    
    // initialize live input mode
    struct termios attr;
    tcgetattr(0, &attr);
    attr.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &attr);
    
    while(running)
    {
      input = get_pressed_key();
      // All LED off
      if (input == 'c') {
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
      }
      // Red LED on
      else if(input == 'r') {
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
        GPIO_SET( &(io->gpio), 12);
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
      }
      // Green LED on 
      else if(input == 'g') {
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
        GPIO_CLR( &(io->gpio), 12);
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
      }
      // Blue LED on
      else if(input == 'b') {
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
        GPIO_CLR( &(io->gpio), 22);
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
      }
      // Yellow LED on
      else if(input == 'y') {
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
        GPIO_SET( &(io->gpio), 22);
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
      }
      // Quit program
      else if(input == 'q') {
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
        running = false;
      }
    } 
      
    printf( "\n\n Quitting... \n\n");

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
