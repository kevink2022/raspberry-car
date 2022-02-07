/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a3pth3parm 
* 
*   - function parameter passing updated from hw0a3pth2func.c
*   - Create two functions: RedBlink and GreenBlink
*   - Red   LED at GPIO12 Blink 5Hz
*   - Green LED at GPIO13 Blink 5Hz
*   - ending by blink count finish, count = 15
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


void RedBlink(int count, volatile struct gpio_register *gpio)
{
  int i = 0;
  
  while(i < count)
  {
    GPIO_SET(gpio,12);  // Red LED on, GPIO12
    usleep(100*1000);   // numbers in micro seconds (100msec.)
    GPIO_CLR(gpio,12);  // Red LED off, GPIO12
    usleep(100*1000);   // numbers in micro seconds (100msec.)
    ++i;
  }
}


void GreenBlink(int count, volatile struct gpio_register *gpio)
{
  int i = 0;
  
  while(i < count)
  {
    GPIO_SET(gpio,13);  // Green LED on, GPIO13
    usleep(100*1000);   // numbers in micro seconds (100msec.)
    GPIO_CLR(gpio,13);  // Green LED off, GPIO13
    usleep(100*1000);   // numbers in micro seconds (100msec.)
    ++i;
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

    /* set the pin function to OUTPUT for GPIO12 - red   LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);


//  RedBlink(15,io);               // Blink Red   LED first
//  GreenBlink(15,io);             // Blink Green LED next
    RedBlink(15,&(io->gpio));      // main task 1: blink Red   LED 15X
    GreenBlink(15,&(io->gpio));    // main task 2: blink Green LED 15X


    /* main task finished  */

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
