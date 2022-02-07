/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a3pth5busy 
* 
*   - Use Busy Wait function instead of 'sleep()' or 'usleep()' function
*   - Watch the Raspberry Pi 4 processor core utilation with 'htop' command
* 
*   - Multi-tasking with pthread, two thread example
*   - Run two functions simultaneously with two separate threads
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


struct my_thread_param
{
  volatile struct gpio_register * gpio;
  int                             count;
};


void dly4500us()
{
  int i = 1;
  while(i <= 1000000)
  {
    ++i;
  }
}


void dly500ms()
{
  int i = 1;
  while(i <= 108)     // 108 => 0.5sec
  {
    dly4500us();
    ++i;
  }
}


void *RedBlink(void * arg)
{
  struct my_thread_param * param = (struct my_thread_param *)arg;
  int i = 0;
  
  while(i < param->count)      // Blink Red LED 'count' times
  {
    GPIO_SET(param->gpio,12);  // Red LED on, GPIO12
    dly500ms();                // Busy delay
    GPIO_CLR(param->gpio,12);  // Red LED off, GPIO12
    dly500ms();                // Busy delay
    ++i;
  }
  
  return NULL;
}


void *GreenBlink(void * arg)
{
  struct my_thread_param * param = (struct my_thread_param *)arg;
  int i = 0;
  
  while(i < param->count)      // Blink Green LED 'count' times
  {
    GPIO_SET(param->gpio,13);  // Green LED on, GPIO13
    dly500ms();                // Busy delay
    GPIO_CLR(param->gpio,13);  // Green LED off, GPIO13
    dly500ms();                // Busy delay
    ++i;
  }
  
  return NULL;
}


int main( void )
{
  volatile struct io_peripherals *io;

  pthread_t t1r;
  pthread_t t2g;
  struct my_thread_param   red_param;
  struct my_thread_param   green_param;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red   LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light   */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;

    red_param.gpio = &(io->gpio);
    red_param.count = 25;             // Red LED 25X blinking
    green_param.gpio = &(io->gpio);
    green_param.count = 15;           // Green LED 15X blinking

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 13);


    // Create two threads t1r and t2g, and run them in parallel
    pthread_create(&t1r, NULL, RedBlink, (void *)&red_param);
    pthread_create(&t2g, NULL, GreenBlink, (void *)&green_param);

    // Wait to finish both t1r and t2g threads
    pthread_join(t1r, NULL);
    pthread_join(t2g, NULL);


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
