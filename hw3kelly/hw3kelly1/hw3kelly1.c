/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 3 Program 1
* 2/2/2022 
* 
* By Kevin Kelly and Kyusun Choi
* 
***************************************************/

/* Homework 2 Sample Program 1
 * Slow LED blinking program example in C for 
 * Raspberry Pi 4 computer with 
 * Red LED and Blue LED: 
 *    --- connected in series with a 430 ohm and 470 ohm resistor
 *    --- connected in series with GPIO 12
 * Green LED and Yellow LED: 
 *    --- connected in series with a 430 ohm and 470 ohm resistor
 *    --- connected in series with GPIO 22
 * Turn on each LED in sequence, switching every 0.25 seconds
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>     // Inlcuded for multithreading
#include <sys/wait.h>   // Inlcuded for multithreading
#include <signal.h>     // Inlcuded for multithreading
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

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red    LED light */
    /* set the pin function to OUTPUT for GPIO22 - green  LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO12
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO22
    
    printf( "\nPress 'q' to quit\n");

    bool running = true;
    int input;
    
    // initialize live input mode
    struct termios attr;
    tcgetattr(0, &attr);
    attr.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &attr);

    // create child pid
    pid_t child_pid = fork();
    if (child_pid == -1) { printf("fork error"); return 1; }
    
    // child process
    if (child_pid == 0) {    
      // loop LEDs
      while (1)
      {
        // Make 12 an output to control RED and GREEN LED
        // Make 22 an input to clear BLUE and ORANGE LED
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO12
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;  //GPIO22
      
        // Clear 12, turning on RED LED
        GPIO_CLR( &(io->gpio), 12);

        usleep(500*1000);

        // Set 12, turning on GREEN LED
        GPIO_SET( &(io->gpio), 12);

        usleep(500*1000);

        
        // Make 12 an input to clear RED and GREEN LED
        // Make 22 an output to contorl BLUE and ORANGE LED
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;  //GPIO12
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO22

        // Clear 22, turning on BLUE LED
        GPIO_CLR( &(io->gpio), 22);

        usleep(500*1000);
        
        // Set 22, turning on ORANGE LED
        GPIO_SET( &(io->gpio), 22);

        usleep(500*1000);
      }
    }
    
    // parent process
    else{
    
      // loop, wait for 'q'
      while(running) {
        input = get_pressed_key();
        if(input == 'q') {
          running = false;
        }
      }
      
      // kill child process
      kill(child_pid, SIGKILL);
      
  }

  }
  else
  {
    ; /* warning message already issued */
  }
  
  printf("\n Quitting... \n\n");

  /* clean the GPIO pins */
  io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
  io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;

  return 0;
}
