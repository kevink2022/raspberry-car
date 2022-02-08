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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>     
#include <sys/wait.h>   
#include <signal.h>     
#include <pthread.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"

typedef struct 
{
  volatile struct gpio_register * gpio;
  int                             pin;
  int                             base;
  int                             set;
  double                          time;
  bool                            cycle;
  bool                            smooth;
  bool                            running;
} pwm_thread_param;

void* phase_pwm(void * arg){
  pwm_thread_param * param = (pwm_thread_param *)arg;
  int i, cycle_count, old_base;
  int phase_cycle, phase_cycle_count, phase_step, phase_step_count; 
  
  // Function got too bloated, had to fine tune with these
  int u_sleep = 10, cycle_mult = 125;

  while(param->running) {
    //If base == set, 
    if(param->base == param->set) {
      if(param->cycle) {
        param->base = param->set;  // If cycling, just switch the base percentage
        param->set = old_base;     // and the percentage to set it to
      } 
      else {
        while((param->base == param->set) && param->running) {
          // PWM CYCLE
          cycle_count = 0;
          GPIO_SET(param->gpio,param->pin);

          while(cycle_count < param->base) {
            cycle_count++;
            usleep(u_sleep);
          }
          GPIO_CLR(param->gpio,param->pin);
          while (cycle_count < 100){
            cycle_count++;
            usleep(u_sleep);
          }
          // PWM CYCLE
        }
      }
    } else if (param->time == 0) {
      param->base = param->set;
    } else {
      old_base = param->base;                                     // Remember base for cycling
      phase_cycle = 0;                                            // Initial cycle
      phase_cycle_count = ((int)(param->time*cycle_mult));               // Number of cycles
      phase_step = (param->set > param->base) ? 1 : -1;           // Getting brighter or dimmmer
      phase_step_count = (phase_step)*(param->set - param->base); // Number of steps (change percent by 1) in phase
    
      while(param->running && (phase_cycle < phase_cycle_count)) {
          // PWM CYCLE
          cycle_count = 0;
          GPIO_SET(param->gpio,param->pin);

          while(cycle_count < param->base) {
            cycle_count++;
            usleep(u_sleep);
          }
          GPIO_CLR(param->gpio,param->pin);
          while (cycle_count < 100){
            cycle_count++;
            usleep(u_sleep);
          }
          // PWM CYCLE

          // This has the potential to arrive at set 1/10th of a second early
          phase_cycle++;
          if(param->smooth && param->base != param->set && (phase_cycle % (phase_cycle_count/phase_step_count) == 0)) {
            param->base += phase_step;
          }
        }
      param->base = param->set;
    }
  }

  // Cleanup
  GPIO_CLR(param->gpio,param->pin);
  return NULL;
}

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

    pthread_t red_green_thread, blue_orange_thread;
    pwm_thread_param *red_green_param;
    pwm_thread_param *blue_orange_param;
    
    red_green_param = malloc(sizeof(pwm_thread_param));
    blue_orange_param = malloc(sizeof(pwm_thread_param));

    red_green_param->gpio = &(io->gpio);
    red_green_param->pin = 12;
    red_green_param->base = 100;
    red_green_param->set = 75;
    red_green_param->time = 2;
    red_green_param->cycle = true;
    red_green_param->smooth = true;;
    red_green_param->running = true;
    
    blue_orange_param->gpio = &(io->gpio);
    blue_orange_param->pin = 22;
    blue_orange_param->base = 97;
    blue_orange_param->set = 87;
    blue_orange_param->time = 1.5;
    blue_orange_param->cycle = true;
    blue_orange_param->smooth = true;
    blue_orange_param->running = true;

    bool running = true;
    int input;
    
    // initialize live input mode
    struct termios attr;
    tcgetattr(0, &attr);
    attr.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &attr);

    pthread_create(&red_green_thread, NULL, phase_pwm, (void *)red_green_param);
    pthread_create(&blue_orange_thread, NULL, phase_pwm, (void *)blue_orange_param);
    
    // loop, wait for 'q'
    while(running) {
      input = get_pressed_key();
      if(input == 'q') {
        red_green_param->running = false;
        blue_orange_param->running = false;
        pthread_join(red_green_thread, NULL);
        pthread_join(blue_orange_thread, NULL);
        io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
        io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
        running = false;
      }
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
