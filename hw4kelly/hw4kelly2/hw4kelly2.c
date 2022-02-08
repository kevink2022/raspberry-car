/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 3 Program 2
* 2/2/2022
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
(1) ‘i’ key hit – increase Red LED light level by 5%,
(2) ‘j’ key hit – decrease Red LED light level by 5%,
(3) ‘r’ key hit – set Red LED light level to 0%,
(4) ‘h’ key hit – set Red LED light level to 50%,
(5) ‘m’ key hit – set Red LED light level to 100%,
(6) ‘w’ key hit – turn-on Orange LED for 2 seconds,
(7) ‘x’ key hit – turn-on Blue LED for 2 seconds,
(8) ‘s’ key hit – turn-on both Orange LED and Blue LED for 3 seconds,
(9) ‘q’ key hit – quit program.
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
#include <pthread.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "wait_period.h"

typedef struct 
{
  volatile struct io_peripherals *io;
  volatile struct gpio_register  *gpio;
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
      phase_cycle_count = ((int)(param->time*cycle_mult));        // Number of cycles
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

void* led_sleep_22(void * arg){
  pwm_thread_param * param = (pwm_thread_param *)arg;
  
  param->io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
  int time = (int)(param->time*125), cycle_count;
  
  while(param->running && time) {
    // PWM CYCLE
    cycle_count = 0;
    GPIO_SET(param->gpio,22);

    while(cycle_count < param->base) {
      cycle_count++;
      usleep(10);
    }
    GPIO_CLR(param->gpio,22);
    while (cycle_count < 100){
      cycle_count++;
      usleep(10);
    //PWM CYCLE
    }
    time--;
  }
  
  GPIO_CLR(param->gpio,22);
  param->io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
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
  bool  done;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;

    /* set initial output state - off */
    GPIO_CLR(&(io->gpio), 12);
    GPIO_CLR(&(io->gpio), 22);
    
    printf("b4 prints");
    
    // print directions
    printf( "\n press 'r' to toggle the red LED\n");
    printf( " press 'g' to toggle the green LED\n");
    printf( " press 'b' to toggle the blue LED\n");
    printf( " press 'y' to toggle the yellow LED\n");
    printf( " press 'c' to turn off all LED\n");
    printf( " press 'q' to fak off\n\n");
    
    printf("after prints");


    //sleep(2);
    printf("trying tred");
    //sleep(2);
    pthread_t red_green_thread, blue_orange_thread;
    printf("made tred");
    //sleep(2);
    pwm_thread_param *red_green_param;
    pwm_thread_param *blue_orange_param;
    printf("made param");
    //sleep(2);
    
    red_green_param = malloc(sizeof(pwm_thread_param));
    blue_orange_param = malloc(sizeof(pwm_thread_param));

    red_green_param->io = io;
    red_green_param->gpio = &(io->gpio);
    red_green_param->pin = 12;
    red_green_param->base = 0;
    red_green_param->set = 0;
    red_green_param->time = 0;
    red_green_param->cycle = false;
    red_green_param->smooth = false;
    red_green_param->running = true;
    
    blue_orange_param->io = io;
    blue_orange_param->gpio = &(io->gpio);
    blue_orange_param->pin = 22;
    blue_orange_param->base = 0;
    blue_orange_param->set = 0;
    blue_orange_param->time = 2;
    blue_orange_param->cycle = false;
    blue_orange_param->smooth = false;
    blue_orange_param->running = true;
    
    printf("set params");

    
    bool running = true;
    int input;
    
    // initialize live input mode
    struct termios attr;
    tcgetattr(0, &attr);
    attr.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &attr);
    
    printf("live input done");
    
    pthread_create(&red_green_thread, NULL, phase_pwm, (void *)red_green_param);

    printf("tred crated");

    while(running)
    {
      input = get_pressed_key();
      // increase Red LED light level by 5%,
      if (input == 'i') {
        red_green_param->set = (red_green_param->set - 5 < 0) ? 0 : red_green_param->set - 5;
      }
      //
      else if(input == 'j') {
        red_green_param->set = (red_green_param->set + 5 > 100) ? 100 : red_green_param->set + 5;
      }
      //
      else if(input == 'r') {
        red_green_param->set = 100;
      }
      //
      else if(input == 'h') {
        red_green_param->set = 50;
      }
      //
      else if(input == 'm') {
        red_green_param->set = 0;
      }
      //
      else if(input == 'w') {
        blue_orange_param->time = 2;
        blue_orange_param->base = 0;
        pthread_create(&blue_orange_thread, NULL, led_sleep_22, (void *)blue_orange_param);
        pthread_detach(blue_orange_thread);
      }
      //
      else if(input == 'x') {
        blue_orange_param->time = 2;
        blue_orange_param->base = 100;
        pthread_create(&blue_orange_thread, NULL, led_sleep_22, (void *)blue_orange_param);
        pthread_detach(blue_orange_thread);
      }
      //
      else if(input == 's') {
        blue_orange_param->time = 3;
        blue_orange_param->base = 50;
        pthread_create(&blue_orange_thread, NULL, led_sleep_22, (void *)blue_orange_param);
        pthread_detach(blue_orange_thread);
      }
      //
      else if(input == 'q') {
        red_green_param->running = false;
        blue_orange_param->running = false;
        pthread_join(red_green_thread, NULL);
        //pthread_join(blue_orange_thread, NULL);
        usleep(10); //Allow time for potential blue_orange thread to cleanup
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
