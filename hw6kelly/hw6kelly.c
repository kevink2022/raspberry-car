/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Homework 5
* 2/20/2022 
* 
* Car Driving Program
* 
* By Kevin Kelly and Kyusun Choi
* 
***************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"

#define QUEUE_SIZE 100
#define STOP 0
#define FORWARD 2
#define BACKWARD 1

struct pause_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct queue_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool            done;
};

struct key_thread_parameter
{
  struct done_flag  * done;
  struct pause_flag * pause_left_motor;
  struct pause_flag * pause_right_motor;
  struct pause_flag * pause_clock;
  char              * control_queue;
  unsigned int      * control_queue_length;
  pthread_mutex_t   * control_queue_lock;
};

struct motor_thread_parameter
{
  char                          * current_command;
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  int                             PWM_pin;
  int                             I1_pin;
  int                             I2_pin;
  int                             IR_pin;
  struct pause_flag *             pause;
  struct done_flag *              done;
  bool                            left_motor;
  enum                      mode {
                                  MODE_1,
                                  MODE_2
                            } mode;
};

struct clock_thread_parameter
{
  int                 period;
  char              * current_command;
  char              * control_queue;
  unsigned int      * control_queue_length;
  struct pause_flag * pause;
  struct done_flag  * done;
  struct pause_flag * motor_pause_left;
  struct pause_flag * motor_pause_right;
  pthread_mutex_t   * control_queue_lock;
};

void *ThreadClock( void * arg  )
{
  #ifdef DEBUG
  printf("CLOCK INIT: cc thread\n"); 
  #endif
  struct clock_thread_parameter * parameter = (struct clock_thread_parameter *)arg;
  #ifdef DEBUG
  printf("CLOCK INIT: cc thread param\n");
  printf("CLOCK INIT: control queue addr: %lx\n", (unsigned long)parameter->control_queue);
  #endif
  *(char*)parameter->current_command = '\0';

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    #ifdef DEBUG
    printf("CLOCK: starting cc\n");
    #endif

    usleep(parameter->period * 1000000);

    #ifdef DEBUG
    printf("CLOCK: queue_len %i\n", *(unsigned int*)parameter->control_queue_length);
    #endif

    // Get commands out of queue
    pthread_mutex_lock( parameter->control_queue_lock );
    if (*(unsigned int*)parameter->control_queue_length > 0){
      #ifdef DEBUG
      printf("CLOCK: control queue addr: %lu\n", (unsigned long)parameter->control_queue);
      printf("CLOCK: in lock, curr_cmd: %c\n", *(char*)parameter->control_queue);
      #endif
      *(char*)parameter->current_command = *(char*)parameter->control_queue;
      
      #ifdef DEBUG
      printf("CLOCK: in lock, new curr_cmd: %c\n", *(char*)parameter->current_command);
      #endif

      *(unsigned int*)parameter->control_queue_length -= 1;
      
      #ifdef DEBUG
      printf("CLOCK: in lock, new queue_len: %i\n", *(unsigned int*)parameter->control_queue_length);
      #endif

      memmove(parameter->control_queue, parameter->control_queue + 1, *(unsigned int*)parameter->control_queue_length);

      // Pause motor queues so they update commands
      pthread_mutex_lock( &(parameter->motor_pause_left->lock) );
      parameter->motor_pause_left->pause = true;
      pthread_mutex_unlock( &(parameter->motor_pause_left->lock) );
      pthread_mutex_lock( &(parameter->motor_pause_right->lock) );
      parameter->motor_pause_right->pause = true;
      pthread_mutex_unlock( &(parameter->motor_pause_right->lock) );

      #ifdef DEBUG
      printf("\nCLOCK: exit update");
      #endif
    }
    pthread_mutex_unlock( parameter->control_queue_lock );
  }
}

// Bringing this down here for easier access temporarily
#define PWM_RANGE 100
#define PWM_MOTOR_MAX 95
#define PWM_MOTOR_MIN 20
#define PWM_SPEED_STEP 5
#define PWM_TURN_STEP 15
#define PWM_ORIENTATION 1
#define PWM_MODE2_STEP 15

#define DEBUG
#undef DEBUG
void *ThreadMotor( void * arg  )
{
  struct motor_thread_parameter * parameter = (struct motor_thread_parameter *)arg;
  int PWM = PWM_MOTOR_MIN, PWM_next = PWM_MOTOR_MIN;
  bool I1 = 0, I2 = 0, I1_next = 0, I2_next = 0;
  char current_command = '\0';
  bool left = parameter->left_motor;
  int mode = MODE_1;

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    // Only execute if any properties change
    if ((mode == MODE_1) && (PWM != PWM_next)){

      // Update properties
      PWM = PWM_next;

      #ifdef DEBUG  
      printf("\n%s MOTOR: Setting PWM: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
      #endif

      // Execute params
      if(left){
        parameter->pwm->DAT2 = PWM;
      } else {
        parameter->pwm->DAT1 = PWM;
      }

    } else if ( I1 && (mode == MODE_2)) {
      while(GPIO_READ(parameter->gpio, parameter->IR_pin) != 0) {
        if(left){
          parameter->pwm->DAT1 -= PWM_MODE2_STEP;
          parameter->pwm->DAT2 += PWM_MODE2_STEP;
        } else {
          parameter->pwm->DAT2 -= PWM_MODE2_STEP;
          parameter->pwm->DAT1 += PWM_MODE2_STEP;
        }
        printf("\n%s MOTOR: mode 2 turn step\n", parameter->left_motor ? "LEFT" : "RIGHT");
      }
      if(left){
        parameter->pwm->DAT2 = PWM;
      } else {
        parameter->pwm->DAT1 = PWM;
      }
    }

    if ((I1 != I1_next) || (I2 != I2_next)) {
      #ifdef DEBUG
      printf("\n%s MOTOR: Setting I1: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", I1);
      #endif

      I1 = I1_next;
      I2 = I2_next;

      #ifdef DEBUG  
      printf("\n%s MOTOR: PWM Pre Slow Stop: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
      #endif
      // Slow down during any mode change
      while(PWM > PWM_MOTOR_MIN){
        PWM -= PWM_SPEED_STEP;
        if(left){
          parameter->pwm->DAT2 = PWM;
        } else {
          parameter->pwm->DAT1 = PWM;
        }
        usleep(10000); // 0.01s
      }
      #ifdef DEBUG  
      printf("\n%s MOTOR: PWM Post Slow Stop: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
      #endif

      if (I1){
        //printf("\n%s MOTOR: Setting I1\n", parameter->left_motor ? "LEFT" : "RIGHT");  
        GPIO_SET( parameter->gpio, parameter->I1_pin );
      } else {
        //printf("\n%s MOTOR: Clearing I1\n", parameter->left_motor ? "LEFT" : "RIGHT");  
        GPIO_CLR( parameter->gpio, parameter->I1_pin );
      }

      #ifdef DEBUG
      printf("\n%s MOTOR: Setting I2: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", I2);
      #endif

      if (I2){
        //printf("\n%s MOTOR: Setting I2\n", parameter->left_motor ? "LEFT" : "RIGHT");  
        GPIO_SET( parameter->gpio, parameter->I2_pin );
      } else {
        //printf("\n%s MOTOR: Clearing I2\n", parameter->left_motor ? "LEFT" : "RIGHT");  
        GPIO_CLR( parameter->gpio, parameter->I2_pin );
      }

      #ifdef DEBUG  
      printf("\n%s MOTOR: PWM Pre Slow Go: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
      #endif
      // If moving, return to original speed
      if (I1 || I2){
        while(PWM < PWM_next){ //we use PWM_next here, as it will be the original speed.
          PWM += PWM_SPEED_STEP;
          if(left){
            parameter->pwm->DAT2 = PWM;
          } else {
            parameter->pwm->DAT1 = PWM;
          }
          usleep(10000); // 0.01s
        }
      }
      #ifdef DEBUG  
      printf("\n%s MOTOR: PWM Post Slow Go: %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
      #endif

      #ifdef DEBUG
      printf("\n%s MOTOR: Everything Set\n", parameter->left_motor ? "LEFT" : "RIGHT");
      #endif
    }

    pthread_mutex_lock( &(parameter->pause->lock) );
    if (parameter->pause->pause)
    {
      // Update params
      switch (*(char*)parameter->current_command)
      {
        case 's':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: STOP\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          I1_next = 0;
          I2_next = 0;
          break;
        case 'w':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: FORWARD\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          I1_next = 1;
          I2_next = 0;
          break;
        case 'x':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: BACKWARD\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          I1_next = 0;
          I2_next = 1;
          break;
        case 'i':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: FASTER\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          if (PWM < PWM_MOTOR_MAX) {PWM_next += PWM_SPEED_STEP;}
          #ifdef DEBUG
          printf("%s MOTOR: PWM = %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM_next);
          #endif
          break;
        case 'j':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: SLOWER\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          if(PWM > PWM_MOTOR_MIN){PWM_next -= PWM_SPEED_STEP;}
          #ifdef DEBUG
          printf("%s MOTOR: PWM = %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM_next);
          #endif
          break;
        case 'a':
          #ifdef DEBUG
          printf("\n%s MOTOR: Recieved Command: LEFT\n", parameter->left_motor ? "LEFT" : "RIGHT");
          #endif
          if(parameter->left_motor){
            if(PWM > PWM_MOTOR_MIN){PWM_next -= PWM_TURN_STEP;}
          } else {
            if (PWM < 100) {PWM_next += PWM_TURN_STEP;}
          }
          #ifdef DEBUG
          printf("%s MOTOR: PWM = %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM_next);
          #endif
          break;
        case 'd':
          #ifdef DEBUG
          printf("\nMOTOR: Recieved Command: RIGHT\n");
          #endif
          if(parameter->left_motor){
            if (PWM < PWM_MOTOR_MAX) {PWM_next += PWM_TURN_STEP;}
          } else {
            if(PWM > PWM_MOTOR_MIN){PWM_next -= PWM_TURN_STEP;}
          }
          #ifdef DEBUG
          printf("%s MOTOR: PWM = %i\n", parameter->left_motor ? "LEFT" : "RIGHT", PWM);
          #endif
          break;
        case '1':
          #ifdef DEBUG
          printf("\nMOTOR: Recieved Command: RIGHT\n");
          #endif
          mode = MODE_1;
          break;
        case '2':
          #ifdef DEBUG
          printf("\nMOTOR: Recieved Command: RIGHT\n");
          #endif
          mode = MODE_2;
          break;  
        default:
          break;
      }
      parameter->pause->pause = false;
    }
    pthread_mutex_unlock( &(parameter->pause->lock) );
  }
}
#undef DEBUG

int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
}

#define DEBUG
void *ThreadKey( void * arg )
{
  struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  bool done;

  do
  {
    switch (get_pressed_key())
    {
      case 'q':
        done = true;
        /* unpause everything */
        printf("KEY_THREAD: Recieved Command: QUIT\n");

        /* indicate that it is time to shut down */
        pthread_mutex_lock( &(thread_key_parameter->done->lock) );
        thread_key_parameter->done->done = true;
        pthread_mutex_unlock( &(thread_key_parameter->done->lock) );
        break;
        break;
      case 's':
        printf("\nKEY_THREAD: Recieved Command: STOP\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 's';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'w':
        printf("KEY_THREAD: Recieved Command: FORWARD\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'w';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: FORWARD\nKEY_THREAD: Queue Lengh: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'x':
        printf("KEY_THREAD: Recieved Command: BACKWARD\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'x';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: BACKWARD\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'i':
        printf("KEY_THREAD: Recieved Command: FASTER\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'i';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: SLOWER\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'j':
        printf("KEY_THREAD: Recieved Command: SLOWER\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'j';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: SLOWER\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'a':
        printf("KEY_THREAD: Recieved Command: LEFT\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'a';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: LEFT\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'd':
        printf("KEY_THREAD: Recieved Command: RIGHT\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'd';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          #ifdef DEBUG
          printf("KEY_THREAD: Added to Queue: RIGHT\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
          #endif
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      
      case 'm':
        
        switch(get_pressed_key())
        {
          case '1':
            printf("KEY_THREAD: Recieved Command: MODE 1\n");

            // Lock Control Thread
            pthread_mutex_lock( thread_key_parameter->control_queue_lock );
            if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
              *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = '1';
              *(unsigned int*)thread_key_parameter->control_queue_length += 1;
              #ifdef DEBUG
              printf("KEY_THREAD: Added to Queue: MODE 1\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
              #endif
            }
            pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
            break;

          case'2':
            printf("KEY_THREAD: Recieved Command: MODE 2\n");

            // Lock Control Thread
            pthread_mutex_lock( thread_key_parameter->control_queue_lock );
            if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
              *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = '2';
              *(unsigned int*)thread_key_parameter->control_queue_length += 1;
              #ifdef DEBUG
              printf("KEY_THREAD: Added to Queue: MODE 2\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
              #endif
            }
            pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
            break;

          default:
            break;
        }
        break;
      default:
        break;
    }
  } while (!done);
  printf( "KEY_THREAD: Exiting...\n" );

  return (void *)0;
}
#undef DEBUG
#define DEBUG
int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread_key_handle;
  struct key_thread_parameter     thread_key_parameter;

  pthread_t                       thread_left_motor_handle;
  pthread_t                       thread_right_motor_handle;
  pthread_t                       thread_clock_handle;
  struct motor_thread_parameter   thread_left_motor_parameter;
  struct motor_thread_parameter   thread_right_motor_parameter;
  struct clock_thread_parameter   thread_clock_parameter;

  struct pause_flag               pause_left_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause_right_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               set_motors = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause_clock = {PTHREAD_MUTEX_INITIALIZER, false};
  struct done_flag                done   = {PTHREAD_MUTEX_INITIALIZER, false};

  pthread_mutex_t                 queue_lock = PTHREAD_MUTEX_INITIALIZER;

  char *queue;
  queue = calloc(QUEUE_SIZE, sizeof(char));
  #ifdef DEBUG
  printf("MAIN: queue addr: %lx", (unsigned long)queue);
  #endif

  unsigned int *queue_len;
  queue_len = calloc(1, sizeof(unsigned int));
  #ifdef DEBUG
  printf("MAIN: queue_len addr: %lx", (unsigned long)queue_len);
  #endif

  char *curr_cmd;
  curr_cmd = calloc(1, sizeof(char));
  #ifdef DEBUG
  printf("MAIN: queue_len addr: %lx", (unsigned long)curr_cmd);
  #endif

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );

    enable_pwm_clock( io );

    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;

    /* set the pin function to alternate function 0 for GPIO12 */
    /* set the pin function to alternate function 0 for GPIO13 */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;

    /* configure the PWM channels */
    io->pwm.RNG1 = PWM_RANGE;     /* the default value */
    io->pwm.RNG2 = PWM_RANGE;     /* the default value */
    io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */
    
    // CLOCK
    thread_clock_parameter.period = .01;
    thread_clock_parameter.pause = &pause_clock;
    thread_clock_parameter.done = &done;
    thread_clock_parameter.control_queue_lock = &queue_lock;
    thread_clock_parameter.control_queue_length = queue_len;
    thread_clock_parameter.control_queue = queue;
    thread_clock_parameter.current_command = curr_cmd;
    thread_clock_parameter.motor_pause_left = &pause_left_motor;
    thread_clock_parameter.motor_pause_right = &pause_right_motor;

    // KEY
    thread_key_parameter.done = &done;
    thread_key_parameter.pause_left_motor = &pause_left_motor;
    thread_key_parameter.pause_right_motor = &pause_right_motor;
    thread_key_parameter.pause_clock = &pause_clock;
    thread_key_parameter.control_queue = queue;
    thread_key_parameter.control_queue_length = queue_len;
    thread_key_parameter.control_queue_lock = &queue_lock;

    // LEFT
    thread_left_motor_parameter.pause = &pause_left_motor;
    thread_left_motor_parameter.done = &done;
    thread_left_motor_parameter.PWM_pin = 12;
    thread_left_motor_parameter.I1_pin = 5;
    thread_left_motor_parameter.I2_pin = 6;
    thread_left_motor_parameter.IR_pin = 24;
    thread_left_motor_parameter.gpio = &(io->gpio);
    thread_left_motor_parameter.pwm = &(io->pwm);
    thread_left_motor_parameter.left_motor = true;
    thread_left_motor_parameter.current_command = curr_cmd;
    thread_left_motor_parameter.mode = MODE_1;

    // RIGHT
    thread_right_motor_parameter.pause = &pause_right_motor;
    thread_right_motor_parameter.done = &done;
    thread_right_motor_parameter.PWM_pin = 13;
    thread_right_motor_parameter.I1_pin = 22;
    thread_right_motor_parameter.I2_pin = 23;
    thread_right_motor_parameter.IR_pin = 25;
    thread_right_motor_parameter.gpio = &(io->gpio);
    thread_right_motor_parameter.pwm = &(io->pwm);
    thread_right_motor_parameter.left_motor = false;
    thread_right_motor_parameter.current_command = curr_cmd;
    thread_right_motor_parameter.mode = MODE_1;

    // THREADS
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    pthread_create( &thread_left_motor_handle, 0, ThreadMotor, (void *)&thread_left_motor_parameter );
    pthread_create( &thread_right_motor_handle, 0, ThreadMotor, (void *)&thread_right_motor_parameter );
    pthread_create( &thread_clock_handle, 0, ThreadClock, (void *)&thread_clock_parameter);
    pthread_join( thread_key_handle, 0 );
    pthread_join( thread_left_motor_handle, 0 );
    pthread_join( thread_right_motor_handle, 0 );
    pthread_join( thread_clock_handle, 0 );
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    printf("\nExiting Program...\n");
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
#undef DEBUG