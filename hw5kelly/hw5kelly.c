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

#define PWM_RANGE 100
#define QUEUE_SIZE 100

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
  struct done_flag *  done;
  struct pause_flag * pause_control;
  struct pause_flag * pause_left_motor;
  struct pause_flag * pause_right_motor;
  struct pause_flag * pause_clock;
  char              * control_queue;
  unsigned int      * control_queue_length;
  pthread_mutex_t   * control_queue_lock;
};

struct control_thread_parameter
{
  char                queue[QUEUE_SIZE];
  int                 queue_length;
  struct pause_flag * pause;
  struct done_flag  * done;
};

struct motor_thread_parameter
{
  char                          * current_command;
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  int                             PWM_pin;
  int                             I1_pin;
  int                             I2_pin;
  struct pause_flag *             pause;
  struct done_flag *              done;
  bool                            left_motor;
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


int  Tstep = 50;  /* PWM time resolution, number used for usleep(Tstep) */
int  Tlevel = 5;  /* repetition count of each light level, eg. repeat 12% light level for 5 times. */

void DimLevUnit(int Level, int pin, volatile struct gpio_register *gpio)
{
  int ONcount, OFFcount;

  ONcount = Level;
  OFFcount = 100 - Level;

  /* create the output pin signal duty cycle, same as Level */
  GPIO_SET( gpio, pin ); /* ON LED at GPIO 18 */
  while (ONcount > 0)
  {
    usleep( Tstep );
    ONcount = ONcount - 1;
  }
  GPIO_CLR( gpio, pin ); /* OFF LED at GPIO 18 */
  while (OFFcount > 0)
  {
    usleep( Tstep );
    OFFcount = OFFcount - 1;
  }
}

void *ThreadClock( void * arg  )
{
  printf("CLOCK INIT: cc thread\n");
  struct clock_thread_parameter * parameter = (struct clock_thread_parameter *)arg;
  printf("CLOCK INIT: cc thread param\n");
  printf("CLOCK INIT: control queue addr: %lx\n", (unsigned long)parameter->control_queue);
  *(char*)parameter->current_command = '\0';

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    printf("CLOCK: starting cc\n");

    usleep(parameter->period * 1000000);

    printf("CLOCK: queue_len %i\n", *(unsigned int*)parameter->control_queue_length);

    // Get commands out of queue
    pthread_mutex_lock( parameter->control_queue_lock );
    if (*(unsigned int*)parameter->control_queue_length > 0){
      printf("CLOCK: control queue addr: %lu\n", (unsigned long)parameter->control_queue);
      printf("CLOCK: in lock, curr_cmd: %c\n", *(char*)parameter->control_queue);
      *(char*)parameter->current_command = *(char*)parameter->control_queue;
      printf("CLOCK: in lock, new curr_cmd: %c\n", *(char*)parameter->current_command);

      *(unsigned int*)parameter->control_queue_length -= 1;
      printf("CLOCK: in lock, new queue_len: %i\n", *(unsigned int*)parameter->control_queue_length);
      memmove(parameter->control_queue, parameter->control_queue + 1, *(unsigned int*)parameter->control_queue_length);

      // Pause motor queues so they update commands
      pthread_mutex_lock( &(parameter->motor_pause_left->lock) );
      parameter->motor_pause_left->pause = true;
      pthread_mutex_unlock( &(parameter->motor_pause_left->lock) );
      pthread_mutex_lock( &(parameter->motor_pause_right->lock) );
      parameter->motor_pause_right->pause = true;
      pthread_mutex_unlock( &(parameter->motor_pause_right->lock) );

      printf("\nCLOCK: exit update");
    }
    pthread_mutex_unlock( parameter->control_queue_lock );
  }
}

void *ThreadMotor( void * arg  )
{
  struct motor_thread_parameter * parameter = (struct motor_thread_parameter *)arg;
  int PWM = 0;
  bool I1 = 0, I2 = 0;
  char current_command = '\0';

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    // Execute params
    parameter->pwm->DAT1 = PWM;
    parameter->pwm->DAT2 = PWM_RANGE - PWM;
    
    if (I1){
      GPIO_SET( parameter->gpio, parameter->I1_pin );
    } else {
      GPIO_CLR( parameter->gpio, parameter->I1_pin );
    }

    if (I2){
      GPIO_SET( parameter->gpio, parameter->I2_pin );
    } else {
      GPIO_CLR( parameter->gpio, parameter->I2_pin );
    }
    

    pthread_mutex_lock( &(parameter->pause->lock) );
    if (parameter->pause->pause)
    {
      // Update params
      switch (*(char*)parameter->current_command)
      {
        case 's':
          printf("\n!!!! MOTOR: Recieved Command: STOP !!!!\n");
          break;
        case 'w':
          printf("\n!!!! MOTOR: Recieved Command: FORWARD !!!!\n");
          I1 = 1;
          I2 = 0;
          break;
        case 'x':
          printf("\nMOTOR: Recieved Command: BACKWARD\n");
          break;
        case 'i':
          printf("\nMOTOR: Recieved Command: FASTER\n");
          PWM += 5;
          break;
        case 'j':
          printf("\nMOTOR: Recieved Command: SLOWER\n");
          PWM -= 5;
          break;
        case 'a':
          printf("\nMOTOR: Recieved Command: LEFT\n");
          break;
        case 'd':
          printf("\nMOTOR: Recieved Command: RIGHT\n");
          break;
        default:
          break;
      }
      parameter->pause->pause = false;
    }
    pthread_mutex_unlock( &(parameter->pause->lock) );
  }
}


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
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 's';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'w':
        printf("KEY_THREAD: Recieved Command: FORWARD\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'w';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: FORWARD\nKEY_THREAD: Queue Lengh: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
      case 'x':
        printf("KEY_THREAD: Recieved Command: BACKWARD\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'x';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
        break;
      case 'i':
        printf("KEY_THREAD: Recieved Command: FASTER\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'i';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
        break;
      case 'j':
        printf("KEY_THREAD: Recieved Command: SLOWER\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'j';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
        break;
      case 'a':
        printf("KEY_THREAD: Recieved Command: LEFT\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'a';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
        break;
      case 'd':
        printf("KEY_THREAD: Recieved Command: RIGHT\n");

        // Lock Control Thread
        pthread_mutex_lock( thread_key_parameter->control_queue_lock );
        thread_key_parameter->pause_control->pause = !(thread_key_parameter->pause_control->pause);
        if (*(int*)thread_key_parameter->control_queue_length < QUEUE_SIZE) {
          *(char*)(thread_key_parameter->control_queue + *(int*)thread_key_parameter->control_queue_length) = 'd';
          *(unsigned int*)thread_key_parameter->control_queue_length += 1;
          printf("KEY_THREAD: Added to Queue: STOP\nKEY_THREAD: Queue Length: %i\n", *(unsigned int*)thread_key_parameter->control_queue_length);
        }
        pthread_mutex_unlock( thread_key_parameter->control_queue_lock );
        break;
        break;
      default:
        break;
    }
  } while (!done);
  printf( "key thread exiting\n" );

  return (void *)0;
}

int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread_key_handle;
  struct key_thread_parameter     thread_key_parameter;

  pthread_t                       thread_control_handle;
  pthread_t                       thread_left_motor_handle;
  pthread_t                       thread_right_motor_handle;
  pthread_t                       thread_clock_handle;
  struct control_thread_parameter thread_control_parameter;
  struct motor_thread_parameter   thread_left_motor_parameter;
  struct motor_thread_parameter   thread_right_motor_parameter;
  struct clock_thread_parameter   thread_clock_parameter;

  struct pause_flag               pause_left_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause_right_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               set_motors = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause_control = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               pause_clock = {PTHREAD_MUTEX_INITIALIZER, false};
  struct done_flag                done   = {PTHREAD_MUTEX_INITIALIZER, false};

  pthread_mutex_t                 queue_lock = PTHREAD_MUTEX_INITIALIZER;

  char *queue;
  queue = calloc(QUEUE_SIZE, sizeof(char));
  printf("MAIN: queue addr: %lx", (unsigned long)queue);
  
  printf( "what\n" );

  unsigned int *queue_len;
  queue_len = calloc(1, sizeof(unsigned int));
  printf("MAIN: queue_len addr: %lx", (unsigned long)queue_len);

  char *curr_cmd;
  curr_cmd = calloc(1, sizeof(char));
  printf("MAIN: queue_len addr: %lx", (unsigned long)curr_cmd);

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );

    enable_pwm_clock( io );

    printf( "enable pwm\n" );


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

    // // CONTROL
    // thread_control_parameter.pause = &pause_control;
    // thread_control_parameter.done = &done;
    // thread_control_parameter.queue_length = 0;
    // memset(&thread_control_parameter.queue, 0, QUEUE_SIZE);

    printf( "config pwm\n" );
    
    // CLOCK
    thread_clock_parameter.period = 1;
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
    thread_key_parameter.pause_control = &pause_control;
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
    thread_left_motor_parameter.gpio = &(io->gpio);
    thread_left_motor_parameter.pwm = &(io->pwm);
    thread_left_motor_parameter.current_command = thread_clock_parameter.current_command;
    thread_left_motor_parameter.left_motor = true;
    thread_left_motor_parameter.current_command = curr_cmd;

    // RIGHT
    thread_right_motor_parameter.pause = &pause_right_motor;
    thread_right_motor_parameter.done = &done;
    thread_right_motor_parameter.PWM_pin = 13;
    thread_right_motor_parameter.I1_pin = 22;
    thread_right_motor_parameter.I2_pin = 23;
    thread_right_motor_parameter.gpio = &(io->gpio);
    thread_right_motor_parameter.pwm = &(io->pwm);
    thread_right_motor_parameter.current_command = thread_clock_parameter.current_command;
    thread_right_motor_parameter.left_motor = false;
    thread_right_motor_parameter.current_command = curr_cmd;


    printf( "thread param\n" );


    // THREADS
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    // pthread_create( &thread_control_handle, 0, ThreadControl, (void *)&thread_control_parameter );
    pthread_create( &thread_left_motor_handle, 0, ThreadMotor, (void *)&thread_left_motor_parameter );
    pthread_create( &thread_right_motor_handle, 0, ThreadMotor, (void *)&thread_right_motor_parameter );
    pthread_create( &thread_clock_handle, 0, ThreadClock, (void *)&thread_clock_parameter);
    printf( "thread create\n" );
    pthread_join( thread_key_handle, 0 );
    pthread_join( thread_control_handle, 0 );
    pthread_join( thread_left_motor_handle, 0 );
    pthread_join( thread_right_motor_handle, 0 );
    pthread_join( thread_clock_handle, 0 );
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

// b hw5kelly.c:424
