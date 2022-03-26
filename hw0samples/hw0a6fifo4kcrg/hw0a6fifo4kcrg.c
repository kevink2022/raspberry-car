/**************************************************
* CMPEN 473, Spring 2022, Penn State University
*
* Sample Program hw0a6fifo3kcrg   ***  ***
*   - Multi-tasking with pthread, four thread example
*   - FIFO queue data passing example,
*   -   keyboard input thread to Control thread and
*   -   Control thread to red and green LED ON/OFF thread
*   - Mutex critical section locking example,
*   -   when accessing common data (multiple FIFO queue)
*   - Real-Time Interrupt-Driven embedded programming example, every 10ms
*   - Key board reading program example, without enter key, non-blocking
*   -
*   - Create four functions: KeyRead, Control, and two LED driving threads
*   -   KeyRead: scan key press every 10ms, 100 times/sec
*   -            pressed key value passed to the Control thread
*   -            through the key input FIFO queues,
*   -            key input preprocessing and filterings are possible.
*   -   Control: Pick up incoming key presses every 10ms, 100 times/sec,
*   -            from the key input FIFO queues,
*   -            generate LED on/off commands and fill the 
*   -            Red LED FIFO queue and Green LED FIFO queue,
*   -            for the LED on/off commands,
*   -            simple and short LED on/off programming is possible
*   -   LEDthread: controls Red and Green LEDs to be ON/OFF based on 
*   -            the key press.  Pressed key values and commands are 
*   -            received through the FIFO queues 
*   -            (Red FIFO queue and Green FIFO queue).
*   -            Queued key value checked every 10ms, 100 times/sec
*   -            'n' to turn-on, 'f' to turn-off the LED light, also
*   -            Delay command for longer operations
*   - Ending program by 'q' key enter
*
* Revision V1.1   On 2/21/2022
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
#include "FIFO.h"


#define FIFO_LENGTH     1024
#define THREE_QUARTERS  (FIFO_LENGTH*3/4)

/* key fifo queue entries are key presses, called 'command':
 *    'r' to turn-on, 't' to turn-off the Red   LED light,
 *    'g' to turn-on, 'h' to turn-off the Green LED light,
 *    'q' for quit the program
 * 
 *  but use 'n' to turn-on, 'f' to turn-off the LED lights
 *  by the common LED thread used for both Red and Green lights.
 *  Moreover, the Delay command can be issued to the LED on/off
 *  driver threads.
 * 
 * As an example, following are implemented in addition:
 *    'y' to blink the Red LED for 4 seconds, and
 *    'j' to blink the Green LED for 4 seconds, 
 *    if any key hit happens multiples times, 
 *    all commands are queued in the FIFO queue for
 *    sequential and orderly execution.                      */

struct thread_command
{
    uint8_t command;
    uint8_t argument;
};

FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);

struct LED_thread_param
{
  const char *                    name;
  volatile struct gpio_register * gpio;
  int                             pin_number;
  struct fifo_t *                 fifo;
  bool *                          quit_flag;
};

struct key_thread_param
{
  const char *                    name;
  struct fifo_t *                 key_fifo;
  bool *                          quit_flag;
};

struct control_thread_param
{
  const char *                    name;
  struct fifo_t *                 key_fifo;
  struct fifo_t *                 red_fifo;
  struct fifo_t *                 green_fifo;
  bool *                          quit_flag;
};



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

  if (ch != -1) {                    // '0' <= valid key range <= 'z'
    if (ch > 47) {                   // ignore all other keys
      if (ch < 123) {
        printf( "\n ch= %d\n", ch);}
      else {ch = -1;}
    }
    else {ch = -1;}
  }

  return ch;
  // this is keyboard reading without 'Enter' key and in
  // non-blocking mode.  This function returns single key
  // value (ASCII code), and returns -1 if no key was hit
}



void *KeyRead(void * arg)
{
  struct key_thread_param * param = (struct key_thread_param *)arg;
  struct thread_command cmd = {0, 0};
  int      keyhit = 0;
  bool     done   = false;
  struct   timespec    timer_state; /* used to wake up every 10ms
           with wait_period() function, similar to interrupt
           occuring every 10ms         */

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag))
  {
    keyhit = get_pressed_key();  // read once every 10ms
    if ( keyhit != -1)
    {
      switch (keyhit)
      {
        case 10:      // 'Enter' key hit => no action
        {
                      // nothing placed on param->key_fifo
        }
        break;

        case 113:  // 'q' for quit
        {
          cmd.command = 113;
          cmd.argument = 0;
          if (!(FIFO_FULL( param->key_fifo )))
          {FIFO_INSERT( param->key_fifo, cmd );}
          else {printf( "key_fifo queue full\n" );}
          *param->quit_flag = true;
        }
        break;

        default:
        {
          cmd.command  = keyhit;  // other key hit
          cmd.argument = 0;
          if (!(FIFO_FULL( param->key_fifo )))
          {FIFO_INSERT( param->key_fifo, cmd );}
          else {printf( "key_fifo queue full\n" );}
        }

      }
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "KeyRead function done\n" );

  return NULL;

}



void *Control(void * arg)
{
  struct control_thread_param * param = (struct control_thread_param *)arg;
  struct thread_command cmd1 = {0, 0};
  struct thread_command cmd2 = {0, 0};
  bool     done   = false;
  struct   timespec    timer_state; /* used to wake up every 10ms
           with wait_period() function, similar to interrupt
           occuring every 10ms         */
  int      ct    = 0;

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

/* Pick up incoming key presses every 10ms, 100 times/sec,
*  from the key input FIFO queues, and generate LED on/off commands, 
*  and fill the Red LED command FIFO queue and Green LED command FIFO queue,
*  for the LED on/off control.  
*  A simple and short LED on/off control programming is possible   */

  while (!*(param->quit_flag))
  {
    if (!(FIFO_EMPTY( param->key_fifo )))
    {
      FIFO_REMOVE( param->key_fifo, &cmd1 );  // read once every 10ms
      printf( " %s= %d  %c\n", param->name, cmd1.command, cmd1.command);
      switch (cmd1.command)
      {
        case 114:     // 'r' for Red LED on => 'n' for turn-ON
        {
          cmd2.command = 110;  // 'n' for turn-ON
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->red_fifo )))
          {FIFO_INSERT( param->red_fifo, cmd2 );}
          else {printf( "red fifo queue full\n" );}
        }
        break;

        case 116:     // 't' for Red LED off => 'f' for turn-OFF
        {
          cmd2.command = 102;  // 'f' for turn-OFF
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->red_fifo )))
          {FIFO_INSERT( param->red_fifo, cmd2 );}
          else {printf( "red fifo queue full\n" );}
        }
        break;

        case 103:     // 'g' for Green LED on => 'n' for turn-ON
        {
          cmd2.command = 110;  // 'n' for turn-ON
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->green_fifo)))
          {FIFO_INSERT( param->green_fifo, cmd2 );}
          else {printf( "green fifo queue full\n" );}
        }
        break;

        case 104:     // 'h' for Green LED off => 'f' for turn-OFF
        {
          cmd2.command = 102;  // 'f' for turn-OFF
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->green_fifo)))
          {FIFO_INSERT( param->green_fifo, cmd2 );}
          else {printf( "green fifo queue full\n" );}
        }
        break;

        case 113:  // 'q' for quit both red and green LED threads 
        {
          cmd2.command = 113;
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->red_fifo )) && !(FIFO_FULL( param->green_fifo)))
          {FIFO_INSERT( param->red_fifo, cmd2 );FIFO_INSERT( param->green_fifo, cmd2 );}
          else {printf( "fifo queue full\n" );}
          done = true;
        }
        break;

        case 121:  // 'y' for blink Red LED for 4 second, queue up the
        {          // blinking commands (queue filled with 80 total commands)
          ct = 20;           // repeat 20 times, ie. 4 sec. blinking
          while (ct != 0)    // ON for 100ms, OFF for 100ms => 200ms per cycle
          {
            cmd2.command = 110;  // 'n' for turn-ON
            cmd2.argument = 0;
            if (!(FIFO_FULL( param->red_fifo )))
            {FIFO_INSERT( param->red_fifo, cmd2 );}
            else {printf( "red fifo queue full\n" );}
          
            cmd2.command = 0;  // delay for 10 steps => 100ms
            cmd2.argument = 10;
            if (!(FIFO_FULL( param->red_fifo )))
            {FIFO_INSERT( param->red_fifo, cmd2 );}
            else {printf( "red fifo queue full\n" );}
          
            cmd2.command = 102;  // 'f' for turn-OFF
            cmd2.argument = 0;
            if (!(FIFO_FULL( param->red_fifo )))
            {FIFO_INSERT( param->red_fifo, cmd2 );}
            else {printf( "red fifo queue full\n" );}
          
            cmd2.command = 0;  // delay for 10 steps => 100ms
            cmd2.argument = 10;
            if (!(FIFO_FULL( param->red_fifo )))
            {FIFO_INSERT( param->red_fifo, cmd2 );}
            else {printf( "red fifo queue full\n" );}
            
            --ct;
          }
        }
        break;

        case 106:  // 'j' for blink Green LED for 4 second, queue up the
        {          // blinking commands (queue filled with 80 total commands)
          ct = 20;           // repeat 20 times, 4 sec. blinking
          while (ct != 0)    // ON for 100ms, OFF for 100ms => 200ms per cycle
          {
            cmd2.command = 110;  // 'n' for turn-ON
            cmd2.argument = 0;
            if (!(FIFO_FULL( param->green_fifo )))
            {FIFO_INSERT( param->green_fifo, cmd2 );}
            else {printf( "green fifo queue full\n" );}
          
            cmd2.command = 0;  // delay for 10 steps => 100ms
            cmd2.argument = 10;
            if (!(FIFO_FULL( param->green_fifo )))
            {FIFO_INSERT( param->green_fifo, cmd2 );}
            else {printf( "green fifo queue full\n" );}
          
            cmd2.command = 102;  // 'f' for turn-OFF
            cmd2.argument = 0;
            if (!(FIFO_FULL( param->green_fifo )))
            {FIFO_INSERT( param->green_fifo, cmd2 );}
            else {printf( "green fifo queue full\n" );}
          
            cmd2.command = 0;  // delay for 10 steps => 100ms
            cmd2.argument = 10;
            if (!(FIFO_FULL( param->green_fifo )))
            {FIFO_INSERT( param->green_fifo, cmd2 );}
            else {printf( "green fifo queue full\n" );}
            
            --ct;
          }          
        }
        break;

        default:
        {
          cmd2.command = cmd1.command;  // other key hit
          cmd2.argument = 0;
          if (!(FIFO_FULL( param->red_fifo )) && !(FIFO_FULL( param->green_fifo)))
          {FIFO_INSERT( param->red_fifo, cmd2 );FIFO_INSERT( param->green_fifo, cmd2 );}
          else {printf( "fifo queue full\n" );}
        }

      }
    }

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "Control function done\n" );

  return NULL;

}



void *LEDThread(void * arg)
{
  struct LED_thread_param * param = (struct LED_thread_param *)arg;
  struct thread_command cmd = {0, 0};
  bool     busy = false;
  bool     done = false;
  struct   timespec    timer_state; /* used to wake up every 10ms
           with wait_period() function,
           similar to interrupt occuring every 10ms  */

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag))
  {
    if (!busy)
    {
      if (!(FIFO_EMPTY( param->fifo )))
      {
        FIFO_REMOVE( param->fifo, &cmd );  // read once every 10ms
        printf( " %s= %d  %c\n", param->name, cmd.command, cmd.command);
        switch (cmd.command)
        {
          case 110:  // 'n' for turn-ON
          {
            GPIO_SET( param->gpio, param->pin_number);
            printf( " %s LED On\n", param->name);
          }
          break;

          case 102:  // 'f' for turn-OFF
          {
            GPIO_CLR( param->gpio, param->pin_number);
            printf( " %s LED Off\n", param->name);
          }
          break;

          case 113:  // 'q' for quit the program
          {
            done = true;
            printf( " %s quit\n", param->name);
          }
          break;

          case 0:   // Delay time command, 
          {
            if (cmd.argument != 0)
            {busy = true;}
            else
            { /* Delay = 0 => no operation needed */ }
          }
          break;

          default:
          {
            printf( " %s wrong cmd\n", param->name);
          }

        }
      }
    }
    else
    {
      if (cmd.argument != 0)
      {cmd.argument = cmd.argument - 1;}
      else
      {busy = false;}
    }
    

    wait_period( &timer_state, 10u ); /* 10ms */

  }

  printf( "%s function done\n", param->name );

  return NULL;

}



int main( void )
{
  struct io_peripherals *io;

  pthread_t tr;
  pthread_t tg;
  pthread_t tk;
  pthread_t tc;
  struct fifo_t key_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t red_fifo   = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t green_fifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  bool quit_flag = false;
  struct LED_thread_param       red_param   = {"Rth", NULL, 12,  &red_fifo, &quit_flag};
  struct LED_thread_param       green_param = {"Gth", NULL, 13,  &green_fifo, &quit_flag};
  struct key_thread_param       key_param   = {"key", &key_fifo, &quit_flag};
  struct control_thread_param   con_param   = {"con", &key_fifo, &red_fifo, &green_fifo, &quit_flag};

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    /* set the pin function to OUTPUT for GPIO12 - red   LED light   */
    /* set the pin function to OUTPUT for GPIO13 - green LED light   */
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_OUTPUT;

    red_param.gpio   = io->gpio;
    green_param.gpio = io->gpio;

    //printf("\n Program will end after 250 Red LED blinking. \n");

    /* set initial output state - off */
    GPIO_CLR(io->gpio, 12);
    GPIO_CLR(io->gpio, 13);


    // Create three threads tr, tg, tk, and tc, and run them in parallel
    pthread_create(&tr, NULL, LEDThread, (void *)&red_param);
    pthread_create(&tg, NULL, LEDThread, (void *)&green_param);
    pthread_create(&tk, NULL, KeyRead,   (void *)&key_param);
    pthread_create(&tc, NULL, Control,   (void *)&con_param);

    // Wait to finish tr, tg, and tk threads
    pthread_join(tr, NULL);
    pthread_join(tg, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);


    /* main task finished  */

    /* clean the GPIO pins */
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;

  }
  else
  {
    ; /* warning message already issued */
  }

  printf( "main function done\n" );

  return 0;
}
