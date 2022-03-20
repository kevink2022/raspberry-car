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
#include "hw6refined.h"

#define QUEUE_SIZE 100
#define STOP 0
#define FORWARD 2
#define BACKWARD 1

#define PWM_RANGE 100
#define PWM_MOTOR_MAX 100 
#define PWM_MOTOR_MIN 20
#define PWM_SPEED_STEP 5
#define PWM_TURN_STEP 15
#define PWM_ORIENTATION 1
#define PWM_MODE2_STEP 20
#define PWM_MODE2_TURN_DELAY 60000
#define PWM_MODE2_OFF_DELAY 500

//#define DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ThreadClock()
//
//  Sends periodic signals to time dependant threads. Kept seperate from other thread to keep
//  time consistent.
void *ThreadClock( void * arg  )
{
  #ifdef DEBUG
  printf("CLOCK: thread started\n"); 
  #endif
  struct clock_thread_parameter * parameter = (struct clock_thread_parameter *)arg;

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    #ifdef DEBUG
    printf("CLOCK: starting cc\n");
    #endif

    usleep(parameter->period * 1000000);

    #ifdef DEBUG
    printf("CLOCK: posting sem\n");
    #endif

    // Wake the control thread to process the queue
    sem_post(parameter->control_thread_sem);

    // Wake the data aquisition thread 
    // for hw7
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ThreadControl()
//
//  Updates the current command and the queue
void *ThreadControl( void * arg  )
{
  struct control_thread_parameter * parameter = (struct control_thread_parameter *)arg;
  *(char*)parameter->current_command = 's';

  #ifdef DEBUG
  printf("CONTROL: sem init\n");
  #endif

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))  // Check done
  {
    pthread_mutex_unlock( &(parameter->done->lock) );
  
    // Wait for signal from clock thread
    sem_wait(parameter->control_thread_sem);

    // Check for commands in the queue
    pthread_mutex_lock( parameter->control_queue->control_queue_lock );
    if (*(unsigned int*)parameter->control_queue->control_queue_length > 0){
      #ifdef DEBUG
      printf("CONTROL: in lock, queue_addr: %lx\n", (unsigned long)parameter->control_queue->control_queue);
      printf("CONTROL: in lock, queue len: %i\n", *(char*)parameter->control_queue->control_queue_length);
      printf("CONTROL: in lock, curr_cmd: %c\n", *(char*)parameter->current_command);
      printf("CONTROL: in lock, queue casted: %c\n", *(char*)parameter->control_queue->control_queue);
      #endif

      // Set the current command to the first command in the queue
      pthread_mutex_lock( (parameter->current_command_lock) );
      *(char*)parameter->current_command = *(char*)parameter->control_queue->control_queue;
      pthread_mutex_unlock( (parameter->current_command_lock) );
      
      #ifdef DEBUG
      printf("CONTROL: in lock, new curr_cmd: %c\n", *(char*)parameter->current_command);
      #endif

      // Decrement the control queue length
      *(unsigned int*)parameter->control_queue->control_queue_length -= 1;
      
      #ifdef DEBUG
      printf("CONTROL: in lock, new queue_len: %i\n", *(unsigned int*)parameter->control_queue->control_queue_length);
      #endif

      // Move queue commands up
      memmove(parameter->control_queue->control_queue, parameter->control_queue->control_queue + 1, *(unsigned int*)parameter->control_queue->control_queue_length);

      // Pause motor queue so it updates command
      pthread_mutex_lock( &(parameter->motor_pause->lock) );
      parameter->motor_pause->pause = true;
      pthread_mutex_unlock( &(parameter->motor_pause->lock) );

      #ifdef DEBUG
      printf("CONTROL: exit update\n");
      #endif
    }
    pthread_mutex_unlock( parameter->control_queue->control_queue_lock );
  
  }
}

// A == LEFT  == DAT2
// B == RIGHT == DAT1

#define DEBUG
#undef DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ThreadMotor()
//
//  Controls the motor, based off different modes
void *ThreadMotor( void * arg  )
{
  struct motor_thread_parameter * parameter = (struct motor_thread_parameter *)arg;
  char current_command = 's';
  int mode = MODE_1;
  bool off_course = false; // Used to exit thread if off course and stuck turing in mode 2

  motor_pin_values motor_pin_values;
  init_motor_pin_values(&motor_pin_values);

  #ifdef DEBUG
  printf("MOTOR: init\n");
  #endif

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    //sleep(1);

    // Update PWM
    if ((motor_pin_values.A_PWM != motor_pin_values.A_PWM_next) || (motor_pin_values.B_PWM != motor_pin_values.B_PWM_next)){
      update_motor_pwm(parameter->motor_pins, &motor_pin_values);
    }

    #ifdef DEBUG
    printf("MOTOR: Updated PWM\n");
    #endif

    #define DEBUG

    // If going forward in mode 2, check IR sensors
    if ( (motor_pin_values.AI1 && motor_pin_values.BI1) && (mode == MODE_2)) {

      // Check IR sensor A
      if(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->AIR_pin) != 0 && !off_course){
        int i = 0;
        #ifdef DEBUG
        printf("MOTOR: Auto turn A \n");
        #endif

        while(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->AIR_pin) != 0) {
          parameter->motor_pins->pwm->DAT2 = PWM_MOTOR_MAX;
          GPIO_CLR( parameter->motor_pins->gpio, parameter->motor_pins->AI1_pin );
          GPIO_SET( parameter->motor_pins->gpio, parameter->motor_pins->AI2_pin );
          usleep(PWM_MODE2_TURN_DELAY); 
          if(i == PWM_MODE2_OFF_DELAY){
            off_course = true;
            #ifdef DEBUG
            printf("MOTOR: Auto A off course\n");
            #endif
            break; // If off course, will never break out of loop
          }
          i++;
        }
        #ifdef DEBUG
        printf("MOTOR: Auto A done\n");
        #endif

        set_motor_pins(parameter->motor_pins, &motor_pin_values);

        #ifdef DEBUG
        printf("MOTOR: Auto A set pins\n");
        #endif
      } 

      // Check IR sensor B
      else if(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->BIR_pin) != 0 && !off_course){
        int i = 0;
        #ifdef DEBUG
        printf("MOTOR: Auto turn B \n");
        #endif

        while(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->BIR_pin) != 0 && !off_course) {
          parameter->motor_pins->pwm->DAT1 = PWM_MOTOR_MAX;
          GPIO_CLR( parameter->motor_pins->gpio, parameter->motor_pins->BI1_pin );
          GPIO_SET( parameter->motor_pins->gpio, parameter->motor_pins->BI2_pin ); 
          usleep(PWM_MODE2_TURN_DELAY);
          if(i == PWM_MODE2_OFF_DELAY){
            off_course = true;
            #ifdef DEBUG
            printf("MOTOR: Auto A off course\n");
            #endif
            break; // If off course, will never break out of loop
          }
          i++;
        }

        #ifdef DEBUG
        printf("MOTOR: Auto B done\n");
        #endif

        set_motor_pins(parameter->motor_pins, &motor_pin_values);

        #ifdef DEBUG
        printf("MOTOR: Auto B set pins\n");
        #endif
      }
    }

    #undef DEBUG

    // Update A and B pins if needed
    if ((motor_pin_values.AI1 != motor_pin_values.AI1_next) || (motor_pin_values.AI2 != motor_pin_values.AI2_next) || (motor_pin_values.BI1 != motor_pin_values.BI1_next) || (motor_pin_values.BI2 != motor_pin_values.BI2_next)) {
      update_motor_pins(parameter->motor_pins, &motor_pin_values);
    }
    
    #ifdef DEBUG
    printf("MOTOR: updated pins\n");
    #endif

    pthread_mutex_lock( &(parameter->pause->lock) );
    if (parameter->pause->pause)
    {
      // Update params
      update_command(&motor_pin_values, *(char*)parameter->current_command, &mode);
      parameter->pause->pause = false;
    }
    pthread_mutex_unlock( &(parameter->pause->lock) );
  
    #ifdef DEBUG
    printf("MOTOR: updated commands\n");
    #endif
  }
}
#undef DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  get_pressed_key()
//
//  returns pressed key
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
#undef DEBUG


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ThreadKey()
//
//  Adds commands to the queue based off pressed keys
void *ThreadKey( void * arg )
{
  struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  bool done = false;
  char mode = '1';
  char input = 's';

  #ifdef DEBUG
  printf("KEY: init\n");
  #endif

  do
  {
    printf("\nHw6m%c> ", mode);
    input = get_pressed_key();

    #ifdef DEBUG
    printf("KEY: input: %c\n", input);
    #endif

    if(input == 'q'){
      printf(" q\n");
      done = true;
      /* indicate that it is time to shut down */
      pthread_mutex_lock( &(thread_key_parameter->done->lock) );
      thread_key_parameter->done->done = true;
      pthread_mutex_unlock( &(thread_key_parameter->done->lock) );
    } 
    else if (input == 'm'){
      input = get_pressed_key();
      printf(" %c\n", input);
      add_to_queue(thread_key_parameter->control_queue, input);
    } 
    else {
      printf(" %c\n", input);
      #ifdef DEBUG
      printf("KEY: sending: %c\n", input);
      sleep(1);
      #endif
      add_to_queue(thread_key_parameter->control_queue, input);
    }
  } while (!done);
  printf( "KEY_THREAD: Exiting...\n" );

  return (void *)0;
}
#undef DEBUG
#define DEBUG
#undef DEBUG
int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t                       thread_key_handle;
  struct key_thread_parameter     thread_key_parameter;

  pthread_t                       thread_motor_handle;
  pthread_t                       thread_clock_handle;
  pthread_t                       thread_control_handle;
  struct motor_thread_parameter   thread_motor_parameter;
  struct clock_thread_parameter   thread_clock_parameter;
  struct control_thread_parameter thread_control_parameter;

  struct pause_flag               pause_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               set_motors  = {PTHREAD_MUTEX_INITIALIZER, false};
  struct done_flag                done        = {PTHREAD_MUTEX_INITIALIZER, false};

  pthread_mutex_t                 queue_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t                 curr_cmd_lock = PTHREAD_MUTEX_INITIALIZER;
  sem_t                           control_thread_sem;
  sem_init(&control_thread_sem, 0, 1);

  #ifdef DEBUG
  printf("MAIN: created sem: %lx", (unsigned long)&control_thread_sem);
  #endif

  char *queue = calloc(QUEUE_SIZE, sizeof(char));
  #ifdef DEBUG
  printf("MAIN: queue addr: %lx", (unsigned long)queue);
  #endif

  unsigned int queue_len = 0;
  char curr_cmd = 's';

  control_queue control_queue;
  control_queue.control_queue = queue;
  control_queue.control_queue_length = &queue_len;
  control_queue.control_queue_lock = &queue_lock;

  #ifdef DEBUG
  printf("MAIN: created control queue: %lx", (unsigned long)&control_queue);
  #endif

  motor_pins motor_pins;
  motor_pins.A_PWM_pin = 12;
  motor_pins.AI1_pin = 5;
  motor_pins.AI2_pin = 6;
  motor_pins.AIR_pin = 24;
  motor_pins.B_PWM_pin = 13;
  motor_pins.BI1_pin = 22;
  motor_pins.BI2_pin = 23;
  motor_pins.BIR_pin = 25;

  #ifdef DEBUG
  printf("MAIN: created motor_pins: %lx", (unsigned long)&motor_pins);
  #endif

  
  #define DEBUG

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

    motor_pins.gpio = &(io->gpio);
    motor_pins.pwm = &(io->pwm);
    
    // CLOCK
    thread_clock_parameter.period = .01;
    thread_clock_parameter.done = &done;
    thread_clock_parameter.control_thread_sem = &control_thread_sem;

    // CONTROL
    thread_control_parameter.done = &done;
    thread_control_parameter.control_thread_sem = &control_thread_sem;
    thread_control_parameter.control_queue = &control_queue;
    thread_control_parameter.current_command = &curr_cmd;
    thread_control_parameter.current_command_lock = &curr_cmd_lock;
    thread_control_parameter.motor_pause = &pause_motor;

    // KEY
    thread_key_parameter.done = &done;
    thread_key_parameter.control_queue = &control_queue;

    // Single MOTOR Thread
    thread_motor_parameter.done = &done;
    thread_motor_parameter.pause = &pause_motor;
    thread_motor_parameter.current_command = &curr_cmd;
    thread_motor_parameter.current_command_lock = &curr_cmd_lock;
    thread_motor_parameter.motor_pins = &motor_pins;

    #ifdef DEBUG
    printf("MAIN: threads ready: \n");
    #endif


    // THREADS
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter );
    #ifdef DEBUG
    printf("MAIN: key created: \n");
    #endif
    pthread_create( &thread_motor_handle, 0, ThreadMotor, (void *)&thread_motor_parameter );
    #ifdef DEBUG
    printf("MAIN: motor created: \n");
    #endif
    pthread_create( &thread_clock_handle, 0, ThreadClock, (void *)&thread_clock_parameter);
    #ifdef DEBUG
    printf("MAIN: clock created: \n");
    #endif
    pthread_create( &thread_control_handle, 0, ThreadControl, (void *)&thread_control_parameter);

    #ifdef DEBUG
    printf("MAIN: threads created");
    #endif

    
    pthread_join( thread_key_handle, 0 );
    pthread_join( thread_motor_handle, 0 );
    pthread_join( thread_clock_handle, 0 );
    pthread_join( thread_control_handle, 0);
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

/////////////////////////// Motor thread functions ///////////////////////////
// Pass pins and pin values through structs which can then be modified

//#define DEBUG

void init_motor_pin_values(motor_pin_values *motor_pin_values){
  motor_pin_values->A_PWM = PWM_MOTOR_MIN;
  motor_pin_values->AI1 = 0;
  motor_pin_values->AI2 = 0;
  motor_pin_values->B_PWM = PWM_MOTOR_MIN;
  motor_pin_values->BI1 = 0;
  motor_pin_values->BI2 = 0;

  motor_pin_values->A_PWM_next = PWM_MOTOR_MIN;
  motor_pin_values->AI1_next = 0;
  motor_pin_values->AI2_next = 0;
  motor_pin_values->B_PWM_next = PWM_MOTOR_MIN;
  motor_pin_values->BI1_next = 0;
  motor_pin_values->BI2_next = 0;
}

void smooth_speed_change(motor_pins *motor_pins, motor_pin_values *motor_pin_values, int initial_speed, int final_speed, int step){
  
  #ifdef DEBUG  
  printf("\nMOTOR-SMOOTH-SPEED: init: %i, init: %i, step: %i,\n", motor_pin_values->A_PWM);
  #endif

  // Slowdown
  if (initial_speed > final_speed) {
    while(motor_pin_values->A_PWM > final_speed){
        motor_pin_values->A_PWM -= step;
        motor_pin_values->B_PWM -= step;
        motor_pins->pwm->DAT2 = motor_pin_values->A_PWM;
        motor_pins->pwm->DAT1 = motor_pin_values->B_PWM;
        usleep(10000); // 0.01s
    }
  } 
  // Speedup
  else {
    while(motor_pin_values->A_PWM < final_speed){
        motor_pin_values->A_PWM += step;
        motor_pin_values->B_PWM += step;
        motor_pins->pwm->DAT2 = motor_pin_values->A_PWM;
        motor_pins->pwm->DAT1 = motor_pin_values->B_PWM;
        usleep(10000); // 0.01s
    }
  }
}

void turn(void){

}

void set_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values){
  
  //#define DEBUG

  #ifdef DEBUG  
  printf("\nMOTOR-SET-PINS: A_PWM : %i\n", motor_pin_values->A_PWM);
  printf("MOTOR-SET-PINS: B_PWM : %i\n", motor_pin_values->B_PWM);
  printf("MOTOR-SET-PINS: A_ : %i\n", motor_pin_values->A_PWM);
  printf("MOTOR-SET-PINS: B_PWM : %i\n", motor_pin_values->B_PWM);
  #endif

  //#undef DEBUG
  
  
  // Set PWM to original values
  motor_pins->pwm->DAT2 = motor_pin_values->A_PWM;
  motor_pins->pwm->DAT1 = motor_pin_values->B_PWM;

  // Set motor control to original values
  if (motor_pin_values->AI1){
    GPIO_SET( motor_pins->gpio, motor_pins->AI1_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->AI1_pin );
  }

  if (motor_pin_values->AI2){
    GPIO_SET( motor_pins->gpio, motor_pins->AI2_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->AI2_pin );
  }      

  if (motor_pin_values->BI1){
    GPIO_SET( motor_pins->gpio, motor_pins->BI1_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->BI1_pin );
  }
  if (motor_pin_values->BI2){
    GPIO_SET( motor_pins->gpio, motor_pins->BI1_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->BI1_pin );
  }
}

#define DEBUG
#undef DEBUG

void update_motor_pwm(motor_pins *motor_pins, motor_pin_values *motor_pin_values) {

  motor_pin_values->A_PWM = motor_pin_values->A_PWM_next;
  motor_pin_values->B_PWM = motor_pin_values->B_PWM_next;

  #ifdef DEBUG  
  printf("\nMOTOR: Setting A_PWM: %i\n", motor_pin_values->A_PWM);
  printf("MOTOR: Setting B_PWM: %i\n", motor_pin_values->B_PWM);
  printf("\nMOTOR: PWM: %lx\n", (unsigned long)motor_pins->pwm);
  printf("\nMOTOR: DAT1: %lx\n", (unsigned long)motor_pins->pwm->DAT1);
  printf("MOTOR: DAT2: %lx\n", (unsigned long)motor_pins->pwm->DAT2);
  sleep(2);
  #endif


  // Set new values
  motor_pins->pwm->DAT2 = motor_pin_values->A_PWM;
  motor_pins->pwm->DAT1 = motor_pin_values->B_PWM;
}

//#undef DEBUG
//#define DEBUG

void update_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values) {
    
  

  motor_pin_values->AI1 = motor_pin_values->AI1_next;
  motor_pin_values->AI2 = motor_pin_values->AI2_next;
  motor_pin_values->BI1 = motor_pin_values->BI1_next;
  motor_pin_values->BI2 = motor_pin_values->BI2_next;

  #ifdef DEBUG  
  printf("\nMOTOR-UPDATE-PINS: A_PWM Pre Slow Stop: %i\n", motor_pin_values->A_PWM);
  printf("MOTOR-UPDATE-PINS: B_PWM Pre Slow Stop: %i\n", motor_pin_values->B_PWM);
  #endif

  #ifdef DEBUG
  printf("\nMOTOR-UPDATE-PINS: Setting AI1: %i\n", motor_pin_values->AI1);
  printf("MOTOR-UPDATE-PINS: Setting BI1: %i\n", motor_pin_values->AI2);
  printf("MOTOR-UPDATE-PINS: Setting AI1: %i\n", motor_pin_values->BI1);
  printf("MOTOR-UPDATE-PINS: Setting BI1: %i\n", motor_pin_values->BI2);
  #endif

  // Slow stop
  smooth_speed_change(motor_pins, motor_pin_values, motor_pin_values->A_PWM, PWM_MOTOR_MIN, PWM_SPEED_STEP);

  // Set motor control to original values
  if (motor_pin_values->AI1){
    GPIO_SET( motor_pins->gpio, motor_pins->AI1_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->AI1_pin );
  }

  if (motor_pin_values->AI2){
    GPIO_SET( motor_pins->gpio, motor_pins->AI2_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->AI2_pin );
  }      

  if (motor_pin_values->BI1){
    GPIO_SET( motor_pins->gpio, motor_pins->BI1_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->BI1_pin );
  }
  if (motor_pin_values->BI2){
    GPIO_SET( motor_pins->gpio, motor_pins->BI2_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->BI2_pin );
  }

  // Slow Start
  smooth_speed_change(motor_pins, motor_pin_values, PWM_MOTOR_MIN, motor_pin_values->A_PWM_next, PWM_SPEED_STEP);

}


//#define DEBUG


void update_command(motor_pin_values *motor_pin_values, char current_command, int *mode){
  // Update params
  switch (current_command)
  {
    case 's':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: STOP\n");
      #endif
      motor_pin_values->AI1_next = 0;
      motor_pin_values->AI2_next = 0;
      motor_pin_values->BI1_next = 0;
      motor_pin_values->BI2_next = 0;
      // Signal Data thread to stop recording
      break;

    case 'w':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: FORWARD\n");
      #endif
      motor_pin_values->AI1_next = 1;
      motor_pin_values->AI2_next = 0;
      motor_pin_values->BI1_next = 1;
      motor_pin_values->BI2_next = 0;
      // Signal Data thread to start recording
      break;

    case 'x':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: BACKWARD\n");
      #endif
      motor_pin_values->AI1_next = 0;
      motor_pin_values->AI2_next = 1;
      motor_pin_values->BI1_next = 0;
      motor_pin_values->BI2_next = 1;
      break;

    case 'i':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: FASTER\n");
      #endif
      if (motor_pin_values->A_PWM < PWM_MOTOR_MAX) {motor_pin_values->A_PWM_next += PWM_SPEED_STEP;}
      if (motor_pin_values->B_PWM < PWM_MOTOR_MAX) {motor_pin_values->B_PWM_next += PWM_SPEED_STEP;}
      #ifdef DEBUG
      printf("\nMOTOR: A_PWM = %i\n", motor_pin_values->A_PWM_next);
      printf("MOTOR: B_PWM = %i\n", motor_pin_values->B_PWM_next);
      #endif
      break;

    case 'j':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: SLOWER\n");
      #endif
      if (motor_pin_values->A_PWM > PWM_MOTOR_MIN) {motor_pin_values->A_PWM_next -= PWM_SPEED_STEP;}
      if (motor_pin_values->B_PWM > PWM_MOTOR_MIN) {motor_pin_values->B_PWM_next -= PWM_SPEED_STEP;}
      #ifdef DEBUG
      printf("\nMOTOR: A_PWM = %i\n", motor_pin_values->A_PWM_next);
      printf("MOTOR: B_PWM = %i\n", motor_pin_values->B_PWM_next);
      #endif
      break;

    case 'a':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: LEFT\n");
      #endif
      //if(parameter->left_motor){
        if(motor_pin_values->A_PWM > PWM_MOTOR_MIN){motor_pin_values->A_PWM_next -= PWM_TURN_STEP;}
      //} else {
        if (motor_pin_values->B_PWM < PWM_MOTOR_MAX) {motor_pin_values->B_PWM_next += PWM_TURN_STEP;}
      //}
      #ifdef DEBUG
      printf("\nMOTOR: A_PWM = %i\n", motor_pin_values->A_PWM_next);
      printf("MOTOR: B_PWM = %i\n", motor_pin_values->B_PWM_next);
      #endif
      break;

    case 'd':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: RIGHT\n");
      #endif
      //if(parameter->left_motor){
        if (motor_pin_values->A_PWM < PWM_MOTOR_MAX) {motor_pin_values->A_PWM_next += PWM_TURN_STEP;}
      //} else {
        if(motor_pin_values->B_PWM > PWM_MOTOR_MIN) {motor_pin_values->B_PWM_next -= PWM_TURN_STEP;}
      //}
      #ifdef DEBUG
      printf("\nMOTOR: A_PWM = %i\n", motor_pin_values->A_PWM_next);
      printf("MOTOR: B_PWM = %i\n", motor_pin_values->B_PWM_next);
      #endif
      break;

    case '0':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 1\n");
      #endif
      *mode = MODE_0;
      break;

    case '1':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 1\n");
      #endif
      *mode = MODE_1;
      break;

    case '2':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 2\n");
      #endif
      *mode = MODE_2;
      break;  

    default:
      break;
  }
}

//#define DEBUG

/////////////////////// KEY THREAD FUNCTIONS /////////////////////////////
void add_to_queue(control_queue *control_queue, char command){
  
  #ifdef DEBUG
  printf("ADD-TO-QUEUE: Recieved Command: %c\n", command);
  #endif

  pthread_mutex_lock( control_queue->control_queue_lock );
  
  if (*(int*)control_queue->control_queue_length < QUEUE_SIZE) {
    *(char*)(control_queue->control_queue + *(int*)control_queue->control_queue_length) = command;
    *(unsigned int*)control_queue->control_queue_length += 1;
  }
  pthread_mutex_unlock( control_queue->control_queue_lock );

  #ifdef DEBUG
  printf("ADD-TO-QUEUE: command added");
  #endif
}
