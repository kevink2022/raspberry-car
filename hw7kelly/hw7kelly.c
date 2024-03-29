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
#include "hw7kelly.h"

#define QUEUE_SIZE 100
#define STOP 0
#define FORWARD 2
#define BACKWARD 1
#define DATA_SPACE_SIZE 1000000

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
    printf("CLOCK: posting sems\n");
    #endif

    // Wake the control thread to process the queue
    sem_post(parameter->control_thread_sem);

    // Wake the data aquisition thread 
    sem_post(parameter->data_thread_sem);
  }
}
#undef DEBUG


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
    
    // Update PWM
    if ((motor_pin_values.A_PWM != motor_pin_values.A_PWM_next) || (motor_pin_values.B_PWM != motor_pin_values.B_PWM_next)){
      update_motor_pwm(parameter->motor_pins, &motor_pin_values);
    }

    #ifdef DEBUG
    printf("MOTOR: Updated PWM\n");
    #endif

    // If going forward in mode 2, check IR sensors
    if ( (motor_pin_values.AI1 && motor_pin_values.BI1) && (mode == MODE_2)) {

      // Check IR sensor A
      if(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->AIR_pin) != 0 && !off_course){
        int i = 0;
        #ifdef DEBUG
        printf("MOTOR: Auto turn A \n");
        #endif

        while(GPIO_READ(parameter->motor_pins->gpio, parameter->motor_pins->AIR_pin) != 0) {
          parameter->motor_pins->pwm->DAT1 = PWM_MOTOR_MAX;
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
          parameter->motor_pins->pwm->DAT2 = PWM_MOTOR_MAX;
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
      update_command(&motor_pin_values, *(char*)parameter->current_command, &mode, parameter->data_signal, parameter->data_samples, parameter->sample_count);
      parameter->pause->pause = false;
    }
    pthread_mutex_unlock( &(parameter->pause->lock) );
  
    #ifdef DEBUG
    printf("MOTOR: updated commands\n");
    #endif
  }
}
#undef DEBUG

//#define DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ThreadData()
//
//  Controls the motor, based off different modes
void *ThreadData( void * arg  )
{
  struct data_thread_parameter * parameter = (struct data_thread_parameter *)arg;
  int sample_count = 500;
  bool recording = true;

  #ifdef DEBUG
  printf("DATA: thread init\n");
  #endif

  initialize_accelerometer_and_gyroscope( parameter->calibration_accelerometer, parameter->calibration_gyroscope, parameter->bsc );

  #ifdef DEBUG
  printf("DATA: accel/gyro init\n");
  #endif

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    #ifdef DEBUG
    printf("DATA: loop\n");
    #endif
  
    // Wait for clock signal
    sem_wait(parameter->data_thread_sem);

    #ifdef DEBUG
    printf("DATA: sem_wait\n");
    #endif

    pthread_mutex_lock( &(parameter->data_signal->lock) );
    // Check if recording  
    if (parameter->data_signal->recording)
    {
      if(parameter->data_signal->m0) // if m0, count samples for 5 seconds (500 samples)
      {
        //pthread_mutex_unlock( &(parameter->data_signal->lock) );

        #ifdef DEBUG
        printf("DATA: m0 loop\n");
        #endif
       
        ////// SAMPLE ///////
        read_accelerometer_gyroscope( parameter->calibration_accelerometer, parameter->calibration_gyroscope, parameter->bsc, parameter->data_samples, parameter->sample_count );
        ////// SAMPLE ///////

        #ifdef DEBUG
        printf("DATA: m0 sample\n");
        #endif

        sample_count--;
        if(sample_count == 0){
          sample_count = 500;
          //pthread_mutex_lock( &(parameter->data_signal->lock) );
          parameter->data_signal->recording = false;
          parameter->data_signal->m0 = false;
          //pthread_mutex_unlock( &(parameter->data_signal->lock) );

          write_to_file(0, parameter->data_samples, parameter->sample_count);
          //average_sample(parameter->data_samples,  parameter->sample_count);
          
          *(parameter->sample_count) = 0;
          printf("MODE 0: Sampling complete");
          printf("\n");
        }
      } 
      else 
      {
        ////// SAMPLE ///////
        read_accelerometer_gyroscope( parameter->calibration_accelerometer, parameter->calibration_gyroscope, parameter->bsc, parameter->data_samples, parameter->sample_count );
        ////// SAMPLE ///////
      }
    }
    pthread_mutex_unlock( &(parameter->data_signal->lock) );
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
      printf(" m");
      input = get_pressed_key();
      mode = input;
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
  struct io_peripherals * io;
  struct calibration_data calibration_accelerometer;
  struct calibration_data calibration_gyroscope;
  struct calibration_data calibration_magnetometer;

  pthread_t                       thread_key_handle;
  struct key_thread_parameter     thread_key_parameter;

  pthread_t                       thread_motor_handle;
  pthread_t                       thread_clock_handle;
  pthread_t                       thread_control_handle;
  pthread_t                       thread_data_handle;

  struct motor_thread_parameter   thread_motor_parameter;
  struct clock_thread_parameter   thread_clock_parameter;
  struct control_thread_parameter thread_control_parameter;
  struct data_thread_parameter    thread_data_parameter;

  struct pause_flag               pause_motor = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag               set_motors  = {PTHREAD_MUTEX_INITIALIZER, false};
  struct done_flag                done        = {PTHREAD_MUTEX_INITIALIZER, false};
  struct data_signal              data_signal = {PTHREAD_MUTEX_INITIALIZER, false, false};


  pthread_mutex_t                 queue_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t                 curr_cmd_lock = PTHREAD_MUTEX_INITIALIZER;
  sem_t                           control_thread_sem;
  sem_t                           data_thread_sem;
  
  sem_init(&control_thread_sem, 0, 1);
  sem_init(&data_thread_sem, 0, 1);

  #ifdef DEBUG
  printf("MAIN: created sem: %lx", (unsigned long)&control_thread_sem);
  #endif

  char *queue = calloc(QUEUE_SIZE, sizeof(char));
  #ifdef DEBUG
  printf("MAIN: queue addr: %lx", (unsigned long)queue);
  #endif

  data_sample *data_space = calloc(DATA_SPACE_SIZE, sizeof(data_sample));
  #ifdef DEBUG
  printf("MAIN: queue addr: %lx", (unsigned long)queue);
  #endif
  int sample_count = 0;

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
  #undef DEBUG
  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );
    // printf( "mem at 0x%8.8X\n", (unsigned long)&(io->cm) );
    // printf( "mem at 0x%8.8X\n", (unsigned long)&(io->pwm));

    enable_pwm_clock( (volatile struct cm_register *)&(io->cm), (volatile struct pwm_register *)&(io->pwm) );

    #ifdef DEBUG
    printf("MAIN: enable pwm: \n");
    printf( "mem at 0x%8.8X\n", (unsigned long)(io->gpio));
    #endif
    

    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_INPUT;

    #ifdef DEBUG
    printf("MAIN: gpio cleared: \n");
    #endif
    
    /* set the pin function to alternate function 0 for GPIO12 */
    /* set the pin function to alternate function 0 for GPIO13 */
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

    #ifdef DEBUG
    printf("MAIN: gpio config: \n");
    #endif

    /* configure the PWM channels */
    io->pwm->RNG1 = PWM_RANGE;     /* the default value */
    io->pwm->RNG2 = PWM_RANGE;     /* the default value */
    io->pwm->CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm->CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm->CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm->CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm->CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.MSEN1 = 1;  /* use M/S algorithm */
    io->pwm->CTL.field.MSEN2 = 1;  /* use M/S algorithm */
    io->pwm->CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm->CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm->CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    #ifdef DEBUG
    printf("MAIN: pwm config: \n");
    #endif
    
    /* configure the I2C interface */
    io->bsc->DIV.field.CDIV  = (PERIPHERAL_CLOCK*10)/400000;
    io->bsc->DEL.field.REDL  = 0x30;
    io->bsc->DEL.field.FEDL  = 0x30;
    io->bsc->CLKT.field.TOUT = 0x40;
    io->bsc->C.field.INTD    = 0;
    io->bsc->C.field.INTT    = 0;
    io->bsc->C.field.INTR    = 0;
    io->bsc->C.field.I2CEN   = 1;
    io->bsc->C.field.CLEAR   = 1;

    #ifdef DEBUG
    printf("MAIN: i2c config: \n");
    #endif


    motor_pins.gpio = (io->gpio);
    motor_pins.pwm = (io->pwm);
    
    // CLOCK
    thread_clock_parameter.period = .01;
    thread_clock_parameter.done = &done;
    thread_clock_parameter.control_thread_sem = &control_thread_sem;
    thread_clock_parameter.data_thread_sem = &data_thread_sem;

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
    thread_motor_parameter.data_signal = &data_signal;
    thread_motor_parameter.data_samples = data_space;
    thread_motor_parameter.sample_count = &sample_count;
    thread_motor_parameter.current_command = &curr_cmd;
    thread_motor_parameter.current_command_lock = &curr_cmd_lock;
    thread_motor_parameter.motor_pins = &motor_pins;

    // DATA
    thread_data_parameter.done = &done;
    thread_data_parameter.data_signal = &data_signal;
    thread_data_parameter.data_thread_sem = &data_thread_sem;
    thread_data_parameter.data_samples = data_space;
    thread_data_parameter.sample_count = &sample_count;
    thread_data_parameter.calibration_accelerometer = &calibration_accelerometer;
    thread_data_parameter.calibration_gyroscope = &calibration_gyroscope;
    thread_data_parameter.calibration_magnetometer = &calibration_magnetometer;
    thread_data_parameter.bsc = (io->bsc);

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

    pthread_create( &thread_data_handle, 0, ThreadData, (void *)&thread_data_parameter);

    #ifdef DEBUG
    printf("MAIN: threads created\n");
    #endif

    
    pthread_join( thread_key_handle, 0 );
    pthread_join( thread_motor_handle, 0 );
    pthread_join( thread_clock_handle, 0 );
    pthread_join( thread_control_handle, 0);
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_INPUT;
    io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_INPUT;
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
    GPIO_SET( motor_pins->gpio, motor_pins->BI2_pin );
  } else {
    GPIO_CLR( motor_pins->gpio, motor_pins->BI2_pin );
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


void update_command(motor_pin_values *motor_pin_values, char current_command, int *mode, struct data_signal * data_signal, data_sample * data_samples, unsigned int * sample_count){
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
      
      // Signal Data thread to start recording
      if (mode != MODE_0){
        pthread_mutex_lock( &(data_signal->lock) );
        data_signal->recording = false;
        data_signal->m0 = false;
        write_to_file(*mode, data_samples, sample_count);
        pthread_mutex_unlock( &(data_signal->lock) );
      }


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
      if (mode != MODE_0){
        pthread_mutex_lock( &(data_signal->lock) );
        data_signal->recording = true;
        data_signal->m0 = false;
        pthread_mutex_unlock( &(data_signal->lock) );
      }

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
      
      if(motor_pin_values->A_PWM > PWM_MOTOR_MIN){motor_pin_values->A_PWM_next -= PWM_TURN_STEP;}
      if (motor_pin_values->B_PWM < PWM_MOTOR_MAX) {motor_pin_values->B_PWM_next += PWM_TURN_STEP;}
      
      #ifdef DEBUG
      printf("\nMOTOR: A_PWM = %i\n", motor_pin_values->A_PWM_next);
      printf("MOTOR: B_PWM = %i\n", motor_pin_values->B_PWM_next);
      #endif
      break;

    case 'd':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: RIGHT\n");
      #endif
      
      if (motor_pin_values->A_PWM < PWM_MOTOR_MAX) {motor_pin_values->A_PWM_next += PWM_TURN_STEP;}
      if(motor_pin_values->B_PWM > PWM_MOTOR_MIN) {motor_pin_values->B_PWM_next -= PWM_TURN_STEP;}
      
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

      // Start m0 data sampling
      pthread_mutex_lock( &(data_signal->lock) );
      data_signal->recording = true;
      data_signal->m0 = true;
      pthread_mutex_unlock( &(data_signal->lock) );

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

    case 't':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 2\n");
      #endif
      printf("This feature will be available at a later date.");
      break;  

    case 'n':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 2\n");
      #endif
      printf("This feature will be available at a later date.");
      break;  

    case 'p':
      #ifdef DEBUG
      printf("\nMOTOR: Recieved Command: MODE 2\n");
      #endif
      print_samples(data_samples, sample_count);
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

/////////////////////// DATA FUNCTIONS /////////////////////////////
void read_MPU6050_registers(                          /* read a register */
    uint8_t                         I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address, /* the address to read from */
    uint8_t *                       read_data,        /* the data read from the SPI device */
    size_t                          data_length,      /* the length of data to send/receive */
    volatile struct bsc_register *  bsc )             /* the BSC address */
{
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 1;
  bsc->FIFO.value      = register_address;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 1;
  bsc->DLEN.field.DLEN = data_length;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  while (data_length > 0)
  {
    *read_data = bsc->FIFO.field.DATA;

    read_data++;
    data_length--;
  }

  return;
}

union MPU6050_transaction_field_data read_MPU6050_register( /* read a register, returning the read value */
    uint8_t                         I2C_address,            /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address,       /* the address to read from */
    volatile struct bsc_register *  bsc )                   /* the BSC address */
{
  union MPU6050_transaction transaction;

  read_MPU6050_registers( I2C_address, register_address, &(transaction.value[1]), 1, bsc );

  return transaction.field.data;
}

void write_MPU6050_register(                                /* write a register */
    uint8_t                               I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                      register_address, /* the address to read from */
    union MPU6050_transaction_field_data  value,            /* the value to write */
    volatile struct bsc_register *        bsc )             /* the BSC address */
{
  union MPU6050_transaction transaction;

  transaction.field.data = value;
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 2;
  bsc->FIFO.value      = register_address;
  bsc->FIFO.value      = transaction.value[1];
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  return;
}

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU6050_transaction_field_data  transaction;
  uint8_t                               data_block_fifo_count[2];
  union uint16_to_2uint8                reconstructor;
  uint16_t                              ii;
  uint16_t                              packet_count;
  int32_t                               gyro_bias_x;
  int32_t                               gyro_bias_y;
  int32_t                               gyro_bias_z;
  int32_t                               accel_bias_x;
  int32_t                               accel_bias_y;
  int32_t                               accel_bias_z;
  uint8_t                               data_block_fifo_packet[12];
  union uint16_to_2uint8                reconstructor_accel_x;
  union uint16_to_2uint8                reconstructor_accel_y;
  union uint16_to_2uint8                reconstructor_accel_z;
  union uint16_to_2uint8                reconstructor_gyro_x;
  union uint16_to_2uint8                reconstructor_gyro_y;
  union uint16_to_2uint8                reconstructor_gyro_z;

  // reset device
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 100000 );

  // get stable time source; auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.PWR_MGMT_2.STBY_ZG      = 0;
  transaction.PWR_MGMT_2.STBY_YG      = 0;
  transaction.PWR_MGMT_2.STBY_XG      = 0;
  transaction.PWR_MGMT_2.STBY_ZA      = 0;
  transaction.PWR_MGMT_2.STBY_YA      = 0;
  transaction.PWR_MGMT_2.STBY_XA      = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_2, transaction, bsc );
  usleep( 200000 );

  // configure device for bias calculation
  transaction.INT_ENABLE.DATA_RDY_EN    = 0; // disable all interrupts
  transaction.INT_ENABLE.reserved0      = 0;
  transaction.INT_ENABLE.I2C_MST_INT_EN = 0;
  transaction.INT_ENABLE.FIFO_OFLOW_EN  = 0;
  transaction.INT_ENABLE.reserved1      = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_ENABLE, transaction, bsc );
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0; // disable FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN    = 0;
  transaction.FIFO_EN.YG_FIFO_EN    = 0;
  transaction.FIFO_EN.XG_FIFO_EN    = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  transaction.PWR_MGMT_1.CLKSEL       = 0;  // turn on internal clock source
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.I2C_MST_CTRL.I2C_MST_CLK   = 0; // disable I2C master
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES   = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN   = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_I2C_MST_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RESET  = 0; // disable FIFO and I2C master modes
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 0;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RESET  = 0; // reset FIFO and DMP
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 1;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  usleep( 15000 );

  // configure MPU6050 gyro and accelerometer for bias calculation
  transaction.CONFIG.DLPF_CFG     = 1;  // set low-pass filter to 188Hz
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc );
  transaction.SMPLRT_DIV.SMPLRT_DIV = 0;  // set sample rate to 1kHz
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_SMPLRT_DIV, transaction, bsc );
  transaction.GYRO_CONFIG.reserved0 = 0;  // set gyro full-scale to 250dps, maximum sensitivity
  transaction.GYRO_CONFIG.FS_SEL    = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc );
  transaction.ACCEL_CONFIG.reserved     = 0; // set accelerometer full-scale to 2g, maximum sensitivity
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST        = 0;
  transaction.ACCEL_CONFIG.YA_ST        = 0;
  transaction.ACCEL_CONFIG.XA_ST        = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc );

  calibration_accelerometer->scale = 2.0/32768.0;  // measurement scale/signed numeric range
  calibration_accelerometer->offset_x = 0;
  calibration_accelerometer->offset_y = 0;
  calibration_accelerometer->offset_z = 0;

  calibration_gyroscope->scale = 250.0/32768.0;
  calibration_gyroscope->offset_x = 0;
  calibration_gyroscope->offset_y = 0;
  calibration_gyroscope->offset_z = 0;

  // configure FIFO to capture accelerometer and gyro data for bias calculation
  transaction.USER_CTRL.SIG_COND_RESET = 0; // enable FIFO
  transaction.USER_CTRL.I2C_MST_RESET  = 0;
  transaction.USER_CTRL.FIFO_RESET     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 1;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0;  // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU6050)
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 1;
  transaction.FIFO_EN.ZG_FIFO_EN    = 1;
  transaction.FIFO_EN.YG_FIFO_EN    = 1;
  transaction.FIFO_EN.XG_FIFO_EN    = 1;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  usleep( 40000 );  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // at end of sample accumulation, turn off FIFO sensor read
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0;  // disable gyro and accelerometer sensors for FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN    = 0;
  transaction.FIFO_EN.YG_FIFO_EN    = 0;
  transaction.FIFO_EN.XG_FIFO_EN    = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_COUNTH, data_block_fifo_count, sizeof(data_block_fifo_count), bsc ); // read FIFO sample count
  reconstructor.field.H = data_block_fifo_count[0];
  reconstructor.field.L = data_block_fifo_count[1];
  packet_count = reconstructor.unsigned_value / 12; // how many sets of full gyro and accelerometer data for averaging

  accel_bias_x = 0;
  accel_bias_y = 0;
  accel_bias_z = 0;
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  for (ii = 0; ii < packet_count; ii++)
  {
    read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_R_W, data_block_fifo_packet, sizeof(data_block_fifo_packet), bsc ); // read data for averaging

    reconstructor_accel_x.field.H = data_block_fifo_packet[0];
    reconstructor_accel_x.field.L = data_block_fifo_packet[1];
    reconstructor_accel_y.field.H = data_block_fifo_packet[2];
    reconstructor_accel_y.field.L = data_block_fifo_packet[3];
    reconstructor_accel_z.field.H = data_block_fifo_packet[4];
    reconstructor_accel_z.field.L = data_block_fifo_packet[5];
    reconstructor_gyro_x.field.H  = data_block_fifo_packet[6];
    reconstructor_gyro_x.field.L  = data_block_fifo_packet[7];
    reconstructor_gyro_y.field.H  = data_block_fifo_packet[8];
    reconstructor_gyro_y.field.L  = data_block_fifo_packet[9];
    reconstructor_gyro_z.field.H  = data_block_fifo_packet[10];
    reconstructor_gyro_z.field.L  = data_block_fifo_packet[11];

    accel_bias_x += reconstructor_accel_x.signed_value; // sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias_y += reconstructor_accel_y.signed_value;
    accel_bias_z += reconstructor_accel_z.signed_value;
    gyro_bias_x  += reconstructor_gyro_x.signed_value;
    gyro_bias_y  += reconstructor_gyro_y.signed_value;
    gyro_bias_z  += reconstructor_gyro_z.signed_value;
  }
  accel_bias_x /= (int32_t)packet_count;
  accel_bias_y /= (int32_t)packet_count;
  accel_bias_z /= (int32_t)packet_count;
  gyro_bias_x /= (int32_t)packet_count;
  gyro_bias_y /= (int32_t)packet_count;
  gyro_bias_z /= (int32_t)packet_count;

  if (accel_bias_z > 0) // remove gravity from the z-axis accelerometer bias calculation
  {
    accel_bias_z -= (int32_t)(1.0/calibration_accelerometer->scale);
  }
  else
  {
    accel_bias_z += (int32_t)(1.0/calibration_accelerometer->scale);
  }

  // the code that this is based off of tried to push the bias calculation values to hardware correction registers
  // these registers do not appear to be functioning, so rely on software offset correction

  // output scaled gyro biases
  calibration_gyroscope->offset_x = ((float)gyro_bias_x)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_y = ((float)gyro_bias_y)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_z = ((float)gyro_bias_z)*calibration_gyroscope->scale;

  // output scaled accelerometer biases
  calibration_accelerometer->offset_x = ((float)accel_bias_x)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_y = ((float)accel_bias_y)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_z = ((float)accel_bias_z)*calibration_accelerometer->scale;

  return;
}

void initialize_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU6050_transaction_field_data  transaction;

  /* print WHO_AM_I */
  printf( "accel WHOAMI (0x68) = 0x%2.2X\n",
      read_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_WHO_AM_I, bsc ).WHO_AM_I.WHOAMI );

  // based off https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

  calibrate_accelerometer_and_gyroscope( calibration_accelerometer, calibration_gyroscope, bsc );

  // reset MPU9205
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 1000 ); // wait for all registers to reset

  // clock source
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );

  // enable acc & gyro
  transaction.PWR_MGMT_2.STBY_ZG      = 0;
  transaction.PWR_MGMT_2.STBY_YG      = 0;
  transaction.PWR_MGMT_2.STBY_XG      = 0;
  transaction.PWR_MGMT_2.STBY_ZA      = 0;
  transaction.PWR_MGMT_2.STBY_YA      = 0;
  transaction.PWR_MGMT_2.STBY_XA      = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );

  // use DLPF set gyro bandwidth 184Hz, temperature bandwidth 188Hz
  transaction.CONFIG.DLPF_CFG     = 1;
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc );

  // +-250dps
  transaction.GYRO_CONFIG.reserved0 = 0;
  transaction.GYRO_CONFIG.FS_SEL    = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc );

  // +-2G
  transaction.ACCEL_CONFIG.reserved     = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST        = 0;
  transaction.ACCEL_CONFIG.YA_ST        = 0;
  transaction.ACCEL_CONFIG.XA_ST        = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc );

  // force into I2C mode, disabling I2C master
  transaction.USER_CTRL.SIG_COND_RESET  = 0;
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 0;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );

  // enable bypass mode
  transaction.INT_PIN_CFG.reserved        = 0;
  transaction.INT_PIN_CFG.I2C_BYPASS_EN   = 1;
  transaction.INT_PIN_CFG.FSYNC_INT_EN    = 0;
  transaction.INT_PIN_CFG.FSYNC_INT_LEVEL = 0;
  transaction.INT_PIN_CFG.INT_RD_CLEAR    = 0;
  transaction.INT_PIN_CFG.LATCH_INT_EN    = 0;
  transaction.INT_PIN_CFG.INT_OPEN        = 0;
  transaction.INT_PIN_CFG.INT_LEVEL       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_PIN_CFG, transaction, bsc );

  return;
}

void read_accelerometer_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc,
    data_sample * data_samples,
    unsigned int * sample_count )
{
  uint8_t                   data_block[6+2+6];
  union uint16_to_2uint8    ACCEL_XOUT;
  union uint16_to_2uint8    ACCEL_YOUT;
  union uint16_to_2uint8    ACCEL_ZOUT;
  union uint16_to_2uint8    GYRO_XOUT;
  union uint16_to_2uint8    GYRO_YOUT;
  union uint16_to_2uint8    GYRO_ZOUT;

  data_sample sample;

  /*
   * poll the interrupt status register and it tells you when it is done
   * once it is done, read the data registers
   */
  do
  {
    usleep( 1000 );
  } while (read_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_STATUS, bsc ).INT_STATUS.DATA_RDY_INT == 0);

  // read the accelerometer values
  read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H, data_block, sizeof(data_block), bsc );
  ACCEL_XOUT.field.H  = data_block[0];
  ACCEL_XOUT.field.L  = data_block[1];
  ACCEL_YOUT.field.H  = data_block[2];
  ACCEL_YOUT.field.L  = data_block[3];
  ACCEL_ZOUT.field.H  = data_block[4];
  ACCEL_ZOUT.field.L  = data_block[5];
  // TEMP_OUT.field.H = data_block[6];
  // TEMP_OUT.field.L = data_block[7];
  GYRO_XOUT.field.H   = data_block[8];
  GYRO_XOUT.field.L   = data_block[9];
  GYRO_YOUT.field.H   = data_block[10];
  GYRO_YOUT.field.L   = data_block[11];
  GYRO_ZOUT.field.H   = data_block[12];
  GYRO_ZOUT.field.L   = data_block[13];

  // printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
  //     GYRO_XOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_x,
  //     GYRO_YOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_y,
  //     GYRO_ZOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_z );

  // printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
  //     (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81,
  //     (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81,
  //     (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 );

  sample.accel_xout = ((ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81);
  sample.accel_yout = ((ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81);
  sample.accel_zout = ((ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81);
  sample.gyro_xout = (GYRO_XOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_x);
  sample.gyro_yout = (GYRO_YOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_y);
  sample.gyro_zout = (GYRO_ZOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_z);

  data_samples[*sample_count] = sample;
  *sample_count += 1;

  return;
}

#define DEBUG

data_sample average_sample(data_sample * data_samples, unsigned int * sample_count){
  
  unsigned int i, samples = *sample_count;
  data_sample average_sample;
  int tenth;

  average_sample.accel_xout = data_samples[0].accel_xout;
  average_sample.accel_yout = data_samples[0].accel_yout;
  average_sample.accel_zout = data_samples[0].accel_zout;
  average_sample.gyro_xout = data_samples[0].gyro_xout;
  average_sample.gyro_yout = data_samples[0].gyro_yout;
  average_sample.gyro_zout = data_samples[0].gyro_zout;

  #ifdef DEBUG
  printf( "Average Sample:\n");
    
  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
    average_sample.gyro_xout,
    average_sample.gyro_yout,
    average_sample.gyro_zout
  );
    
  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
    average_sample.accel_xout,
    average_sample.accel_yout,
    average_sample.accel_zout
  );
  #endif  

  for(i = 1; i < samples; i++){
    average_sample.accel_xout = (average_sample.accel_xout * i + data_samples[i].accel_xout) / (i + 1);
    average_sample.accel_yout = (average_sample.accel_yout * i + data_samples[i].accel_yout) / (i + 1);
    average_sample.accel_zout = (average_sample.accel_zout * i + data_samples[i].accel_zout) / (i + 1);
    average_sample.gyro_xout = (average_sample.gyro_xout * i + data_samples[i].gyro_xout) / (i + 1);
    average_sample.gyro_yout = (average_sample.gyro_yout * i + data_samples[i].gyro_yout) / (i + 1);
    average_sample.gyro_zout = (average_sample.gyro_zout * i + data_samples[i].gyro_zout) / (i + 1);
  }

  #ifdef DEBUG
  printf( "Average Sample:\n");
    
  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
    average_sample.gyro_xout,
    average_sample.gyro_yout,
    average_sample.gyro_zout
  );
    
  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
    average_sample.accel_xout,
    average_sample.accel_yout,
    average_sample.accel_zout
  );  
  #endif

}

void print_samples(data_sample * data_samples, unsigned int * sample_count){

  unsigned int i, samples = *sample_count;
  data_sample max = data_samples[0], min = data_samples[0], tenth;

  for(i = 1; i < samples; i++){
    if (data_samples[i].accel_xout < min.accel_xout) {
      min.accel_xout = data_samples[i].accel_xout;
    } else if (data_samples[i].accel_xout > max.accel_xout) {
      max.accel_xout = data_samples[i].accel_xout;
    }

    if (data_samples[i].accel_yout < min.accel_yout) {
      min.accel_yout = data_samples[i].accel_yout;
    } else if (data_samples[i].accel_yout > max.accel_yout) {
      max.accel_yout = data_samples[i].accel_yout;
    }

    if (data_samples[i].accel_zout < min.accel_zout) {
      min.accel_zout = data_samples[i].accel_zout;
    } else if (data_samples[i].accel_zout > max.accel_zout) {
      max.accel_zout = data_samples[i].accel_zout;
    }

    if (data_samples[i].gyro_xout < min.gyro_xout) {
      min.gyro_xout = data_samples[i].gyro_xout;
    } else if (data_samples[i].gyro_xout > max.gyro_xout) {
      max.gyro_xout = data_samples[i].gyro_xout;
    }

    if (data_samples[i].gyro_yout < min.gyro_yout) {
      min.gyro_yout = data_samples[i].gyro_yout;
    } else if (data_samples[i].gyro_yout > max.gyro_yout) {
      max.gyro_yout = data_samples[i].gyro_yout;
    }

    if (data_samples[i].gyro_xout < min.gyro_xout) {
      min.gyro_xout = data_samples[i].gyro_xout;
    } else if (data_samples[i].gyro_xout > max.gyro_xout) {
      max.gyro_xout = data_samples[i].gyro_xout;
    }
  }

  printf( "\nMax:\n");
    
  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
    max.gyro_xout,
    max.gyro_yout,
    max.gyro_zout
  );
    
  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
    max.accel_xout,
    max.accel_yout,
    max.accel_zout 
  );

  printf( "\nMin:\n");
    
  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
    min.gyro_xout,
    min.gyro_yout,
    min.gyro_zout
  );
    
  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
    min.accel_xout,
    min.accel_yout,
    min.accel_zout 
  );

  printf( "\nRange:\n");
    
  printf( "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
    max.gyro_xout - min.gyro_xout,
    max.gyro_yout - min.gyro_yout,
    max.gyro_zout - min.gyro_zout
  );
    
  printf( "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
    max.accel_xout - min.accel_xout,
    max.accel_yout - min.accel_yout,
    max.accel_zout - min.accel_zout 
  );

  tenth.accel_xout = (max.accel_xout - min.accel_xout)/10;
  tenth.accel_yout = (max.accel_yout - min.accel_yout)/10;
  tenth.accel_zout = (max.accel_zout - min.accel_zout)/10;
  tenth.gyro_xout = (max.gyro_xout - min.gyro_xout)/10;
  tenth.gyro_yout = (max.gyro_yout - min.gyro_yout)/10;
  tenth.gyro_zout = (max.gyro_zout - min.gyro_zout)/10;

  printf( "\nGY |AC\n");
  
  for(i = 0; i < samples; i++){
    printf( "%i%i%i|",
      (long)(data_samples[i].gyro_xout / tenth.gyro_xout),
      (long)(data_samples[i].gyro_yout / tenth.gyro_yout),
      (long)(data_samples[i].gyro_zout / tenth.gyro_zout)
    );
      
    printf( "%i%i%i\n",
      (long)(data_samples[i].accel_xout / tenth.accel_xout),
      (long)(data_samples[i].accel_yout / tenth.accel_yout),
      (long)(data_samples[i].accel_zout / tenth.accel_zout)
    );   
  }
}

void write_to_file(int mode, data_sample * data_samples, unsigned int * sample_count){

  unsigned int i, samples = *sample_count;
  FILE * file;

  switch (mode)
  {
    case 0:
      file = fopen("./hw7m0data.txt", "w");
      break;
    case 1:
      file = fopen("./hw7m1data.txt", "w");
      break;
    case 2:
      file = fopen("./hw7m2data.txt", "w");
      break;    
    default:
      break;
  }

  fprintf(file, "Gyro X, Gyro Y, Gyro Z, Accel X, Accel Y, Accel Z\n");

  for(i = 0; i < samples; i++){
    
    fprintf( file, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
      data_samples[i].gyro_xout,
      data_samples[i].gyro_yout,
      data_samples[i].gyro_zout,
      data_samples[i].accel_xout,
      data_samples[i].accel_yout,
      data_samples[i].accel_zout 
    );
  }

  fclose(file);
}

