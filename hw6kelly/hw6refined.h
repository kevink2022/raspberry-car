#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <semaphore.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"

struct pause_flag
{
  pthread_mutex_t lock;
  bool            pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool            done;
};

typedef struct {
  volatile struct gpio_register * gpio;
  volatile struct pwm_register  * pwm;
  int                             A_PWM_pin;
  int                             AI1_pin;
  int                             AI2_pin;
  int                             AIR_pin;
  int                             B_PWM_pin;
  int                             BI1_pin;
  int                             BI2_pin;
  int                             BIR_pin;
} motor_pins;
  
typedef struct {
  int                             A_PWM;
  int                             AI1;
  int                             AI2;
  int                             AIR;
  int                             B_PWM;
  int                             BI1;
  int                             BI2;
  int                             BIR;

  int                             A_PWM_next;
  int                             AI1_next;
  int                             AI2_next;
  int                             AIR_next;
  int                             B_PWM_next;
  int                             BI1_next;
  int                             BI2_next;
  int                             BIR_next;
} motor_pin_values;

typedef struct {
  char              * control_queue;
  unsigned int      * control_queue_length;
  pthread_mutex_t   * control_queue_lock;
} control_queue;

enum mode {
  MODE_0,
  MODE_1,
  MODE_2
};

struct clock_thread_parameter
{
  struct done_flag  * done;
  int                 period;
  sem_t             * control_thread_sem;
  // sem_t             * data_thread_sem; *for hw7
};

struct key_thread_parameter
{
  struct done_flag  * done;
  control_queue     * control_queue;
};

struct control_thread_parameter
{
  struct done_flag  * done;
  sem_t             * control_thread_sem;
  control_queue     * control_queue;
  char              * current_command;
  pthread_mutex_t   * current_command_lock;
  struct pause_flag * motor_pause;
};


struct motor_thread_parameter
{
  struct done_flag  * done;
  struct pause_flag * pause;
  char              * current_command;
  pthread_mutex_t   * current_command_lock;
  motor_pins        * motor_pins;
};


void *ThreadClock( void * arg  );
void *ThreadControl( void * arg  );
void *ThreadMotor( void * arg  );
int get_pressed_key(void);
void *ThreadKey( void * arg );

/////////////////////////// Motor thread functions ///////////////////////////
void init_motor_pin_values(motor_pin_values *motor_pin_values);
void smooth_speed_change(motor_pins *motor_pins, motor_pin_values *motor_pin_values, int initial_speed, int final_speed, int step);
void turn(void);
void set_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_motor_pwm(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_command(motor_pin_values *motor_pin_values, char current_command, int *mode);


// Key thread func
void add_to_queue(control_queue *control_queue, char command);