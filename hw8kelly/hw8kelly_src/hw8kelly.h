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
#include "uart.h"
#include "bsc.h"
#include "MPU6050.h"
#include "MPU9250.h"



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

struct data_signal
{
  pthread_mutex_t lock;
  bool            recording;
  bool            m0;
};

typedef struct {
  int                             A_PWM;
  int                             AI1;
  int                             AI2;
  int                             AIR;
  int                             B_PWM;
  int                             BI1;
  int                             BI2;
  int                             BIR;
} motor_pin_values;

typedef struct 
{
  pthread_mutex_t     lock;
  bool                recording;
  motor_pin_values  * camera_set_pins;
  int                 delay_mult;
} camera_signal;

struct RGB_pixel
{
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

typedef struct 
{
  float gyro_xout;
  float gyro_yout;
  float gyro_zout;
  float accel_xout;
  float accel_yout;
  float accel_zout;
} data_sample;

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
  sem_t             * data_thread_sem;
  sem_t             * camera_thread_sem;
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
  struct done_flag    * done;
  struct pause_flag   * pause;
  struct pause_flag   * calibrate;
  struct data_signal  * data_signal;
  camera_signal       * camera_signal;
  data_sample         * data_samples;
  unsigned int        * sample_count;
  char                * current_command;
  pthread_mutex_t     * current_command_lock;
  motor_pins          * motor_pins;
};

struct data_thread_parameter
{
  struct done_flag              * done;
  sem_t                         * data_thread_sem;
  struct data_signal            * data_signal;
  data_sample                   * data_samples;
  unsigned int                  * sample_count;
  struct calibration_data       * calibration_accelerometer;
  struct calibration_data       * calibration_gyroscope;
  struct calibration_data       * calibration_magnetometer;
  volatile struct bsc_register  * bsc;
};

struct camera_thread_parameter
{
  struct done_flag              * done;
  struct pause_flag             * calibrate;
  sem_t                         * camera_thread_sem;
  camera_signal                 * camera_signal;
};


void *ThreadClock( void * arg  );
void *ThreadControl( void * arg  );
void *ThreadMotor( void * arg  );
void *ThreadData( void * arg  );
int get_pressed_key(void);
void *ThreadKey( void * arg );

/////////////////////////// Motor thread functions ///////////////////////////
void init_motor_pin_values(motor_pin_values *motor_pin_values);
void smooth_speed_change(motor_pins *motor_pins, motor_pin_values *motor_pin_values, int initial_speed, int final_speed, int step);
void turn(void);
void set_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_motor_pwm(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_motor_pins(motor_pins *motor_pins, motor_pin_values *motor_pin_values);
void update_command(motor_pins *motor_pins, motor_pin_values *motor_pin_values, char next_command, char *command, int *mode, int *hw, struct data_signal * data_signal, data_sample * data_samples, unsigned int * sample_count, camera_signal * camera_signal, struct pause_flag * calibrate);
// Key thread func
void add_to_queue(control_queue *control_queue, char command);



// Data Aquisition Stuff

#define APB_CLOCK 250000000

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))

union uint16_to_2uint8
{
  struct uint16_to_2uint8_field
  {
    uint8_t   L;  /* Little Endian byte order means that the least significant byte goes in the lowest address */
    uint8_t   H;
  }         field;
  uint16_t  unsigned_value;
  int16_t   signed_value;
};

struct calibration_data
{
  float scale;
  float offset_x;
  float offset_y;
  float offset_z;
};

void read_MPU6050_registers(                          /* read a register */
    uint8_t                         I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address, /* the address to read from */
    uint8_t *                       read_data,        /* the data read from the SPI device */
    size_t                          data_length,      /* the length of data to send/receive */
    volatile struct bsc_register *  bsc );             /* the BSC address */

union MPU6050_transaction_field_data read_MPU6050_register( /* read a register, returning the read value */
    uint8_t                         I2C_address,            /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address,       /* the address to read from */
    volatile struct bsc_register *  bsc );                  /* the BSC address */

void write_MPU6050_register(                                /* write a register */
    uint8_t                               I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                      register_address, /* the address to read from */
    union MPU6050_transaction_field_data  value,            /* the value to write */
    volatile struct bsc_register *        bsc );            /* the BSC address */

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc );

void initialize_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc );

void read_accelerometer_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc,
    data_sample * data_samples,
    unsigned int * sample_count  );

data_sample average_sample(data_sample * data_samples, unsigned int * sample_count);

void print_samples(data_sample * data_samples, unsigned int * sample_count);

void write_to_file(int mode, data_sample * data_samples, unsigned int * sample_count);

void calibrate_camera(unsigned char * data, unsigned int* cutoff, int* averages);

void get_offsets(unsigned char * data, unsigned int* cutoff, int* averages, int* offsets);