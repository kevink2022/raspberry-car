/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Sample Program hw0a4pwm1p5 
* 
*   - Use PWM hardware in Raspberry Pi 4 Computer to dim LED lights
*   - Watch the Raspberry Pi 4 processor core utilation with 'htop' command
* 
*   - Dim Red   LED at GPIO12, level=95% using PWM hardware in Raspberry Pi 4 Computer
*   - Dim Green LED at GPIO13, level= 5% using PWM hardware in Raspberry Pi 4 Computer
*   - ending by 'ctl c' key hits
* 
* Revision V1.1   On 1/26/2022
* By Kyusun Choi
* 
* Hardware pwm LED dimming program example in C for 
* Raspberry Pi 4 computer with 
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


/* setting the RPi4 hardware PWM range, power level=pwm.DAT/PWM_RANGE   */
/* Minimum range is 32, typical range is 100, stepping by 1% => pwm.DAT */
/* If 0.1% stepping needed for control, use range of 1000               */
/* Example: pwm.DAT1/PWM_RANGE = 789/1000 = 78.9% level                 */
#define PWM_RANGE 100


int main( void )
{
  uint32_t DLevel;  /* dimming level as duty cycle */
  
  volatile struct io_peripherals *io;

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );
    
    enable_pwm_clock( io );
    
    /* set the pin function to alternate function 0 for GPIO12, PWM for LED on GPIO12 */
    /* set the pin function to alternate function 0 for GPIO13, PWM for LED on GPIO13 */
    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

    /* configure the PWM channels */
    io->pwm.RNG1 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm.RNG2 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm.DAT1 = 1;             /* initial beginning level=1/100=1% */
    io->pwm.DAT2 = 1;             /* initial beginning level=1/100=1% */
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
    io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm, level=pwm.DAT1/PWM_RANGE */
    io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm, level=pwm.DAT2/PWM_RANGE */
    io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    printf( "\n Press 'ctl c' to quit.\n");
    
    DLevel = 95;                      /* set dim level for Red LED */
    io->pwm.DAT1 = DLevel;            /* set Red   LED light level=95/100, GPIO12 dim */
    io->pwm.DAT2 = PWM_RANGE-DLevel;  /* set Green LED light level= 5/100, GPIO13 dim */
    
    while (1)
    {
      usleep(10*1000);                /* 10msec */
    }
    
    /* wait for 'ctl c' key hit */
    

    /* following will not be run */
    printf( "\n 'ctl c' key hit, now quiting ... \n");
    
    /* LED dimming continues by PWM hardware even after program ends */

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
