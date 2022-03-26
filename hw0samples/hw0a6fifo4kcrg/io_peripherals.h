/*
 * io_peripherals.h
 *
 *  Created on: Feb 3, 2018
 *      Author: steveb
 */

#ifndef IO_PERIPHERALS_H_
#define IO_PERIPHERALS_H_

struct pcm_register
{
  uint8_t unused; /* empty structure */
};

struct io_peripherals
{
  volatile struct cm_register   *cm;               /* offset = 0x101000, width = 0xA8 */
  volatile struct gpio_register *gpio;             /* offset = 0x200000, width = 0x84 */
  volatile struct pcm_register  *pcm;              /* offset = 0x203000, width = 0x24 */
  volatile struct spi_register  *spi;              /* offset = 0x204000, width = 0x18 */
  volatile struct pwm_register  *pwm;              /* offset = 0x20c000, width = 0x28 */
};

#endif /* IO_PERIPHERALS_H_ */
