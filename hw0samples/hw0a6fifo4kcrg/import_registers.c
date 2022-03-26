#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include "cm.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "io_peripherals.h"
#include "import_registers.h"

#if 0
#define PHYSICAL_ADDRESS 0x20000000 /* base for BCM2708 */
#elseif 0
#define PHYSICAL_ADDRESS 0x3F000000 /* base for BCM2709 */
#else
#define PHYSICAL_ADDRESS 0xFE000000 /* base for BCM2835 */
#endif
#define ADDRESS_LENGTH   0x02000000

struct io_peripherals *import_registers( void )
{
  static struct io_peripherals io;
  struct io_peripherals *return_value; /* the return value of this function */
  int             mmap_file;    /* the file descriptor used to map the memory */
  off_t           page_size;

  mmap_file = open( "/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC );
  if (mmap_file != -1)
  {
//    page_size = getpagesize();
    page_size = 4096;

    /* try to put the physical I/O space at the same address range in the virtual address space */
    io.cm = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x101000 );
    if (io.cm != MAP_FAILED)
    {
      io.gpio = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x200000 );
      if (io.gpio != MAP_FAILED)
      {
        io.spi = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x204000 );
        if (io.spi != MAP_FAILED)
        {
          io.pwm = mmap( 0, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, mmap_file, PHYSICAL_ADDRESS+0x20c000 );
          if (io.pwm != MAP_FAILED)
          {
            return_value = &io; /* mapped memory */
          }
          else
          {
            printf( "unable to map register space4\n" );

            close( mmap_file );

            return_value = NULL;
          }
        }
        else
        {
          printf( "unable to map register space3\n" );

          close( mmap_file );

          return_value = NULL;
        }
      }
      else
      {
        printf( "unable to map register space2\n" );

        close( mmap_file );

        return_value = NULL;
      }
    }
    else
    {
      printf( "unable to map register space1\n" );

      close( mmap_file );

      return_value = NULL;
    }
  }
  else
  {
    printf( "unable to open /dev/mem\n" );

    return_value = NULL;
  }

  return return_value;
}

