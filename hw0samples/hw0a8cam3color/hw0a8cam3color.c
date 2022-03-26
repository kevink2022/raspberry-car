/**************************************************
* CMPEN 473, Spring 2022, Penn State University
*
* Sample Program hw0a8cam3color   ***  ***
*   - Assumming a camera attached to Raspberry Pi computer
*   - Assumming the RaspiCAM library installed
*   - This program takes a picture and then save it to a file, plus
*   -   separating the image into Red, Green, and Blue colors and
*   -   save each color image as a picture, saving them to three files.
*
*           Author: K. Choi
*       Created on: March 08, 2022
*  Last updated on: March 12, 2022
*       Updated by: K.Choi
*
* Genetic Raspberry Pi camera connected to Raspberry Pi 4 computer
* (low cost camera (Arducam 5MP) available on Amazon.com, $10.00/board - 2022)
*
***************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "bsc.h"
#include "pwm.h"
#include "enable_pwm_clock.h"
#include "io_peripherals.h"
#include "wait_period.h"
#include "FIFO.h"
#include "MPU6050.h"
#include "MPU9250.h"
#include "wait_key.h"
#include "raspicam_wrapper.h"

int main( int argc, char *argv[] )
{
  struct raspicam_wrapper_handle *  Camera;       /* Camera handle */
  int                               return_value; /* the return value of this program */
  size_t                            image_size;

    /* Open camera */
    printf( "Opening Camera...\n" );

    Camera = raspicam_wrapper_create();
    if (Camera != NULL)
    {
      if (raspicam_wrapper_open( Camera ))
      {
        // wait a while until camera stabilizes, once at the beginning
        printf( "Sleeping for 1 secs\n" );
        sleep( 1 );

        // capture - take picture
        raspicam_wrapper_grab( Camera );

        // allocate memory to obtain the image data of picture
        image_size = raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB );
        // find image size first, then allocate the memory of the image size

        // image data treated as 1 dimensional string, create another copy for work
        unsigned char *data = (unsigned char *)malloc( image_size );
        unsigned char *data_copy = (unsigned char *)malloc( image_size );

        // extract the image in rgb format
        // transfer camera image to 'data'
        raspicam_wrapper_retrieve( Camera, data, RASPICAM_WRAPPER_FORMAT_RGB );
        memcpy( data_copy, data, image_size );

        // save the image as picture file, .ppm format file
        FILE * outFile = fopen( "pic1.ppm", "wb" );
        if (outFile != NULL)
        {
          fprintf( outFile, "P6\n" );  // write .ppm file header
          fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
          // write the image data
          fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
          fclose( outFile );
          printf( "Image, picture saved as pic1.ppm\n" );


          // change the image for Red color only, and save it as another picture file
          outFile = fopen( "pic1red.ppm", "wb" );
          if (outFile != NULL)
          {
            // now read data as RGB pixel
            // convert to Red-color-only image/picture
            struct RGB_pixel
            {
                unsigned char R;
                unsigned char G;
                unsigned char B;
            };
            struct RGB_pixel* source_pixel;
            struct RGB_pixel* destination_pixel;
            unsigned int      pixel_count;
            unsigned int      pixel_index;
            unsigned char     pixel_value;
            
            // view data as R-byte, G-byte, and B-byte per pixel
            source_pixel      = (struct RGB_pixel *)data_copy;
            destination_pixel = (struct RGB_pixel *)data;
            pixel_count = raspicam_wrapper_getHeight( Camera ) * raspicam_wrapper_getWidth( Camera );
            for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
            {
              // pickup only the Red color
              pixel_value = (((unsigned int)(source_pixel[pixel_index].R)) +
                             ( 0 ) +
                             ( 0 )) / 1; // do not worry about rounding
              destination_pixel[pixel_index].R = pixel_value;
              destination_pixel[pixel_index].G = 0;
              destination_pixel[pixel_index].B = 0;
            }
            
            // save the new image as picture file, .ppm format file
            fprintf( outFile, "P6\n" );  // write .ppm file header
            fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
            // write the image data
            fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
            fclose( outFile );
            printf( "Image, picture saved as pic1red.ppm\n" );
            
            
            
            // change the image for Green color only, and save it as another picture file
            outFile = fopen( "pic1green.ppm", "wb" );
            if (outFile != NULL)
            {
              // convert to Green-color-only image/picture
              source_pixel      = (struct RGB_pixel *)data_copy;
              destination_pixel = (struct RGB_pixel *)data;
              pixel_count = raspicam_wrapper_getHeight( Camera ) * raspicam_wrapper_getWidth( Camera );
              for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
              {
                // pickup only the Green color
                pixel_value = (( 0 ) +
                               ((unsigned int)(source_pixel[pixel_index].G)) +
                               ( 0 )) / 1; // do not worry about rounding
                destination_pixel[pixel_index].R = 0;
                destination_pixel[pixel_index].G = pixel_value;
                destination_pixel[pixel_index].B = 0;
              }
              
              // save the new image as picture file, .ppm format file
              fprintf( outFile, "P6\n" );
              fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
              // write the image data
              fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
              fclose( outFile );
              printf( "Image, picture saved as pic1green.ppm\n" );
              
              
              
              // change the image for Blue color only, and save it as another picture file
              outFile = fopen( "pic1blue.ppm", "wb" );
              if (outFile != NULL)
              {
                // convert to Blue-color-only image/picture
                source_pixel      = (struct RGB_pixel *)data_copy;
                destination_pixel = (struct RGB_pixel *)data;
                pixel_count = raspicam_wrapper_getHeight( Camera ) * raspicam_wrapper_getWidth( Camera );
                for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
                {
                  // pickup only the Blue color
                  pixel_value = (( 0 ) +
                                 ( 0 ) +
                                 ((unsigned int)(source_pixel[pixel_index].B))) / 1; // do not worry about rounding
                  destination_pixel[pixel_index].R = 0;
                  destination_pixel[pixel_index].G = 0;
                  destination_pixel[pixel_index].B = pixel_value;
                }
                
                // save the new image as picture file, .ppm format file
                fprintf( outFile, "P6\n" );
                fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
                // write the image data
                fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
                fclose( outFile );
                printf( "Image, picture saved as pic1blue.ppm\n" );
                
                return_value = 0;
              }
              else
              {
                printf( "unable to save separate Blue color image\n" );
                return_value = -1;
              }
              
              
            }
            else
            {
              printf( "unable to save separate Green color image\n" );
              return_value = -1;
            }
            
            
          }
          else
          {
            printf( "unable to save separate Red color image\n" );
            return_value = -1;
          }
          
          
        }
        else
        {
          printf( "unable to open output file, to save the camera image\n" );
          return_value = -1;
        }
        
        //free resources
        free( data );
        free( data_copy );
        raspicam_wrapper_destroy( Camera );
      }
      else
      {
        printf( "unable to open output file\n" );
        return_value = -1;
      }
      
      
    }
    else
    {
      printf( "Error opening camera\n" );
      printf( "Unable to allocate camera handle\n" );
      return_value = -1;
    }
    
    
    return return_value;
}
