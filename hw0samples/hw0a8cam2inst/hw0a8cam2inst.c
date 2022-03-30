/**************************************************
* CMPEN 473, Spring 2022, Penn State University
*
* Sample Program hw0a8cam2inst   ***  ***
*   - Attach a camera to Raspberry Pi computer
*   - Install RaspiCAM library
*   - Take a picture - check Camera and RaspiCAM library
*
*           Author: steveb
*       Created on: Feb.  18, 2017
*  Last updated on: March 10, 2022
*       Updated by: K.Choi
*
* Genetic Raspberry Pi camera connected to Raspberry Pi 4 computer
* (low cost camera (Arducam 5MP) available on Amazon.com, $10.00/board - 2022)
*
***************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
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
        // image data treated as 1 dimensional string
        unsigned char *data = (unsigned char *)malloc( image_size );

        // extract the image in rgb format
        // transfer camera image to 'data'
        raspicam_wrapper_retrieve( Camera, data, RASPICAM_WRAPPER_FORMAT_RGB );

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

          printf( "Image size: %lu\n", image_size );

          printf( "Max pixel: %lu\n", 15 + 1280*15 + 79*16 + 59*16*1280 );


          // change the image as gray scale picture and save as another file
          FILE * outFile = fopen( "pic1g.ppm", "wb" );
          if (outFile != NULL)
          {
            // convert to gray-scale picture, and
            // now read data as RGB pixel
            struct RGB_pixel
            {
                unsigned char R;
                unsigned char G;
                unsigned char B;
            };
            struct RGB_pixel* pixel;
            //unsigned char   * pixel;
            unsigned int      pixel_count;
            unsigned int      pixel_index;
            unsigned char     pixel_value;
            unsigned int      by, bx, iy, ix;
            unsigned long     block_value;
            unsigned long     cutoff = 10;
            int block = 0;

            printf("Height: %d, Width: %d", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ));

            sleep(1);

            printf("Starting processing\n");

            pixel = (struct RGB_pixel*)data; // view data as R-byte, G-byte, and B-byte per pixel
            pixel_count = raspicam_wrapper_getHeight( Camera ) * raspicam_wrapper_getWidth( Camera );
            pixel_value = 0;
            for(by = 0; by < 60; by++){
              for(bx = 0; by < 80; bx++){
                block_value = 0;

                for(iy = 0; iy < 16; iy++){
                  for(ix = 0; ix < 16; ix++){
                    block_value += (((unsigned int)(pixel[ix + 1280*iy + bx*16 + by*16*1280].R)) +
                                  ((unsigned int)(pixel[ix + 1280*iy + bx*16 + by*16*1280].G)) +
                                  ((unsigned int)(pixel[ix + 1280*iy + bx*16 + by*16*1280].B))) / 3; // do not worry about rounding

                    //block_value += (pixel.R[ix + 1280*iy + bx*16 + by*16*1280]);
                  }
                }

                // printf("Block %i value: %lu\n", block, block_value);
                // printf("  Adjusted %lu : Cutoff %lu\n", block_value/256, cutoff);

                if (block_value/256 > cutoff) {
                  for(iy = 0; iy < 16; iy++){
                    for(ix = 0; ix < 16; ix++){
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].R)) = 0;
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].G)) = 0;
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].B)) = 0; // do not worry about rounding
                    }
                  }
                } else {
                  for(iy = 0; iy < 16; iy++){
                    for(ix = 0; ix < 16; ix++){
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].R)) = 255;
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].G)) = 255;
                      ((pixel[ix + 1280*iy + bx*16 + by*16*1280].B)) = 255; // do not worry about rounding
                    }
                  }
                }
              }
            }
              
            // pixel[pixel_index].R = pixel_value;  // same intensity for all three color
            // pixel[pixel_index].G = pixel_value;
            // pixel[pixel_index].B = pixel_value;
            // pixel_value++;
            // if (pixel_value == 255){
            //   pixel_value = 0;
            // }
            //}

            printf("Image Processed");
        
            // save the image as picture file, .ppm format file
            fprintf( outFile, "P6\n" );  // write .ppm file header
            fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
            // write the image data
            fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
            fclose( outFile );
            printf( "Image, picture saved as pic1g.ppm\n" );
      
            return_value = 0;
          }
          else
          {
            printf( "unable to save gray-scale image\n" );
            return_value = -1;
          }

          //free resources
          free( data );
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
        return_value = -1;
      }
    }
    else
    {
      printf( "Unable to allocate camera handle\n" );
      return_value = -1;
    }

    return return_value;
}
