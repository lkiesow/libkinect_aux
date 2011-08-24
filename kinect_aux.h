/*******************************************************************************
 *
 *       Filename:  kinect_aux.h
 *
 *    Description:  Control Kinect motors and read acceleration data.
 *                  Parts taken from ROS and libfreenect
 *
 *        Version:  1.0
 *        Created:  06/23/2011 03:22:43 PM
 *       Modified:  08/24/2011 12:30:00 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lars Kiesow (lkiesow), lkiesow@uos.de
 *         Author:  Denis Meyer (denmeyer), denmeyer@uos.de
 *        Company:  Universität Osnabrück
 *
 ******************************************************************************/

#ifndef KINECT_AUX_H__
#define KINECT_AUX_H__

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "kinect_aux_lib.h"


/*****************************************************
 * global variables
 *****************************************************/


int libusb_initialized = 0;
int device_initialized = 0;


/*****************************************************
 * library functions
 *****************************************************/


/**
 * Initialize libusb
 * @return 1 if successfully initialized libusb, 0 else
 */
int initLibusb() {
  return !libusb_init( 0 );
}

/**
 * Open (kinect) device
 * @param deviceIndex device to open (0 = open first kinect)
 * @return 1 if successfully opened device, 0 else
 */
int openDevice(int deviceIndex)
{
  openAuxDevice( deviceIndex );
  if ( !dev ) {
    return 0;
  }
  return 1;
}


/**
 * Initialize
 * @return 1 if successfully initialized, 0 else
 */
int init() {
  /* initialize libusb */
  if(!libusb_initialized)
    {
      int initMsg = initLibusb();
      if(!initLibusb()) {
	fprintf( stderr, "Error: Cannot initialize libusb\n\n" );
	return 0;
      }
      libusb_initialized = 1;
    }

  /* initialize kinect device */
  if(!device_initialized)
    {
      if(!openDevice(DEVICE_INDEX)) {
	libusb_exit(0);
	fprintf( stderr, "Error: No valid aux device found on device index %d\n\n", DEVICE_INDEX );
	return 0;
      }
      device_initialized = 1;
    }
  return 1;
}

/**
 * clean
 *   - exit libusb
 */
void clean() {
  if(libusb_initialized)
    {
      libusb_exit(0);
      libusb_initialized = 0;
    }
}

/**
 * reads kinect data into a given struct
 * @param kinect_aux_data struct
 */
void getKinectData(kinect_aux_data *data)
{
  uint8_t stat1, stat2;
  *data = readState(&stat1, &stat2);
}

/**
 * automatically levels the kinect
 * @return 1 if successfully leveled, 0 else
 */
int autoLevel() {
  if(!init())
    {
      return 0;
    }
  /* set tiltangle to 0 */
  setTiltAngle( 0.0 );
  /* wait to ensure that the rotation has started */
  /* usleep( 100 ); */
  kinect_aux_data dat;
  do {
    getKinectData(&dat);
    usleep( 10 );
    /* wait until the operation is finished */
  } while ( dat.raw.tilt.status != 0 );
  clean();
  return 1;
}

/**
 * prints the current state
 */
void printCurrentState()
{
  if(!init())
    {
      return;
    }
  kinect_aux_data dat;
  getKinectData(&dat);
  printf( "%3u % 3d % 12f % 5d % 5d % 5d % 12f % 12f % 12f\n",
	  dat.raw.tilt.status,
	  dat.raw.tilt.angle,
	  dat.conv.tilt.angle,
	  dat.raw.acc.x,
	  dat.raw.acc.y,
	  dat.raw.acc.z,
	  dat.conv.acc.x,
	  dat.conv.acc.y,
	  dat.conv.acc.z );
  clean();
}




/* don't call this function */
int test2()
{
  kinect_aux_data dat;
  uint8_t stat1, stat2;
  /* Step 2: Determine current accerleration values */
  int64_t med_raw_accz = 0;
  double  med_angle = 0;
  int i = 0;
  for ( ; i < 100; ++i ) {
    dat = readState(&stat1, &stat2);
    med_raw_accz += dat.raw.acc.z;
  }
  med_raw_accz /= 100;
  
  /* Step 3: Estimate desired angle for zero gravity, apply it and wait till
   * rot is done. */
  printf( "Estimated angle: %f\n", med_angle - ( med_raw_accz * 0.3 / 4.0 ) );
  setTiltAngle( med_angle - ( med_raw_accz * 0.3 / 4.0 ) );
  usleep( 100 );
  do {
    dat = readState(&stat1, &stat2);
    usleep( 10 );
  } while ( dat.raw.tilt.status != 0 );
  
  libusb_exit(0);
  return EXIT_SUCCESS;
}

#endif /* KINECT_AUX_H__ */
