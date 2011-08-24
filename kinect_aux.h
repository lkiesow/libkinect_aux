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
 * defines
 *****************************************************/


#define TILT_SIDE_LOWER -0.5
#define TILT_SIDE_UPPER 0.5


/*****************************************************
 * statuses
 *****************************************************/


#define TURNING 300
#define FAILURE 400


/*****************************************************
 * global variables
 *****************************************************/


int libusb_initialized = 0;
int device_initialized = 0;


/*****************************************************
 * helper functions
 *****************************************************/


/**
 * checks if num is in [lower, upper]
 * @param num the number to check
 * @param lower lower bound (excluded)
 * @param upper upper bound (excluded)
 * @return 1 if num is in [lower, upper], 0 else
 */
int inInterval(double num, double lower, double upper) {
  return (num > lower) && (num < upper);
}


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
 * initialize
 * @return 1 if successfully initialized, 0 else
 */
int initKinectAux() {
  /* initialize libusb */
  if(!libusb_initialized) {
    int initMsg = initLibusb();
    if(!initLibusb()) {
      fprintf( stderr, "Error: Cannot initialize libusb\n\n" );
      return 0;
    }
    libusb_initialized = 1;
  }
  
  /* initialize kinect device */
  if(!device_initialized) {
    if(!openDevice(DEVICE_INDEX)) {
      libusb_exit(0);
      fprintf( stderr, "Error: No valid aux device found on device index %d\n\n", DEVICE_INDEX );
      return 0;
    }
    device_initialized = 1;
  }

  usleep( 10000 );

  return libusb_initialized && device_initialized;
}

/**
 * returns if currently initialized
 * @return 1 if currently initialized, 0 else
 */
int isInitialized() {
  return libusb_initialized && device_initialized;
}

/**
 * clean
 *   - exit libusb
 */
void cleanKinectAux() {
  if(libusb_initialized) {
    libusb_exit(0);
    libusb_initialized = 0;
  }
}

/**
 * reads kinect data into a given struct
 * @param kinect_aux_data struct
 */
void getKinectData(kinect_aux_data *data) {
  uint8_t stat1, stat2;
  *data = readState(&stat1, &stat2);
}

/**
 * automatically levels the kinect
 * @return 1 if successfully leveled, 0 else
 */
int autoLevel() {
  if(isInitialized()) {
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
    return 1;
  }
  return 0;
}

/**
 * checks whether the kinect is currently tilted to the left or the right side
 * @return - TURNING if device is currently turning and no valid data came back
 *         - FAILURE when not initialized
 *         - -1 when tilted to the left side
 *         - 1 when tilted to the right side
 *         - 0 when not tilted
 */
int tiltedTo() {
  if(isInitialized()) {
    kinect_aux_data dat;
    getKinectData(&dat);
    usleep( 10 );
    /* if not currently turning around */
    if(dat.raw.tilt.status == 4) {
      return TURNING;
    }
    if((dat.conv.acc.x > 0) && !inInterval(dat.conv.acc.x, TILT_SIDE_LOWER, TILT_SIDE_UPPER)) {
      return -1;
    } else if((dat.conv.acc.x < 0) && !inInterval(dat.conv.acc.x, TILT_SIDE_LOWER, TILT_SIDE_UPPER)) {
      return 1;
    } else {
      return 0;
    }
  }
  return FAILURE;
}

/**
 * prints the current state
 */
void printCurrentState()
{
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
}

#endif /* KINECT_AUX_H__ */
