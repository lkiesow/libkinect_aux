/*******************************************************************************
 *
 *       Filename:  kinect_aux_lib.h
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

#ifndef KINECT_AUX_LIB_H__
#define KINECT_AUX_LIB_H__

#include <libusb.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

/* VID and PID for Kinect and motor/acc/leds */
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
/* Constants for accelerometers */
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
/* The kinect can tilt from +31 to -31 degrees in what looks like 1 degree
 * increments. The control input looks like 2*desired_degrees */
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)
#define DEVICE_INDEX 0


/*****************************************************
 * global variables and structs
 *****************************************************/


libusb_device_handle * dev = 0;

typedef struct {
  struct {
    struct {
      int16_t x;
      int16_t y;
      int16_t z;
    } acc;
    struct {
      int8_t  angle;
      uint8_t status;
    } tilt;
  } raw;
  struct {
    struct {
      double x;
      double y;
      double z;
    } acc;
    struct {
      double  angle;
      uint8_t status;
    } tilt;
  } conv;
} kinect_aux_data;


/*****************************************************
 * libusb-related functions
 *****************************************************/


/**
 * openAuxDevice
 * @param index
 */
void openAuxDevice( int index ) {
  /* pointer to pointer of device, used to retrieve a list of devices */
  libusb_device ** devs;
  /* get the list of devices */
  int cnt = libusb_get_device_list( 0, &devs );
  if (cnt < 0) {
    fprintf( stderr, "Error: No device on USB\n\n" );
    return;
  }
  
  int i, nr_mot = 0;
  for ( i = 0; i < cnt; i++ ) {
    struct libusb_device_descriptor desc;
    const int r = libusb_get_device_descriptor( devs[i], &desc );
    if ( r < 0 ) {
      continue;
    }
    
    /* Search for the aux */
    if ( desc.idVendor == MS_MAGIC_VENDOR
	 && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT ) {
      
      /* If the index given by the user matches our camera index */
      if (nr_mot == index) {
	if ( ( libusb_open( devs[i], &dev ) != 0 ) || ( dev == 0 ) ) {
	  fprintf( stderr, "Error: Cannot open aux %d.\n\n", index );
	  return;
	}
	// Claim the aux
	libusb_claim_interface( dev, 0 );
	break;
      } else {
	nr_mot++;
      }
    }
  }

  /* free the list, unref the devices in it */
  libusb_free_device_list( devs, 1 );
}

/**
 * readState
 * @param stat1
 * @param stat2
 * @return kinect_aux_data
 */
kinect_aux_data readState(uint8_t *stat1, uint8_t *stat2) {
  uint8_t buf[10];
  const int ret = libusb_control_transfer( dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0 );
  if ( ret != 10 ) {
    fprintf( stderr, "Error: Error in accelerometer reading, "
	     "libusb_control_transfer returned %d\n\n", ret );
    exit( EXIT_FAILURE );
  }
  
  kinect_aux_data dat;
  dat.raw.acc.x       = (int16_t) (((uint16_t)buf[2] << 8) | buf[3]);
  dat.raw.acc.y       = (int16_t) (((uint16_t)buf[4] << 8) | buf[5]);
  dat.raw.acc.z       = (int16_t) (((uint16_t)buf[6] << 8) | buf[7]);
  dat.raw.tilt.angle  = (int8_t) buf[8];
  dat.raw.tilt.status = buf[9];
  
  *stat1 = buf[8];
  *stat2 = (int8_t) buf[8];
  dat.conv.acc.x      = (double) dat.raw.acc.x * GRAVITY / FREENECT_COUNTS_PER_G;
  dat.conv.acc.y      = (double) dat.raw.acc.y * GRAVITY / FREENECT_COUNTS_PER_G;
  dat.conv.acc.z      = (double) dat.raw.acc.z * GRAVITY / FREENECT_COUNTS_PER_G;
  dat.conv.tilt.angle = (double) dat.raw.tilt.angle / 2.;
  dat.conv.tilt.status = dat.raw.tilt.status;
  
  return dat;
}

/**
 * setTiltAngle
 * @param angle
 */
void setTiltAngle( double angle ) {
  uint8_t empty[0x1];
  
  angle = (angle < MIN_TILT_ANGLE) 
    ? MIN_TILT_ANGLE 
    : ((angle > MAX_TILT_ANGLE) 
       ? MAX_TILT_ANGLE 
       : angle);
  angle = angle * 2;
  const int ret = libusb_control_transfer( dev, 0x40, 0x31, 
					   (uint16_t)angle, 0x0, empty, 0x0, 0 );
  if ( ret != 0 ) {
    fprintf( stderr, "Error: Error in setting tilt angle, libusb_control_transfer returned %d\n\n", ret );
    exit( EXIT_FAILURE );
  }
}

/**
 * setTiltAngle
 * @param angle
 */
void setLedOption( const uint16_t option ) {
  uint8_t empty[0x1];
  
  const int ret = libusb_control_transfer( dev, 0x40, 0x06, 
					   (uint16_t) option, 0x0, empty, 0x0, 0 );
  if ( ret != 0 ) {
    fprintf( stderr, "Error: Error in setting LED options, "
	     "libusb_control_transfer returned %d\n\n", ret );
    exit( EXIT_FAILURE );
  }
  
}

#endif /* KINECT_AUX_LIB_H__ */
