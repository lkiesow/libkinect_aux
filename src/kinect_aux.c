/*******************************************************************************
 *
 *       Filename:  kinect_aux.h
 *
 *    Description:  Control Kinect motors and read acceleration data.
 *                  Parts taken from ROS and libfreenect
 *
 *        Version:  10827
 *        Created:  06/23/2011 03:22:43 PM
 *       Modified:  8/27/11 18:57:37
 *       Compiler:  gcc
 *
 *         Author:  Lars Kiesow (lkiesow), lkiesow@uos.de
 *         Author:  Denis Meyer (denmeyer), denmeyer@uos.de
 *        Company:  Universität Osnabrück
 *
 ******************************************************************************/

#include "kinect_aux.h"
#include <libusb.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

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

libusb_device_handle * kinectaux_dev = 0;


/*******************************************************************************
 * Establishes connection to Kinect with given id. Automatically search for the
 * correct device.
 *
 * @param index  Index of Kinect to use. Zero is the first Kinect connected,
 *               one the second, ...
 ******************************************************************************/
void kinectaux_open_device( int index ) {

	/* pointer to pointer of device, used to retrieve a list of devices */
	libusb_device ** devs;
	/* get the list of devices */
	int cnt = libusb_get_device_list( 0, &devs );
	if ( cnt < 0 ) {
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
				if ( ( libusb_open( devs[i], &kinectaux_dev ) != 0 ) || ( !kinectaux_dev  ) ) {
					fprintf( stderr, "Error: Cannot open aux %d.\n\n", index );
					return;
				}
				// Claim the aux
				libusb_claim_interface( kinectaux_dev, 0 );
				break;
			} else {
				nr_mot++;
			}
		}
	}

	/* free the list, unref the devices in it */
	libusb_free_device_list( devs, 1 );
}


/*******************************************************************************
 * Read data from the Kinect.
 *
 * @return  A kinectaux_data struct containing the current state of the Kinect.
 ******************************************************************************/
kinectaux_data kinectaux_get_data() {
	uint8_t buf[10];
	const int ret = libusb_control_transfer( 
			kinectaux_dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0 );
	if ( ret != 10 ) {
		fprintf( stderr, "Error: Error in accelerometer reading, "
				"libusb_control_transfer returned %d\n\n", ret );
		exit( EXIT_FAILURE );
	}

	kinectaux_data dat;
	dat.raw.acc.x       = (int16_t) (((uint16_t)buf[2] << 8) | buf[3]);
	dat.raw.acc.y       = (int16_t) (((uint16_t)buf[4] << 8) | buf[5]);
	dat.raw.acc.z       = (int16_t) (((uint16_t)buf[6] << 8) | buf[7]);
	dat.raw.tilt.angle  = (int8_t) buf[8];
	dat.raw.tilt.status = buf[9];

	dat.conv.acc.x      = (double) dat.raw.acc.x * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.acc.y      = (double) dat.raw.acc.y * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.acc.z      = (double) dat.raw.acc.z * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.tilt.angle = (double) dat.raw.tilt.angle / 2.;
	dat.conv.tilt.status = dat.raw.tilt.status;

	return dat;
}


/**
 * Set the tilt angle of the Kinect. This action is nonblocking. 
 *
 * @param angle  The angle the Kinect should tilt to.
 * @return  Successful or not.
 */
int kinectaux_set_tilt( double angle ) {
	if ( !kinectaux_dev ) {
		return 0;
	}
	uint8_t empty[0x1];

	angle = (angle < MIN_TILT_ANGLE) 
		? MIN_TILT_ANGLE 
		: ((angle > MAX_TILT_ANGLE) 
				? MAX_TILT_ANGLE 
				: angle);
	angle = angle * 2;
	const int ret = libusb_control_transfer( kinectaux_dev, 0x40, 0x31, 
			(uint16_t) angle, 0x0, empty, 0x0, 0 );
	if ( ret ) {
		fprintf( stderr, "Error: Error in setting tilt angle, "
				"libusb_control_transfer returned %d\n\n", ret );
		return 0;
	}
	return 1;
}

/**
 * Set the leds of the Kinect.
 *
 * @param option  Unsigned 16-bit integer as flag for the leds.
 * @return  Successful or not.
 **/
int kinectaux_set_led( const uint16_t option ) {
	uint8_t empty[0x1];

	const int ret = libusb_control_transfer( kinectaux_dev, 0x40, 0x06, 
			(uint16_t) option, 0x0, empty, 0x0, 0 );
	if ( ret ) {
		fprintf( stderr, "Error: Error in setting LED options, "
				"libusb_control_transfer returned %d\n\n", ret );
		return 0;
	}
	return 1;

}


/**
 * Initialized libusb, get device descriptor and prepare library. This function
 * must be called before using any other functions of the library.
 *
 * @param idx  Index of the Kinect to use.
 * @return  Successful or not.
 **/
int kinectaux_init( int idx ) {

	if ( kinectaux_dev ) {
		libusb_exit( NULL );
	}

	/* initialize libusb */
	if ( libusb_init( NULL ) ) {
		fprintf( stderr, "Error: Cannot initialize libusb.\n" );
		kinectaux_dev = NULL;
		return 0;
	}

	/* initialize kinect device */
	kinectaux_open_device( idx );
	if ( !kinectaux_dev ) {
		libusb_exit( NULL );
		fprintf( stderr, "Error: No valid aux device found on device index %d\n\n", idx );
		return 0;
	}

	return 1;
}


/**
 * Closes the connection to the Kinect.
 **/
void kinectaux_exit() {
	libusb_exit(0);
	kinectaux_dev = NULL;
}


/**
 * Automatically levels the Kinect. This function is blocking. After calling
 * this function you can be sure that the Kinect is level unless the function
 * exited with a failure state.
 *
 * @return  If successful or not.
 **/
int kinectaux_autolevel() {
	if ( !kinectaux_dev ) {
		return 0;
	}
	/* set tiltangle to 0 */
	kinectaux_set_tilt( 0.0 );
	/* wait to ensure that the rotation has started */
	do {
		usleep( 100 );
		/* wait until the operation is finished */
	} while ( kinectaux_get_data().raw.tilt.status );
	return 1;
}


/**
 * Returns the angle (degrees) the Kinect is tilted sideways.
 *
 * @return  Tilt angle of the Kinect.
 **/
double kinectaux_lateral_tilt() {

	if ( !kinectaux_dev ) {
		return HUGE_VAL;
	}
	double accx = kinectaux_get_data().conv.acc.x / GRAVITY;
	if ( accx > 1 ) {
		accx = 1.0;
	}
	return asin( accx ) * 180 / M_PI;

}
