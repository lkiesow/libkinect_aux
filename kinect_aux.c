/*******************************************************************************
 *
 *       Filename:  kinect_aux.cpp
 *
 *    Description:  Control Kinect motors and read acceleration data.
 *                  Parts taken from ROS and libfreenect
 *
 *        Version:  1.0
 *        Created:  06/23/2011 03:22:43 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lars Kiesow (lkiesow), lkiesow@uos.de
 *        Company:  Universität Osnabrück
 *
 ******************************************************************************/

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
 

libusb_device_handle * dev = 0;


void openAuxDevice( int index ) {

	/* pointer to pointer of device, used to retrieve a list of devices */
	libusb_device ** devs;
	/* get the list of devices */
	int cnt = libusb_get_device_list( 0, &devs );
	if (cnt < 0) {
		fprintf( stderr, "No device on USB\n" );
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
					fprintf( stderr, "Cannot open aux %d.\n", index );
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


kinect_aux_data readState() {
	uint8_t buf[10];
	const int ret = libusb_control_transfer( dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0 );
	if ( ret != 10 ) {
		fprintf( stderr, "Error in accelerometer reading, "
				"libusb_control_transfer returned %d", ret );
		exit( EXIT_FAILURE );
	}
	
	kinect_aux_data dat;
	dat.raw.acc.x       = (int16_t) (((uint16_t)buf[2] << 8) | buf[3]);
	dat.raw.acc.y       = (int16_t) (((uint16_t)buf[4] << 8) | buf[5]);
	dat.raw.acc.z       = (int16_t) (((uint16_t)buf[6] << 8) | buf[7]);
	dat.raw.tilt.angle  = (int8_t) buf[8];
	dat.raw.tilt.status = buf[9];
	
	fprintf( stderr, "%d %d\n", buf[8], (int8_t) buf[8] );
	dat.conv.acc.x      = (double) dat.raw.acc.x * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.acc.y      = (double) dat.raw.acc.y * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.acc.z      = (double) dat.raw.acc.z * GRAVITY / FREENECT_COUNTS_PER_G;
	dat.conv.tilt.angle = (double) dat.raw.tilt.angle / 2.;
	dat.conv.tilt.status = dat.raw.tilt.status;

	return dat;
}


void setTiltAngle( double angle ) {
	uint8_t empty[0x1];

	angle = (angle<MIN_TILT_ANGLE) 
		? MIN_TILT_ANGLE 
		: ((angle>MAX_TILT_ANGLE) 
				? MAX_TILT_ANGLE 
				: angle);
	angle = angle * 2;
	const int ret = libusb_control_transfer( dev, 0x40, 0x31, 
			(uint16_t)angle, 0x0, empty, 0x0, 0 );
	if ( ret != 0 ) {
		fprintf( stderr, "Error in setting tilt angle, "
				"libusb_control_transfer returned %d\n", ret );
		exit( EXIT_FAILURE );
	}
}

void setLedOption( const uint16_t option ) {
	uint8_t empty[0x1];
	
	const int ret = libusb_control_transfer( dev, 0x40, 0x06, 
			(uint16_t) option, 0x0, empty, 0x0, 0 );
	if ( ret != 0 ) {
		fprintf( stderr, "Error in setting LED options, "
				"libusb_control_transfer returned %d\n", ret );
		exit( EXIT_FAILURE );
	}

}


int main( int argc, char ** argv ) {

	int i, ret = libusb_init( 0 );
	if (ret) {
		fprintf( stderr, "Cannot initialize libusb, error: %d\n", ret );
		return 1;
	}
	
	/* Device to open (0 = open first kinect) */
	int deviceIndex = 0;
	openAuxDevice( deviceIndex );
	if ( !dev ) {
		fprintf( stderr, "No valid aux device found" );
		libusb_exit(0);
		return 2;
	}

	kinect_aux_data dat;

	/* Step 1: Set Tiltangle to 0 and wait while kinect is rotating */
	setTiltAngle( 0.0 );
	/* Wait to ensure that the rotation has started */
	usleep( 100 );
	do {
		dat = readState();
		usleep( 10 );
	/* Wait until the operation is finished */
	} while ( dat.raw.tilt.status != 0 );

	/* Step 2: Determine current accerleration values */
	int64_t med_raw_accz = 0;
	double  med_angle = 0;
	for ( i = 0; i < 100; i++ ) {
		dat = readState();
		med_raw_accz += dat.raw.acc.z;
	}
	med_raw_accz /= 100;

	/* Print state for show */
	dat = readState();
	printf( "%3u % 3d % 12f % 5d % 5d % 5d % 12f % 12f % 12f\n",
			dat.raw.tilt.status, dat.raw.tilt.angle, dat.conv.tilt.angle,
			dat.raw.acc.x, dat.raw.acc.y, dat.raw.acc.z,
			dat.conv.acc.x, dat.conv.acc.y, dat.conv.acc.z );

	/* Step 3: Estimate desired angle for zero gravity, apply it and wait till
	 * rot is done. */
	printf( "Estimated angle: %f\n", med_angle - ( med_raw_accz * 0.3 / 4.0 ) );
	setTiltAngle( med_angle - ( med_raw_accz * 0.3 / 4.0 ) );
	usleep( 100 );
	do {
		dat = readState();
		usleep( 10 );
	} while ( dat.raw.tilt.status != 0 );

	/* Print last state for show */
	printf( "%3u % 3d % 12f % 5d % 5d % 5d % 12f % 12f % 12f\n",
			dat.raw.tilt.status, dat.raw.tilt.angle, dat.conv.tilt.angle,
			dat.raw.acc.x, dat.raw.acc.y, dat.raw.acc.z,
			dat.conv.acc.x, dat.conv.acc.y, dat.conv.acc.z );

	for ( i = 0; i < 999; i++ ) {
		dat = readState();
		printf( "%3u % 3d % 12f % 5d % 5d % 5d % 12f % 12f % 12f\n",
				dat.raw.tilt.status, dat.raw.tilt.angle, dat.conv.tilt.angle,
				dat.raw.acc.x, dat.raw.acc.y, dat.raw.acc.z,
				dat.conv.acc.x, dat.conv.acc.y, dat.conv.acc.z );

	}
	
	libusb_exit(0);
	return 0;
}
