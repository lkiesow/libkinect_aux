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

#ifndef KINECT_AUX_H_INCLUDED
#define KINECT_AUX_H_INCLUDED

#include <stdint.h>

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
} kinectaux_data;


/**
 * openAuxDevice
 * @param index
 */
void kinectaux_open_device( int index );

/**
 * readState
 * @return kinect_aux_data
 */
kinectaux_data kinectaux_get_data();

/**
 * setTiltAngle
 * @param angle
 */
int kinectaux_set_tilt( double angle );

/**
 * setTiltAngle
 * @param angle
 */
int kinectaux_set_led( const uint16_t option );

/**
 * initialize
 * @return 1 if successfully initialized, 0 else
 */
int kinectaux_init( int idx );

/**
 * clean
 *   - exit libusb
 */
void kinectaux_exit();

/**
 * automatically levels the kinect
 * @return 1 if successfully leveled, 0 else
 */
int kinectaux_autolevel();

/**
 * checks whether the kinect is currently tilted to the left or the right side
 * @return - TURNING if device is currently turning and no valid data came back
 *         - FAILURE when not initialized
 *         - -1 when tilted to the left side
 *         - 1 when tilted to the right side
 *         - 0 when not tilted
 */
double kinectaux_lateral_tilt();

#endif /* KINECT_AUX_H_INCLUDED */
