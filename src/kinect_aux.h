/**
 * @file       kinect_aux.h
 * @brief      Library which provides easy acces to Kinect accelerator, leds
 *             and motor.
 * @details    Library which provides easy acces to the Kinect accelerator,
 *             leds and motor. Furthermore it contains routines for automatic
 *             leveling of the kinect. Parts of this code were taken from ROS
 *             and libfreenect.
 * @author     Lars Kiesow (lkiesow), lkiesow@uos.de
 * @author     Denis Meyer (denmeyer), denmeyer@uos.de
 * @version    110827
 * @date       Created:       2011-08-20 17:28:28
 * @date       Last modified: 8/27/11 20:05:48
 *
 */

#ifndef KINECT_AUX_H_INCLUDED
#define KINECT_AUX_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
/* only need to export C interface i used by C++ source code */
extern "C" {
#endif

/**
 * @brief Struct to store the current state of the Kinect.
 **/
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
 * Initialized libusb, get device descriptor and prepare library. This function
 * must be called before using any other functions of the library.
 * @brief Initializes library.
 *
 * @param idx  Index of the Kinect to use.
 * @return  Successful or not.
 **/
int kinectaux_init( int idx );

/**
 * Closes the connection to the Kinect.
 * @brief Deinitialize library.
 **/
void kinectaux_exit();

/******************* INTERNAL FUNCTION
 * Establishes connection to Kinect with given id. Automatically search for the
 * correct device.
 *
 * @param index  Index of Kinect to use. Zero is the first Kinect connected,
 *               one the second, ...
 **/
/* void kinectaux_open_device( int index ); */

/**
 * Read data from the Kinect.
 * @brief Get data from Kinect.
 *
 * @return  A kinectaux_data struct containing the current state of the Kinect.
 **/
kinectaux_data kinectaux_get_data();

/**
 * Set the tilt angle of the Kinect. This action is nonblocking. 
 * @brief Set tilt angle of the Kinect.
 *
 * @param angle  The angle the Kinect should tilt to.
 * @return  Successful or not.
 */
int kinectaux_set_tilt( double angle );

/**
 * Set the leds of the Kinect.
 * @brief Set the leds.
 *
 * @param option  Unsigned 16-bit integer as flag for the leds.
 * @return  Successful or not.
 **/
int kinectaux_set_led( const uint16_t option );

/**
 * Automatically levels the Kinect. This function is blocking. After calling
 * this function you can be sure that the Kinect is level unless the function
 * exited with a failure state.
 * @brief Automatically levels the Kinect.
 *
 * @return  If successful or not.
 **/
int kinectaux_autolevel();

/**
 * Returns the angle (degrees) the Kinect is tilted sideways.
 * @brief Get sideways tilt angle.
 *
 * @return  Tilt angle of the Kinect.
 **/
double kinectaux_lateral_tilt();

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* KINECT_AUX_H_INCLUDED */
