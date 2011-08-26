/*******************************************************************************
 *
 *       Filename:  kinect_aux.h
 *
 *    Description:  Makes use of the kinect auxiliary library
 *
 *        Version:  1.0
 *        Created:  08/24/2011 12:30:00 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Denis Meyer (denmeyer), denmeyer@uos.de
 *        Company:  Universität Osnabrück
 *
 ******************************************************************************/

#include "kinect_aux.h"
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char ** argv ) {
	if( !kinectaux_init( 0 ) ) {
		return EXIT_FAILURE;
	}

	while ( 1 ) {
		kinectaux_autolevel();
		usleep( 10 );
	}

	int i;
	for ( i = 0; i < 10000; i++ ) {
		kinectaux_data d = kinectaux_get_data();
		printf( "%d %u %f\n", d.raw.acc.x, (uint16_t) d.raw.acc.x, d.conv.acc.x );
		usleep( 10000 );
	}

	for ( i = 0; i < 10000; i++ ) {
		printf( "Your kinect is rotated %f degrees sideways.\n", kinectaux_lateral_tilt() );
		usleep( 10000 );
	}

	kinectaux_exit();
	return EXIT_SUCCESS;
}
