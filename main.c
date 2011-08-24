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

int main( int argc, char ** argv ) {
  while(autoLevel()) {};
  return EXIT_SUCCESS;
}
