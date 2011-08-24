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
  if(!initKinectAux()) {
    return EXIT_FAILURE;
  }

  /* check whether tilted */
  /*
  int tilted;
  while(1) {
    tilted = tiltedTo();
    if((tilted != TURNING) && (tilted != FAILURE)) {
      printf("%s\n", (tilted == -1) ? "Tilted left" : ((tilted == 1) ? "Tilted right" : "Not tilted"));
    } else if(tilted == TURNING) {
      printf("Currently turning\n");
    } else if(tilted == FAILURE) {
      printf("Not initialized\n");
    }
    usleep( 500000 );
  }
  */

  /* print current status */
  /*
  while(1) {
    printCurrentState();
    usleep( 500000 );
  }
  */

  /* set automatic level */
  /* while(autoLevel()) {}; */

  cleanKinectAux();
  return EXIT_SUCCESS;
}
