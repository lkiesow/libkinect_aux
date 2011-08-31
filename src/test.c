/*******************************************************************************
 *
 *       Filename:  test.c
 *
 *    Description:  Makes use of the kinect auxiliary library
 *
 *        Created:  08/24/2011 12:30:00 PM
 *       Compiler:  gcc
 *
 *         Author:  Denis Meyer (denmeyer), denmeyer@uos.de
 *                  Lars KIesow (lkiesow), lkiesow@uos.de
 *        Company:  Universität Osnabrück
 *
 ******************************************************************************/

#include "kinect_aux.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

void * continuous_autolevel( void * ptr ) {
  while ( 1 ) {
    kinectaux_autolevel();
    usleep( 10 );
  }
  return NULL; 
}

/* keyboard hit */
int kbhit(void) {
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}

int main( int argc, char ** argv ) {
  if( !kinectaux_init( 0 ) ) {
    return EXIT_FAILURE;
  }
  
  pthread_t t;
  pthread_create( &t, NULL, continuous_autolevel, NULL );
  
  int ch = 0;

  /*
  printf("Data\nPress <Esc> to continue...\n");
  while(ch != 27) {
    kinectaux_data d = kinectaux_get_data();
    printf( "%d %u %f\n", d.raw.acc.x, (uint16_t) d.raw.acc.x, d.conv.acc.x );
    usleep( 10000 );
    if(kbhit()) {
      ch=getchar();
    }
  }

  ch = 0;
  */

  printf("Rotation\nPress <Esc> to quit...\n");
  while(ch != 27) {
    printf( "Your kinect is rotated %f degrees sideways.\n", kinectaux_lateral_tilt() );
    usleep( 10000 );
    if(kbhit()) {
      ch=getchar();
    }
  }
  printf("\n");
  
  kinectaux_exit();
  return EXIT_SUCCESS;
}
