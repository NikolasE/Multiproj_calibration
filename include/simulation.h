/*
 * simulation.h
 *
 *  This file contains simulation methods for the mocap with g2o.
 *
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "mocap_defs.h"

// create rectangle in x-y-Plane centered around (x_c,y_c,z) with given width and height
void createRect(Mocap_object* mo,float z, float w, float h, float x_c=0, float y_c =0);


class Simulator {

  static const float c_x = 319.5;
  static const float c_y = 239.5;
  static const float f_x = 525;
  static const float f_y = 525;

  static const float c_width = 640;
  static const float c_height = 480;


public:
  IplImage* img;

  Simulator(){
    img = cvCreateImage(cvSize(640,480), IPL_DEPTH_32F,3);
  }



  void printProjections(Mocap_object* mo);

};


#endif /* SIMULATION_H_ */
