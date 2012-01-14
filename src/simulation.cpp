/*
 * simulation.cpp
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */


#include "simulation.h"

void createRect(Mocap_object* mo,   float z, float w, float h, float x_c, float y_c){
  mo->vertices.clear();
  mo->addVertex(x_c-w/2,y_c-h/2,z);
  mo->addVertex(x_c-w/2,y_c+h/2,z);
  mo->addVertex(x_c+w/2,y_c-h/2,z);
  mo->addVertex(x_c-w/2,y_c-h/2,z);

  mo->printObject();
}
