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

#endif /* SIMULATION_H_ */
