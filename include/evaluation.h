/*
 * evaluation.h
 *
 *  This file contains some functions to evaluate the performance of the g2o-Mocap.
 *  This includes dependency from starting pose, number of iterations, ...
 *
 *
 *
 *  Created on: Jan 16, 2012
 *      Author: engelhar
 */

#ifndef EVALUATION_H_
#define EVALUATION_H_

#include "mocap_defs.h"
#include "optimization.h"
#include "simulation.h"
#include <fstream>


void checkStartingPose();
bool isSimilar(Eigen::Affine3f& a, Eigen::Affine3f& b, float max_d, float max_rad, float* error = NULL);








#endif /* EVALUATION_H_ */
