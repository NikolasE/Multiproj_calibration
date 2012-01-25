/*
 * groundtruth.h
 *
 *  Creates an interface to the Motion Analysis Mocap
 *
 *
 *  Created on: Jan 23, 2012
 *      Author: engelhar
 *
 *
 *
 */

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_

#include <ros/ros.h>
#include "mocap_defs.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>


struct Groundtruth {

  Groundtruth(){model_loaded = false;};
  bool loadBag(char* filename);

  void readPropFile(char* filename);

//  Mocap_object getNextPose(ros::Time& timestamp);

  bool model_loaded;
  Mocap_object base_object;
  void computePoses();

private:

  vector<Mocap_object> object_instances;
  vector<Eigen::Affine3f> poses;

  uint partial_msg_cnt;
  uint invalid_msg_cnt;
  uint complete_msg_cnt;




};






#endif /* GROUNDTRUTH_H_ */
