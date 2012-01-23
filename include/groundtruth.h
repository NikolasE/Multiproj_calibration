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

  Groundtruth(){};
  bool openBag(char* filename, ros::NodeHandle* nh_);

  void readPropFile(char* filename);

  Mocap_object getNextPose(ros::Time& timestamp);


  Mocap_object base_object;

private:
  ros::NodeHandle* nh;
  rosbag::Bag bag;
  std::vector<std::string> topics;
  rosbag::View* view;



};






#endif /* GROUNDTRUTH_H_ */
