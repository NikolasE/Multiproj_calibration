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

  Groundtruth(){model_loaded = false; sim_model_loaded = false;};
  bool loadBag(const char* filename);


  void readSimPropFile(const char* filename);
  void readBagPropFile(const char* filename);
  bool getNextInstance(Mocap_object& obj);
  bool getNextSimInstance(Mocap_object& obj);



  void computePoses();

  uint getInstanceCound() { return object_instances.size();}
  uint getFilePosition() { return file_idx;}
  uint skip;
  uint file_idx; // should be private
  Mocap_object bag_base_object;
  Mocap_object bag_sim_object;

private:



  bool model_loaded;
  bool sim_model_loaded;


  void readPropFile(const char* filename, Mocap_object& mo);
  vector<Mocap_object> object_instances;
  vector<Eigen::Affine3f> poses;

  uint partial_msg_cnt;
  uint invalid_msg_cnt;
  uint complete_msg_cnt;




};






#endif /* GROUNDTRUTH_H_ */
