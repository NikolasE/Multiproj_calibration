/*
 * groundtruth.cpp
 *
 *  Created on: Jan 23, 2012
 *      Author: engelhar
 */

#include "groundtruth.h"
#include <visualization_msgs/MarkerArray.h>
#include <fstream>




Mocap_object Groundtruth::getNextPose(ros::Time& timestamp){

  BOOST_FOREACH(rosbag::MessageInstance const m, *view)
  {


    visualization_msgs::MarkerArray::ConstPtr markers = m.instantiate<visualization_msgs::MarkerArray>();


    uint s = uint( markers->markers[0].points.size() );
    printf("size: %i\n", int( s ));
    for (uint i=0; i<s; ++i);
    return Mocap_object();
  }
  return Mocap_object();
   // only one message
}

bool Groundtruth::openBag(char* filename, ros::NodeHandle* nh_){

  bag.open(filename, rosbag::bagmode::Read);
  topics.clear();
  topics.push_back(std::string("/cortex_marker_array"));
  view = new rosbag::View(bag, rosbag::TopicQuery(topics));
  nh = nh_;

  return true;
}


void Groundtruth::readPropFile(char* filename){
  ifstream iff; iff.open(filename);
  uint vertex_cnt; iff >> vertex_cnt;
  base_object = Mocap_object();
  float x,y,z; char c;
  for (uint i=0; i<vertex_cnt; ++i){
    iff >> x >> c >> y >> c >> z; // seperated by a comma
    base_object.addVertex(x/1000,y/1000,z/1000);// values in mm, but rest in m
//    printf("v %i: %f %f %f\n", i,x/1000,y/1000,z/1000);
  }
  ROS_INFO("Read object with %i vertices from %s",int(vertex_cnt), filename);
}
