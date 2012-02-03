/*
 * groundtruth.cpp
 *
 *  Created on: Jan 23, 2012
 *      Author: engelhar
 */

#include "groundtruth.h"
#include <visualization_msgs/MarkerArray.h>
#include <fstream>



bool Groundtruth::getNextSimInstance(Mocap_object& obj){
  if (file_idx>= object_instances.size())
    return false;
  Mocap_object o = object_instances[file_idx];

  ROS_INFO("valid points: %i", o.valid_point_cnt);

  if (o.gt_trafo_valid){
    obj = bag_sim_object;
    obj.moveObject(o.gt_trafo);
  }else
  {
    obj.points.clear();
    obj.valid_point_cnt = 0;
  }

  file_idx+=skip;
  return true;

}


bool Groundtruth::getNextInstance(Mocap_object& obj){
   if (file_idx>= object_instances.size())
     return false;
   obj = object_instances[file_idx];

   file_idx+=skip;
   return true;
}

void Groundtruth::computePoses(){
  assert(model_loaded);

//  float residual;

//  ofstream off("/home/engelhar/Desktop/residual.txt");

  int valid_cnt = 0;

  for (uint i=0; i<object_instances.size(); ++i){
    Mocap_object* mo = &object_instances[i];
    mo->gt_trafo_valid = bag_base_object.getTrafoTo(*mo, mo->gt_trafo);//, &residual);
//    off << residual << endl;
    if (mo->gt_trafo_valid) valid_cnt++;
  }

  ROS_INFO("Valid after residual check %i", valid_cnt);

//  off.close();

}

bool Groundtruth::loadBag(const char* filename){

  assert(model_loaded);

  rosbag::Bag bag;
  std::vector<std::string> topics;

  bag.open(filename, rosbag::bagmode::Read);
  topics.push_back(std::string("/cortex_marker_array"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  object_instances.reserve(50000);

  partial_msg_cnt = 0;
  invalid_msg_cnt = 0;
  complete_msg_cnt = 0;


//


  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    visualization_msgs::MarkerArray::ConstPtr markers = m.instantiate<visualization_msgs::MarkerArray>();

    uint s = uint( markers->markers[0].points.size() );
    assert(s == bag_base_object.points.size());

    Mocap_object mo;
    mo.points.resize(s);
    mo.point_valid.resize(s);
    mo.valid_point_cnt = 0;
    mo.stamp =  markers->markers[0].header.stamp;

    for (uint i=0; i<s; ++i){
      float x = markers->markers[0].points[i].x;
      float y = markers->markers[0].points[i].y;
      float z = markers->markers[0].points[i].z;

      mo.point_valid[i] = (x==x);

//      off << i << "      " << x << " " << y << " " << z << endl;

      if (mo.point_valid[i]){
        mo.valid_point_cnt++;
        Eigen::Vector3f p(x,y,z);
        mo.points[i] = p;
      }
    }

    if (mo.valid_point_cnt == s)
      complete_msg_cnt++;
    if (mo.valid_point_cnt >= 3 && mo.valid_point_cnt < s )
      partial_msg_cnt++;
    if (mo.valid_point_cnt < 3)
      invalid_msg_cnt++;

    object_instances.push_back(mo);
  }

  uint total_msg_cnt = complete_msg_cnt+partial_msg_cnt+invalid_msg_cnt;

  ROS_INFO("Read %i msgs from %s",int(total_msg_cnt), filename );
  ROS_INFO("complete: %i, partial: %i, invalid: %i", int(complete_msg_cnt), int(partial_msg_cnt), int(invalid_msg_cnt));

  assert(total_msg_cnt == object_instances.size());


//  off.close();

  computePoses();

  file_idx = 0;

  return true;
}

void Groundtruth::readBagPropFile(const char* filename){
  bag_base_object = Mocap_object();
  readPropFile(filename, bag_base_object);
  model_loaded = true;
}

void Groundtruth::readSimPropFile(const char* filename){
  bag_sim_object = Mocap_object();
  readPropFile(filename, bag_sim_object);
  sim_model_loaded = true;
}

void Groundtruth::readPropFile(const char* filename, Mocap_object& mo){
  ifstream iff; iff.open(filename);
  uint vertex_cnt; iff >> vertex_cnt;

  float x,y,z; char c;
  for (uint i=0; i<vertex_cnt; ++i){
    iff >> x >> c >> y >> c >> z; // seperated by a comma
    mo.addVertex(x/1000,y/1000,z/1000);// values in mm, but rest in m
    //    printf("v %i: %f %f %f\n", i,x/1000,y/1000,z/1000);
  }
  ROS_INFO("Read object with %i vertices from %s",int(vertex_cnt), filename);
}
