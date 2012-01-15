/*
 * mocap_defs.h
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */

#ifndef MOCAP_DEFS_H_
#define MOCAP_DEFS_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>



#include <g2o/types/slam3d/vertex_trackxyz.h>
#include <g2o/types/slam3d/edge_trackxyz_trackxyz.h>
#include <g2o/types/slam3d/edge_project.h>
#include <cmath>
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;


struct Mocap_object {

  point_cloud cloud;
  vector<g2o::VertexTrackXYZ*> vertices;


  void addVertex(float x, float y, float z){
    point_type p;
    p.x = x; p.y=y; p.z=z;
    cloud.points.push_back(p);
  }



  void printObject(){

    for (uint i=0; i<cloud.points.size(); ++i){
      point_type c = cloud.points[i];
      printf("v %i: %f %f %f \n", i, c.x,c.y, c.z);
    }

    for (uint i=0; i<cloud.points.size()-1; ++i){
      point_type v = cloud.points[i];
      for (uint j=i+1; j<cloud.points.size(); ++j){
        point_type c = cloud.points[j];
        printf("rel (%i,%i): %f %f %f\n", i,j, c.x-v.x,c.y-v.y,c.z-v.y);
      }
    }

  }


  void getPoseFromVertices();

  // compares all pairwise distances. objects are considered the same if no
  // dist is larger than max_dist
  // assumes that object is defined by its distances
  bool isSameObject(Mocap_object& other, float dist_thres, float* max_dist = NULL);




};



#endif /* MOCAP_DEFS_H_ */
