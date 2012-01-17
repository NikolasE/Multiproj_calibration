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
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Dense>



#include <g2o/types/slam3d/vertex_trackxyz.h>
#include <g2o/types/slam3d/edge_trackxyz_trackxyz.h>
#include <g2o/types/slam3d/edge_project.h>
#include <cmath>

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

typedef map<int, CvPoint2D32f> Observations;


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

  // reads estimate pose from the g2o-vertices back into the cloud
  // has to be called after optimization before the comparison function can be called
  void getPoseFromVertices();

  // compares all pairwise distances. objects are considered the same if no
  // dist is larger than max_dist
  // assumes that object is defined by its distances
  bool isSameObject(Mocap_object& other, float dist_thres, float* max_dist = NULL);



  // trafo should be array of length 6 with (x,y,z,phi, theta, psi) (in rad)
  bool getTrafoTo(Mocap_object& other,Eigen::Affine3f& t, float* trafo = NULL);

  // inverse of trafo is applied to other-object (if not NULL)
  double max_point_dist(Mocap_object& other, Eigen::Affine3f* trafo = NULL);

};



#endif /* MOCAP_DEFS_H_ */
