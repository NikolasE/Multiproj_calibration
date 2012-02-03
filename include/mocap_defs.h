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

using namespace std;

typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

typedef map<int, CvPoint2D32f> Observations;


void affine3fToXyzRpy(Eigen::Affine3f t, float* trafo);
void xyzRpyToAffine3f(float* trafo, Eigen::Affine3f& t);


struct Camera {
  uint id;
  Eigen::Affine3f pose;
  g2o::VertexSE3* vertex;
  Observations obs;

  Camera() {pose = Eigen::Affine3f::Identity(); }

  static const float c_width = 640;
  static const float c_height = 480;


  static const float c_x = 319.5;
  static const float c_y = 239.5;
  static const float f_x = 525;
  static const float f_y = 525;

};

struct Mocap_object {


  ros::Time stamp;
  vector<Eigen::Vector3f> points;
  vector<bool> point_valid;
  uint valid_point_cnt; // number of trues in point_valid

  Eigen::Affine3f gt_trafo;
  bool gt_trafo_valid;

  void reset();

//   point_cloud cloud;
  vector<g2o::VertexTrackXYZ*> vertices;

  void addVertex(float x, float y, float z){
    points.push_back(Eigen::Vector3f(x,y,z));
    point_valid.push_back(true);
    valid_point_cnt++;
  }

  void printObject(){

    for (uint i=0; i<points.size(); ++i){
      Eigen::Vector3f c = points[i];
      printf("v %i: %f %f %f \n", i, c.x(),c.y(), c.z());
    }

    for (uint i=0; i<points.size()-1; ++i){
      Eigen::Vector3f v = points[i];
      for (uint j=i+1; j<points.size(); ++j){
        Eigen::Vector3f c = points[j]-v;
        printf("rel (%i,%i): %f %f %f\n", i,j, c.x(),c.y(),c.z());
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
  bool getTrafoTo(Mocap_object& other, Eigen::Affine3f& t, float* residual = NULL, float* trafo = NULL);

  // inverse of trafo is applied to other-object (if not NULL)
  double max_point_dist(Mocap_object& other, Eigen::Affine3f* trafo = NULL);

  void moveObject(Eigen::Affine3f& trafo);

private:
  // compare eval/residual_mean.txt
  // set to -1 to deactivate
  static const float threshold_max_residual = 0.4/100;// 0.4cm

};



#endif /* MOCAP_DEFS_H_ */
