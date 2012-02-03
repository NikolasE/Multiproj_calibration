

#include "mocap_defs.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/transform.h>

using namespace Eigen;

void Mocap_object::getPoseFromVertices(){
  assert(vertices.size() == points.size());
  for (uint i=0; i<points.size(); ++i){
    points[i] = vertices[i]->estimate().cast<float>();
  }
}


void Mocap_object::moveObject(Eigen::Affine3f& trafo){
  for (uint i=0; i<points.size(); ++i)
    if (point_valid[i])
      points[i] = trafo*points[i];
}

bool Mocap_object::isSameObject(Mocap_object& other, float dist_thres, float* max_dist){

  if (points.size() != other.points.size()) {
    ROS_WARN("isSameObject: %i != %i", (int)points.size(), (int)other.points.size() );
    return false;
  }

  if (max_dist != NULL) *max_dist = -1;

  uint n = points.size();
  for (uint i=0; i<n-1; ++i){
    Vector3f v = points[i];
    Vector3f v_o = other.points[i];

    if (!point_valid[i] || ! other.point_valid[i])
      continue;

    for (uint j=i+1; j<n; ++j){
      if (!point_valid[j] || ! other.point_valid[j])
        continue;

      Vector3f c = points[j];
      Vector3f c_o = other.points[j];

      double d1 = (c-v).norm();
      double d2 = (c_o-v_o).norm();

      double diff = fabs(d1-d2);

      if (max_dist != NULL && *max_dist<diff)
        *max_dist = diff;

      if (diff>dist_thres)
        return false;
    }
  }

  return true;

}


void affine3fToXyzRpy(Eigen::Affine3f t, float* trafo){
  pcl::getTranslationAndEulerAngles(t, trafo[0],trafo[1],trafo[2],trafo[3],trafo[4],trafo[5]);
}

void xyzRpyToAffine3f(float* trafo, Eigen::Affine3f& t){
  pcl::getTransformation(trafo[0],trafo[1],trafo[2],trafo[3],trafo[4],trafo[5],t);
}

bool Mocap_object::getTrafoTo(Mocap_object& other,Eigen::Affine3f& t,float* residual,  float* trafo){

  if (!isSameObject(other, 0.1)){
    cout << "#### getTrafoTo: not the same object" << endl;
    return false;
  }

  pcl::TransformationFromCorrespondences tfc;

  //  if (valid_point_cnt < 4 || other.valid_point_cnt < 4){
  //    return false;
  //  }

  // get mean
  uint match_cnt = 0;
  for (uint i=0; i<points.size(); ++i){

    //  for (Observations::iterator it = )

    if (!point_valid[i] || !other.point_valid[i])
      continue;

    //    if (!other.point_valid[i]){
    ////      ROS_INFO("point %i invalid", i);
    //      continue;
    //    }

    //    cout << "i: " << i << endl;
    //    cout << points[i] << endl;
    //    cout << other.points[i] << endl;

    tfc.add(points[i],other.points[i]);
    match_cnt++;
  }


  // since the MoCap confuses some points, only full measurements are used
  // ROS_INFO("matches: %i", match_cnt);
  if (match_cnt < 3)//points.size())
    return false;

  t = tfc.getTransformation();



  if (trafo!=NULL) affine3fToXyzRpy(t,trafo);

  if (threshold_max_residual>0 || residual != NULL){

    // get sum of distances from moved object to original

    double max_err = 0;
    float res = 0;

    for (uint i=0; i<points.size(); ++i){

      if (!point_valid[i] || !other.point_valid[i])
        continue;

      // dist between corresponding points after trafo
      double n = (t*points[i]-other.points[i]).norm();

      max_err = max(max_err,n);
      res += n/match_cnt;
    }

    if (residual != NULL)
      *residual = res;

    if (threshold_max_residual > 0 && max_err > threshold_max_residual)
      return false;


  }

  return true;

}


double Mocap_object::max_point_dist(Mocap_object& other, Eigen::Affine3f* trafo){

  double m = -1;
  assert(points.size() == other.points.size());

  Eigen::Affine3f inv = Eigen::Affine3f::Identity();
  if (trafo!=NULL) inv = trafo->inverse();

  int match_cnt = 0;
  for (uint i=0; i<points.size(); ++i){
    if (!point_valid[i] || !other.point_valid[i])
      continue;

    Eigen::Vector3f b = points[i];
    Eigen::Vector3f a = other.points[i];

    b = inv*b; // inv is unity if no trafo given!

    match_cnt++;
    double d = (b-a).norm();
    if (d>m)  m=d;

  }

  if (match_cnt == 0){
    ROS_WARN("max_point_dist computed, but no valid pairs compared");
    return -1; // as initialization...
  }

  return m;



}

void Mocap_object::reset(){

  points.clear();
  point_valid.clear();
  valid_point_cnt = 0;
  vertices.clear();
  stamp.fromNSec(-1);

}
