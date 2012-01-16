

#include "mocap_defs.h"
#include <pcl/common/transformation_from_correspondences.h>


void Mocap_object::getPoseFromVertices(){
  assert(vertices.size() == cloud.points.size());

  for (uint i=0; i<cloud.points.size(); ++i){
    Eigen::Vector3d pose = vertices[i]->estimate();
    cloud.points[i].x = pose(0);
    cloud.points[i].y = pose(1);
    cloud.points[i].z = pose(2);
  }

}


bool Mocap_object::isSameObject(Mocap_object& other, float dist_thres, float* max_dist){

  if (cloud.points.size() != other.cloud.points.size()) return false;

  if (max_dist != NULL) *max_dist = -1;

  uint n = cloud.points.size();
  for (uint i=0; i<n-1; ++i){
    point_type v = cloud.points[i];
    point_type v_o = other.cloud.points[i];
    for (uint j=i+1; j<n; ++j){
      point_type c = cloud.points[j];
      point_type c_o = other.cloud.points[j];

      double d1 = sqrt(pow(c.x-v.x,2)+pow(c.y-v.y,2)+pow(c.z-v.z,2));
      double d2 = sqrt(pow(c_o.x-v_o.x,2)+pow(c_o.y-v_o.y,2)+pow(c_o.z-v_o.z,2));

      double diff = fabs(d1-d2);

//      cout << "edge between " << v << " and " << c << endl;
//      cout << "and edge between " << v_o << " and " << c_o << endl;
//      printf("d1,d2: %f %f , abs: %f\n", d1, d2, diff);


      if (max_dist != NULL && *max_dist<diff)
        *max_dist = diff;

      if (diff>dist_thres)
        return false;
    }
  }

  return true;

}


bool Mocap_object::getTrafoTo(Mocap_object& other,Eigen::Affine3f& t, float* trafo){

  if (!isSameObject(other, 0.1)){
    cout << "#### getTrafoTo: not the same object" << endl;
    return false;
  }

  pcl::TransformationFromCorrespondences tfc;


  // get mean
  for (uint i=0; i<cloud.points.size(); ++i){
    point_type from = cloud.points[i];
    point_type to = other.cloud.points[i];

    tfc.add(Eigen::Vector3f(from.x,from.y,from.z),Eigen::Vector3f(to.x,to.y,to.z));
  }

  t = tfc.getTransformation();

  if (trafo!=NULL){
    // pcl::getTranslationAndEulerAngles
    trafo[0] = t(0,3);
    trafo[1] = t(1,3);
    trafo[2] = t(2,3);
    trafo[3] = atan2f(t(2,1), t(2,2));
    trafo[4] = asinf(-t(2,0));
    trafo[5] = atan2f(t(1,0), t(0,0));
  }

  return true;

}


double Mocap_object::max_point_dist(Mocap_object& other, Eigen::Affine3f* trafo){

  double m = -1;
  assert(cloud.points.size() == other.cloud.points.size());

  Eigen::Affine3f inv = Eigen::Affine3f::Identity();
  if (trafo!=NULL) inv = trafo->inverse();

  for (uint i=0; i<cloud.points.size(); ++i){
    point_type a = cloud.points[i];
    point_type b = other.cloud.points[i];

    Eigen::Vector3f b_3(b.x,b.y,b.z);
    Eigen::Vector3f a_3(a.x,a.y,a.z);

    b_3 = inv*b_3; // inv is unity if no trafo given!

    double d = (b_3-a_3).norm();
    if (d>m)  m=d;

  }

  return m;



}
