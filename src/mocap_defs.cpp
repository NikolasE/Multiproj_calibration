

#include "mocap_defs.h"


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
//
//      printf("d1,d2: %f %f , abs: %f\n", d1, d2, diff);


      if (max_dist != NULL && *max_dist<diff)
        *max_dist = diff;

      if (diff>dist_thres)
        return false;
    }
  }

  return true;

}
