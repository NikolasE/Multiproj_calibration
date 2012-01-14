/*
 * mocap_defs.h
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */

#ifndef MOCAP_DEFS_H_
#define MOCAP_DEFS_H_

#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/camera_parameters.h>
#include <g2o/types/slam3d/vertex_trackxyz.h>
#include <g2o/types/slam3d/edge_project_disparity.h>
#include <g2o/types/slam3d/edge_se3_quat.h>

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>



struct Mocap_object {

  std::vector<Eigen::Vector4f> vertices;

  void addVertex(float x, float y, float z){
    Eigen::Vector4f v(x,y,z,1);
    vertices.push_back(v);
  }



  void printObject(){

    for (uint i=0; i<vertices.size(); ++i){
      Eigen::Vector4f c = vertices[i];
       printf("v %i: %f %f %f", i, c.x(),c.y(), c.z());
    }

    for (uint i=0; i<vertices.size()-1; ++i){
      Eigen::Vector4f v = vertices[i];
      for (uint j=i+1; j<vertices.size(); ++j){
        Eigen::Vector4f c = vertices[j]-v;
        printf("rel (%i,%i): %f %f %f", i,j, c.x(),c.y(), c.z());
      }
    }

  }


};



#endif /* MOCAP_DEFS_H_ */
