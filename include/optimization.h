/*
 * optimization.h
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */

#ifndef OPTIMIZATION_H_
#define OPTIMIZATION_H_


#include <g2o/types/slam3d/camera_parameters.h>

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mocap_defs.h"
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

class Optimizer {



public:
  Mocap_object* m_obj;

  g2o::SparseOptimizer* optimizer;
  float cx, cy, fx, fy;

  void setCamParams(Camera* cam){
    cx=cam->c_x; cy=cam->c_y; fx=cam->f_x; fy=cam->f_y; }

  void setObject(Mocap_object* mo){m_obj = mo; m_obj->vertices.clear();}

  void testVertex2Edge();

  void initOptimizer();

  void optimize(int iter);

  void getOptimizedObject(Mocap_object* new_obj);
  void getRelPose(Mocap_object& moved, Mocap_object& init);

  void addCameraToGraph(Camera& cam);
private:

  uint vertex_id;
  void addMocapObjectToGraph();




};






#endif /* OPTIMIZATION_H_ */
