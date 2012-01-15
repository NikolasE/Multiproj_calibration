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
  std::vector<CvPoint2D32f> obs;
  g2o::SparseOptimizer* optimizer;
  float cx, cy, fx, fy;

  void setCamParams(float cx_, float cy_, float fx_, float fy_){
    cx=cx_; cy=cy_; fx=fx_; fy=fy_; }

  void setObject(Mocap_object* mo){m_obj = mo; m_obj->vertices.clear();}
  void setOberservations(std::vector<CvPoint2D32f> obs_){obs = obs_;}

  void testVertex2Edge();


  void InitOptimization();

  void optimize(int iter);

  void getOptimizedObject(Mocap_object* new_obj);
  void getRelPose(Mocap_object& moved, Mocap_object& init);

private:
  void initOptimizer();
  void addMocapObjectToGraph();
  void addProjectionEdgesToGraph();
//  std::vector<g2o::VertexTrackXYZ*> vertices;



};






#endif /* OPTIMIZATION_H_ */
