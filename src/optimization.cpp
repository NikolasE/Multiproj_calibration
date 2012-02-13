/*
 * optimization.cpp
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */

#include "optimization.h"

void Optimizer::initOptimizer(){

  optimizer = new g2o::SparseOptimizer;

  optimizer->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  g2o::BlockSolverX::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>(); // alternative: CHOLMOD
  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(optimizer,linearSolver);
  optimizer->setSolver(solver_ptr);
  optimizer->setVerbose(false);
  optimizer->setUserLambdaInit(100.);

  g2o::CameraParameters* cameraParams = new g2o::CameraParameters();
  cameraParams->setKcam(fx,fy,cx,cy);
  //  ROS_INFO("cam: %f %f %f %f",fx,fy,cx,cy );
  g2o::SE3Quat offset; // identity
  cameraParams->setOffset(offset);
  cameraParams->setId(0);
  optimizer->addParameters(cameraParams);

  vertex_id = 0;

}

void Optimizer::optimize(int iter, bool verbose){

  optimizer->save("/home/engelhar/Desktop/before.g2o");

  optimizer->setVerbose(verbose);
  optimizer->initializeOptimization();


  //  EdgeSet::iterator edge_iter = optimizer->edges().begin();
  //  for(int i =0; edge_iter != optimizer->edges().end(); edge_iter++, i++) {
  //
  //
  //    g2o::EdgeTrackXYZ_2* myedge = dynamic_cast<g2o::EdgeTrackXYZ_2*>(*edge_iter);
  //    if (myedge){
  //      myedge->computeError();
  //      ROS_INFO("EdgeTrackXYZ_2: chi2 %f", myedge->chi2());
  //      continue;
  //    }
  //
  //    g2o::EdgeProject* myedge2 = dynamic_cast<g2o::EdgeProject*>(*edge_iter);
  //    if (myedge2){
  //      myedge2->computeError();
  //      ROS_INFO("EdgeProject: chi2 %f", myedge2->error().norm());
  //      continue;
  //    }
  //
  //    ROS_ERROR("strange edge found");
  //
  //
  //  }

  //  for (EdgeContainer::const_iterator it = optimizer->_activeEdges.begin(); it != _activeEdges.end(); it++) {
  //    OptimizableGraph::Edge* e = *it;
  //    e->computeError();
  //    if (e->robustKernel()) {
  //      e->robustifyError();
  //    }
  //  }


  optimizer->computeActiveErrors();

//  double achi = optimizer->activeChi2();
//  printf("Optimization with %i nodes and %i edges \n", (int)optimizer->vertices().size(),(int) optimizer->edges().size());
//  std::cerr << "Start   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  optimizer->optimize(iter);

//  achi = optimizer->activeChi2();
//  std::cerr << "Final   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  optimizer->save("/home/engelhar/Desktop/after.g2o");
}



void Optimizer::addCameraToGraph(Camera& cam, bool fixed){

  cam.vertex = new g2o::VertexSE3();
  Eigen::Quaterniond eigen_quat(cam.pose.rotation().cast<double>());
  Eigen::Vector3d translation(cam.pose.translation()[0], cam.pose.translation()[1],cam.pose.translation()[2]);
  g2o::SE3Quat(eigen_quat, translation);

  cam.vertex->setEstimate(g2o::SE3Quat(eigen_quat, translation));
  cam.vertex->setFixed(fixed);
  cam.vertex->setId(vertex_id++);
  optimizer->addVertex(cam.vertex, NULL);
}

void  Optimizer::addCamerasToGraph(vector<Camera>& cams, bool fix_all){

  if (fix_all)
    cout << "Fixing all cams" << endl;
  else
    cout << "Fixing only first cam" << endl;

  for (uint i=0;i<cams.size(); ++i){
    if (fix_all)
      addCameraToGraph(cams[i], true);
    else
      addCameraToGraph(cams[i], i==0);
  }

}



void  Optimizer::addProjections(Camera& cam, Mocap_object& obj){


  if (obj.vertices.size() == 0){
    assert(1==0 && "Trying to add edge to Mocap_object that is not yet in the graph");
  }

  assert(cam.vertex != NULL);

  for (Observations::iterator it = cam.obs.begin(); it != cam.obs.end(); ++it){

    int v_id = it->first;
    CvPoint2D32f px = it->second;

    g2o::EdgeProject *obsEdge = new g2o::EdgeProject();
    obsEdge->vertices()[0] = cam.vertex;
    obsEdge->vertices()[1] = obj.vertices[v_id];

    //    cout << "cam: " << cam.vertex << " obj: " << m_obj->vertices[v_id]<< endl;

    obsEdge->information() = Eigen::Matrix<double, 3, 3>::Identity();
    obsEdge->setCacheId(0,0);
    Eigen::Vector2d m;
    m(0) = px.x;  m(1) = px.y;
    //    ROS_INFO("obs: %f %f", px.x,px.y);
    obsEdge->setMeasurement(m);
    //    projectionEdge->setRobustKernel(true);
    optimizer->addEdge(obsEdge);
  }

}

void Optimizer::addProjections(vector<Camera>& cams, Mocap_object& obj){
  for (uint i=0;i<cams.size(); ++i){
    addProjections(cams[i], obj);
  }
}



void Optimizer::addMocapObjectToGraph(Mocap_object& new_obj){

  uint n = new_obj.points.size();

  new_obj.vertices.resize(n);
  for (uint i=0; i<n; ++i){
    g2o::VertexTrackXYZ* v = new g2o::VertexTrackXYZ();
    //    cout << "new vertex pointer " << v << endl;
    v->setEstimate(new_obj.points[i].cast<double>());
    v->setId(vertex_id++);
    v->setFixed(false);
    optimizer->addVertex(v,NULL);
    new_obj.vertices[i] = v;
  }


  for (uint i=0; i<n-1; ++i){
    Eigen::Vector3f v = new_obj.points[i];
    for (uint j=i+1; j<n; ++j){
      Eigen::Vector3f  c = new_obj.points[j];

      g2o::EdgeTrackXYZ_2* edge = new g2o::EdgeTrackXYZ_2();
      edge->vertices()[0] = new_obj.vertices[i];
      edge->vertices()[1] = new_obj.vertices[j];

      //      cout << "connecting " <<   m_obj->vertices[i] << "  " <<     m_obj->vertices[j] << endl;

      // large information to force constraint to be fulfilled
      edge->information() = Eigen::Matrix<double, 3, 3>::Identity()*100;
      double measurement = (c-v).norm();

      //      cout << "edge between " << v << " and " << c << endl;
      //      cout << "meas: " << measurement << endl;

      edge->setMeasurementData(&measurement);
      optimizer->addEdge(edge);
      //      edges.push_back(edge);
    }
  }

}
