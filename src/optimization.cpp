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
  g2o::SE3Quat offset; // identity
  cameraParams->setOffset(offset);
  cameraParams->setId(0);
  optimizer->addParameters(cameraParams);




}

void Optimizer::optimize(int iter){

  optimizer->computeActiveErrors();


  double achi = optimizer->activeChi2();
  printf("Optimization with %i nodes and %i edges \n", (int)optimizer->vertices().size(),(int) optimizer->edges().size());
  std::cerr << "start   chi2= " << achi << " edge_cnt = "  <<  achi/optimizer->edges().size() << std::endl;

  optimizer->optimize(iter);

  achi = optimizer->activeChi2();
  std::cerr << "Final   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;
}


void Optimizer::InitOptimization(){

  initOptimizer();
  addMocapObjectToGraph();
  addProjectionEdgesToGraph();


  optimizer->save("/home/engelhar/Desktop/before.g2o");
  optimizer->initializeOptimization();
  //  cout << "edges: " <<  optimizer->edges().size() << endl;
  //
  //
  //
  //  optimizer->save("/home/engelhar/Desktop/after.g2o");


  //  EdgeSet::iterator edge_iter = optimizer->edges().begin();
  //  for(int i =0;edge_iter != optimizer->edges().end() && i<3; edge_iter++, i++) {
  //    g2o::EdgeTrackXYZ_2* edge = dynamic_cast<g2o::EdgeTrackXYZ_2*>(*edge_iter);
  //    Eigen::Vector3d ev = edge->error();
  //    printf("edge %i has error %f \n", i, ev.norm());
  //  }


}


void Optimizer::testVertex2Edge(){

  initOptimizer();

  addMocapObjectToGraph();

  vector<g2o::EdgeTrackXYZ_2*> edges;
  g2o::EdgeTrackXYZ_2  *edge = new g2o::EdgeTrackXYZ_2();
  edge->vertices()[0] = m_obj->vertices[0];
  edge->vertices()[1] = m_obj->vertices[1];
  edge->information() = Eigen::Matrix<double, 3, 3>::Identity();
  double measurement = 5;
  edge->setMeasurementData(&measurement);
  optimizer->addEdge(edge);
  edges.push_back(edge);

  edge = new g2o::EdgeTrackXYZ_2();
  edge->vertices()[0] = m_obj->vertices[0];
  edge->vertices()[1] = m_obj->vertices[2];
  edge->information() = Eigen::Matrix<double, 3, 3>::Identity();
  measurement = 5;
  edge->setMeasurementData(&measurement);
  optimizer->addEdge(edge);
  edges.push_back(edge);

  edge = new g2o::EdgeTrackXYZ_2();
  edge->vertices()[0] = m_obj->vertices[1];
  edge->vertices()[1] = m_obj->vertices[2];
  edge->information() = Eigen::Matrix<double, 3, 3>::Identity();
  measurement = 10;
  edge->setMeasurementData(&measurement);
  optimizer->addEdge(edge);
  edges.push_back(edge);


  optimizer->save("/home/engelhar/Desktop/before.g2o");
  optimizer->initializeOptimization();

  optimizer->computeActiveErrors();
  double achi = optimizer->activeChi2();
  std::cerr << "start   chi2= " << achi << " edge_cnt = "  <<  optimizer->edges().size() << std::endl;

  optimizer->optimize(10);

  achi = optimizer->activeChi2();
  std::cerr << "Final   chi2= " << achi << " Normalized chi2= "  << achi / optimizer->edges().size() << std::endl;

  optimizer->save("/home/engelhar/Desktop/after.g2o");




}


void Optimizer::addProjectionEdgesToGraph(){



  // one vertex for each marker, but not every marker has an observation
  assert(m_obj->cloud.points.size() == m_obj->vertices.size());
  assert(m_obj->cloud.points.size() >= obs.size());


  // add camera
  g2o::VertexSE3* cam = new g2o::VertexSE3();
  cam->setEstimate(g2o::SE3Quat());
  cam->setFixed(true);
  cam->setId(m_obj->vertices.size());
  optimizer->addVertex(cam, NULL);


  // add projections
  for (Observations::iterator it = obs.begin(); it != obs.end(); ++it){

    int vertex_id = it->first;
    CvPoint2D32f px = it->second;

    g2o::EdgeProject *obsEdge = new g2o::EdgeProject();
    obsEdge->vertices()[0] = cam;
    obsEdge->vertices()[1] = m_obj->vertices[vertex_id];
    obsEdge->information()=Eigen::Matrix<double, 3, 3>::Identity();
    obsEdge->setCacheId(0,0);
    Eigen::Vector2d m; m(0) = px.x;  m(1) = px.y;
    obsEdge->setMeasurement(m);
    //    projectionEdge->setRobustKernel(true);
    optimizer->addEdge(obsEdge);
  }


}


void Optimizer::addMocapObjectToGraph(){

  uint n = m_obj->cloud.points.size();

  for (uint i=0; i<n; ++i){
    point_type p = m_obj->cloud.points[i];

    g2o::VertexTrackXYZ* v = new g2o::VertexTrackXYZ();
    v->setEstimate(Eigen::Vector3d(p.x,p.y,p.z));
    v->setId(i);
    v->setFixed(false);
    optimizer->addVertex(v,NULL);
    m_obj->vertices.push_back(v);
  }


  for (uint i=0; i<n-1; ++i){
    point_type v = m_obj->cloud.points[i];
    for (uint j=i+1; j<n; ++j){
      point_type c = m_obj->cloud.points[j];

      g2o::EdgeTrackXYZ_2* edge = new g2o::EdgeTrackXYZ_2();
      edge->vertices()[0] = m_obj->vertices[i];
      edge->vertices()[1] = m_obj->vertices[j];
      // large information to force constraint to be fulfilled
      edge->information() = Eigen::Matrix<double, 3, 3>::Identity()*10;
      double measurement = sqrt(pow(c.x-v.x,2)+pow(c.y-v.y,2)+pow(c.z-v.z,2));

      //      cout << "edge between " << v << " and " << c << endl;
      //      cout << "meas: " << measurement << endl;


      edge->setMeasurementData(&measurement);
      optimizer->addEdge(edge);
      //      edges.push_back(edge);
    }
  }

}
