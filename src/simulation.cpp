/*
 * simulation.cpp
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */


#include "simulation.h"

double Simulator::getGaussianSample(double mu, double sigma){
  double v=0;
  for (int i=0; i<12; ++i)
    v+= rand()*1.0/RAND_MAX;

  v/=6;

  return v*sigma+mu;


}


void Simulator::createRect(Mocap_object* mo, float z, float w, float h, float x_c, float y_c){
  mo->cloud.points.clear();
  mo->addVertex(x_c-w/2,y_c-h/2,z);
  mo->addVertex(x_c-w/2,y_c+h/2,z);
  mo->addVertex(x_c+w/2,y_c+h/2,z);
  mo->addVertex(x_c+w/2,y_c-h/2,z);
}

void Simulator::createTriangle(Mocap_object* mo){

  mo->cloud.points.clear();
  mo->addVertex(0,0,0);
  mo->addVertex(3,0,0);
  mo->addVertex(0,-1,0);
  mo->addVertex(3,2,2);
}


void Simulator::perturbProjections( std::vector<CvPoint2D32f>& obs, double sigma){

  for (uint i=0; i<obs.size(); i++){

    double dx = Simulator::getGaussianSample(0,sigma);
    double dy = Simulator::getGaussianSample(0,sigma);

    ROS_WARN("observation moved by %f %f", dx,dy);

    obs[i].x += dx; // zero centered normal
    obs[i].y += dy;
  }

}


void Simulator::trafoObject(Mocap_object* mo, float dx, float dy, float dz,float phi, float theta, float psi){


  uint N = mo->cloud.points.size();

  assert(N>0);

  Eigen::Vector3f mean(0,0,0);
  // find mean of object:
  for (uint i=0; i<N; ++i){
    point_type p=mo->cloud.points[i];
    mean(0) += p.x; mean(1) += p.y; mean(2) += p.z;
  }

  mean /= N;

  // demean pointcloud:
  for (uint i=0; i<N; ++i){
    point_type p=mo->cloud.points[i];
    p.x -= mean(0);  p.y -= mean(1); p.z -= mean(2);
    mo->cloud.points[i] = p;
  }

  // transform pointcloud (rotate, add translation and mean
  tf::Quaternion quat = tf::createQuaternionFromRPY(phi, theta, psi);

//  printf("quat: %f %f %f %f\n", quat.x(),quat.y(),quat.z(), quat.w());

  Eigen::Quaternionf eig_quad(quat.x(),quat.y(),quat.z(), quat.w());

//  cout << "foo" << eig_quad << endl;

  pcl::transformPointCloud(mo->cloud, mo->cloud, Eigen::Vector3f(dx+mean(0),dy+mean(1),dz+mean(2)), eig_quad);
}


std::vector<CvPoint2D32f> Simulator::computeProjections(Mocap_object* mo, bool show_image){

  if (show_image)
    cvNamedWindow("Projection",1);

  std::vector<CvPoint2D32f> prj;

  for (uint i=0; i<mo->cloud.points.size(); ++i){
    point_type  c = mo->cloud.points[i];
    float x_px = c.x/c.z*f_x+c_x;
    float y_px = c.y/c.z*f_y+c_y;

    if (c.z < 0)
      std::cerr << "########### vertex " << i << " is behind the camera" << std::endl;

//    printf("%i: %f %f \n",i, x_px, y_px);

    prj.push_back(cvPoint2D32f(x_px,y_px));

    //    if (x_px < 0 || y_px < 0 || x_px >= c_width || y_px >= c_height)
    //      printf("not within image! \n ");
  }


  if (show_image) {
    // print on image
    cvSet(img, cvScalar(0,0,0));
    for (uint i=0; i<prj.size(); ++i){
      float x = prj[i].x; float y = prj[i].y;
      cvCircle(img, cvPoint(x,y),5, CV_RGB(0,255,0),2);
    }

    for (uint i=0; i<prj.size(); ++i){
      float x = prj[i].x; float y = prj[i].y;
      float x_ = prj[(i+1)%prj.size()].x; float y_ = prj[(i+1)%prj.size()].y;
      cvLine(img,cvPoint(x,y),cvPoint(x_,y_), CV_RGB(255,0,0),2);
    }
    cvShowImage("Projection", img);
  }

  return prj;
}
