/*
 * simulation.cpp
 *
 *  Created on: Jan 14, 2012
 *      Author: engelhar
 */


#include "simulation.h"
#include <pcl/common/transform.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

double Simulator::getGaussianSample(double mu, double sigma){
  double v=0;
  for (int i=0; i<12; ++i)
    v+= rand()*1.0/RAND_MAX;
  v/=6;
  return v*sigma+mu;
}

void Simulator::createRect(Mocap_object* mo, float z, float w, float h, float x_c, float y_c){
  mo->reset();
  mo->addVertex(x_c-w/2,y_c-h/2,z);
  mo->addVertex(x_c-w/2,y_c+h/2,z);
  mo->addVertex(x_c+w/2,y_c+h/2,z);
  mo->addVertex(x_c+w/2,y_c-h/2,z);
}


void Simulator::createRandomPose(float sig_trans, float sig_rot, float* mean, float* trafo){

  if (mean == NULL){
    for (int i=0; i<3; ++i) trafo[i] = sig_trans*(rand()*1.0/RAND_MAX-0.5);
    for (int i=3; i<6; ++i) trafo[i] = sig_rot*(rand()*1.0/RAND_MAX-0.5);
    return;
  }

  for (int i=0; i<3; ++i) trafo[i]= mean[i] + sig_trans*(rand()*1.0/RAND_MAX-0.5);
  for (int i=3; i<6; ++i) trafo[i]= mean[i] + sig_rot*(rand()*1.0/RAND_MAX-0.5);

}

void Simulator::createTriangle(Mocap_object* mo){

  mo->reset();
  mo->addVertex(0,0,0);
  mo->addVertex(3,0,0);
  mo->addVertex(0,-1,0);
  mo->addVertex(3,2,2);
}


void Simulator::perturbProjections(Observations& obs, double sigma){

  for (Observations::iterator it = obs.begin(); it != obs.end(); ++it){

    double dx = Simulator::getGaussianSample(0,sigma);
    double dy = Simulator::getGaussianSample(0,sigma);

    ROS_WARN("observation moved by %f %f", dx,dy);

    CvPoint2D32f p = it->second;

    p.x += dx; // zero centered normal
    p.y += dy;

    obs[it->first] = p;

  }

}

void Simulator::trafoObject(Mocap_object* mo, float* trafo){
  trafoObject(mo, trafo[0], trafo[1], trafo[2], trafo[3], trafo[4], trafo[5]);

}
void Simulator::trafoObject(Mocap_object* mo, float dx, float dy, float dz,float phi, float theta, float psi){


//  uint N = mo->cloud.points.size();
//
//  assert(N>0);
//
//  Eigen::Vector3f mean(0,0,0);
//  // find mean of object:
//  for (uint i=0; i<N; ++i){
//    point_type p=mo->cloud.points[i];
//    mean(0) += p.x; mean(1) += p.y; mean(2) += p.z;
//  }
//
//  mean /= N;
//
//  // demean pointcloud:
//  for (uint i=0; i<N; ++i){
//    point_type p=mo->cloud.points[i];
//    p.x -= mean(0);  p.y -= mean(1); p.z -= mean(2);
//    mo->cloud.points[i] = p;
//  }

  Eigen::Affine3f trafo;
//  pcl::getTransformation(dx+mean(0),dy+mean(1),dz+mean(2),phi, theta, psi,trafo);
  pcl::getTransformation(dx,dy,dz,phi,theta, psi,trafo);

  for (uint i=0; i<mo->points.size(); ++i){
    if (mo->point_valid[i])
      mo->points[i] = trafo*mo->points[i];

  }

//  pcl::transformPointCloud(mo->cloud, mo->cloud, trafo);
}


int Simulator::computeObservations(Mocap_object* mo, Camera* cam,  bool show_image){


  char img_name[100];
  if (show_image){
    sprintf(img_name,"Projection_%i", cam->id);
    cvNamedWindow(img_name,1);
  }

  cam->obs.clear();

  Eigen::Affine3f c2w = cam->pose.inverse();

  for (uint i=0; i<mo->points.size(); ++i){
    if (!mo->point_valid[i]) continue;
    Eigen::Vector3f  c = mo->points[i];

    c = c2w*c;

//    ROS_INFO("pt in cam frame: %f %f %f", c.x(), c.y(), c.z());

    float x_px = c.x()/c.z()*cam->f_x+cam->c_x;
    float y_px = c.y()/c.z()*cam->f_y+cam->c_y;

    // point behind camera or not within field of view
    if (c.z() < 0 || x_px < 0 || y_px < 0 || x_px >= cam->c_width || y_px >= cam->c_height){
      continue;
    }

    cam->obs[i] = cvPoint2D32f(x_px,y_px);

  }

  if (show_image) {
    // print on image
    cvSet(img, cvScalar(0,0,0));

    for (Observations::iterator it = cam->obs.begin(); it!=cam->obs.end(); ++it){
      cvCircle(img, cvPoint(it->second.x,it->second.y),3, CV_RGB(0,255,0),2);
    }

//    for (uint i=0; i<prj.size(); ++i){
//      float x = prj[i].x; float y = prj[i].y;
//      float x_ = prj[(i+1)%prj.size()].x; float y_ = prj[(i+1)%prj.size()].y;
//      cvLine(img,cvPoint(x,y),cvPoint(x_,y_), CV_RGB(255,0,0),2);
//    }
    cvShowImage(img_name, img);
    cvWaitKey(5);
  }


  return cam->obs.size();

}
