/*
 * evaluation.cpp
 *
 *  Created on: Jan 16, 2012
 *      Author: engelhar
 */

#include "evaluation.h"
#include <time.h>
#include <cstdlib>



// error: dx^2+dy^2+dz^2 (in cm) + roll^2... (in deg
bool isSimilar(Eigen::Affine3f& a, Eigen::Affine3f& b, float max_d, float max_rad, float* error){


  Eigen::Affine3f rel = a.inverse()*b;
  float t[6];
  affine3fToXyzRpy(rel,t);

//  printf("%f %f %f %f %f %f \n", t[0], t[1], t[2], t[3], t[4], t[5]);

  bool close = true;
  for (int k=0; k<3;k++) if (fabs(t[k])>max_d) close = false;
  for (int k=3; k<6;k++) if (fabs(t[k])>max_rad) close = false;

  if (error !=NULL){
    *error = 0;
    float e_d = 0;
    float e_r = 0;

    for (int k=0; k<3;k++) e_d += t[k]*t[k]*100*100;// m to cm
    for (int k=3; k<6;k++) e_r += (t[k]/M_PI*180)*(t[k]/M_PI*180);

//    ROS_INFO("error: dist: %.2f, deg: %.2f", e_d, e_r);

    *error = (e_d+e_r);
  }



  return close;
}


void checkStartingPose(){





  //  srand ( time(NULL) );
  //
  //
  //
  //
  //  // check this many starting poses
  //  int trial_cnt = 1000;
  //
  //  // number of g2o-iterations
  //  int g2o_iter_cnt = 1000;
  //
  //  // double of maximal deviation in each direction (in m)
  //  float dist_trans_max_2 = 4;
  //  // double of maximal angle deviation in each direction (in rad)
  //  float dist_rad_max_2 = M_PI;
  //
  //  // final position of object (could also be sampled)
  //  float x,y,z,roll,pitch,yaw;
  //  x = y = 0;
  //  z = 5;
  //  roll = pitch = yaw = 0;
  //
  //
  //  Simulator sim;
  //  Optimizer optimizer;
  //  Mocap_object mo;
  //  Mocap_object mo_opt;
  //  optimizer.setCamParams(sim.c_x,sim.c_y,sim.f_x, sim.f_y);
  //
  //  double mean_max_error = 0;
  //  double max_max_error = 0;
  //
  //  ofstream off;
  //  off.open("/home/engelhar/ros/mocap/error.txt");
  //
  //  // number of iterations with final max error < 0.01 (1cm)
  //  int perfect_cnt = 0;
  //
  //  for (int i=1; i<=trial_cnt; ++i){
  //
  //    sim.createTriangle(&mo);
  //    sim.createTriangle(&mo_opt);
  //
  //
  //
  //    float dx = dist_trans_max_2*(rand()*1.0/RAND_MAX-0.5);
  //    float dy = dist_trans_max_2*(rand()*1.0/RAND_MAX-0.5);
  //    float dz = dist_trans_max_2*(rand()*1.0/RAND_MAX-0.5);
  //
  //    float d_roll  = dist_rad_max_2*(rand()*1.0/RAND_MAX-0.5);
  //    float d_pitch = dist_rad_max_2*(rand()*1.0/RAND_MAX-0.5);
  //    float d_yaw   = dist_rad_max_2*(rand()*1.0/RAND_MAX-0.5);
  //
  //
  //    // move copy of object to a random place in the vincinity
  //    sim.trafoObject(&mo,x+dx,y+dy,z+dz,roll+d_roll,pitch+d_pitch,yaw+d_yaw);
  //    sim.trafoObject(&mo_opt,x,y,z,roll,pitch,yaw);
  //
  //
  //    // create projections from static object
  //    std::vector<CvPoint2D32f> prj = sim.computeProjections(&mo, false);
  //    optimizer.setOberservations(prj);
  //    optimizer.setObject(&mo_opt);
  //    optimizer.InitOptimization();
  //
  //    ros::Time begin = ros::Time::now();
  //    optimizer.optimize(g2o_iter_cnt);
  //    printf("opt took %i ms\n", int((ros::Time::now()-begin).toNSec()/1000/1000));
  //    mo_opt.getPoseFromVertices();
  //
  //    double max_dist = mo.max_point_dist(mo_opt);
  //
  //    mean_max_error += max_dist;
  //
  //    if (max_dist > max_max_error)
  //      max_max_error = max_dist;
  //
  //    if (max_dist<0.01)
  //      perfect_cnt++;
  //
  //    ROS_WARN("iteration %i: final error: %.2f", i, max_dist);
  //
  //    off  << max_dist << endl;
  //
  //  }
  //
  //  mean_max_error /= trial_cnt;
  //
  //  ROS_WARN("after %i runs: maxmaxError: %.2f, mean max error: %.2f", trial_cnt, max_max_error, mean_max_error);
  //  ROS_WARN("convergence to optimum in %i cases (%.1f %%)", perfect_cnt, perfect_cnt*100.0/trial_cnt);
  //  off.close();


}
