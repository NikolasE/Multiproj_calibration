/*
 * extrinsic_calibration.cpp
 *
 *  Created on: Feb 9, 2012
 *      Author: engelhar
 */


#include "extrinsic_calibration.h"



void ExtrinsicCalibration::init(){
  assert(bagFileName.length() > 0 && "No BagFile set (use setBagFile)");
  assert(propFileName.length() > 0 && "No PropFile set (use setPropFile)");

  gt.readBagPropFile(propFileName.c_str());
  gt.loadBag(bagFileName.c_str());

  if (simProp_set)
    gt.readSimPropFile(simPropFileName.c_str()); // sim.prop
}


#define NO_OUTPUT

void ExtrinsicCalibration::run(int file_skip, int step){

  assert(cams_->size()> 0 && "No cams set!");
  assert(simProp_set && "Only sim obect is implemented");

  Optimizer optimizer;
  optimizer.setCamParams(&cams_->at(0));

  optimizer.initOptimizer();
  optimizer.addCamerasToGraph(*cams_, false); // fix first cam

  Mocap_object init_object = gt.bag_sim_object;

  gt.skip = step;
  gt.file_idx = file_skip;

  Mocap_object mo;

  while (gt.getNextInstance(mo) && gt.file_idx < uint(file_skip + 10000) ){

    Mocap_object init_object = gt.bag_sim_object;

    cout << gt.file_idx << endl;

    if (!mo.gt_trafo_valid) continue;

    Eigen::Affine3f trafo = mo.gt_trafo;

    Mocap_object mo_sim = gt.bag_sim_object;
    mo_sim.moveObject(trafo);

#ifndef NO_OUTPUT
    int id=0;
    id = sendObject(marker_pub, id, mo_sim, "gt_marker", 1,0,1);
#endif

    int total_obs = 0;

    for (uint i=0; i<cams_->size(); ++i){
      total_obs += sim_.computeObservations(&mo_sim,&cams_->at(i),false);
#ifndef NO_OUTPUT
      id = sendCam(marker_pub, cams[i], id,  "gt_marker_cam", 1,0,0);
      id = sendProjectionRays(marker_pub, cams[i], id, "gt_marker_proj", 0,0,1);
#endif
    }

    //      ROS_INFO("obs_cnt: %i", total_obs);

    if (total_obs>=4 && mo.gt_trafo_valid){

      optimizer.addMocapObjectToGraph(init_object);
      optimizer.addProjections(*cams_, init_object);

#ifndef NO_OUTPUT
      id = sendObject(marker_pub, id, init_object, "gt_marker", 0,1,0);
#endif
    }else
    {
#ifndef NO_OUTPUT
      id = sendObject(marker_pub, id, gt.bag_sim_object, "gt_marker", 1,0,0);
#endif
    }


#ifndef NO_OUTPUT
    ros::Duration(atof(argv[2])).sleep();
#endif

  }


  // move cam poses a bit
  // don't move first cam!

  vector<Eigen::Matrix4f> orig;
  orig.resize(cams_->size());
  for (uint i=0; i<cams_->size(); ++i){
    orig[i] = cams_->at(i).vertex->estimate().to_homogenious_matrix().cast<float>();
  }

  for (uint i=1; i<cams_->size(); ++i){

    Eigen::Quaterniond eigen_quat(cams_->at(i).pose.rotation().cast<double>());
    Eigen::Vector3d translation(cams_->at(i).pose.translation()[0]+1, cams_->at(i).pose.translation()[1]+0.5,cams_->at(i).pose.translation()[2]+0.2);
    g2o::SE3Quat(eigen_quat, translation);

    cams_->at(i).vertex->setEstimate(g2o::SE3Quat(eigen_quat, translation));
  }


  optimizer.optimize(1000, true);

  for (uint i=0; i<10; ++i)
    optimizer.optimize(100, true);


  vector<Eigen::Matrix4f> estimated;
  estimated.resize(cams_->size());
  for (uint i=0; i<cams_->size(); ++i){
    estimated[i] = cams_->at(i).vertex->estimate().to_homogenious_matrix().cast<float>();
  }

  cout << "diff between cams" << endl;
  for (uint i=0; i<cams_->size()-1; ++i){
    for (uint j=i+1; j<cams_->size(); ++j){
      ROS_INFO("dist %i %i", i,j);
      Eigen::Matrix4f d_orig =  orig[i].inverse()*orig[j];
      Eigen::Matrix4f d_est =  estimated[i].inverse()*estimated[j];
      cout << d_orig << endl;
      cout << d_est << endl;

      cout << "dist between rel poses:" << endl;
      cout << d_orig.inverse()*d_est << endl;

    }
  }


}

