/*
 * main.cpp
 *
 *  Created on: Jan 8, 2012
 *      Author: engelhar
 */
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
//#include <cv_bridge/CvBridge.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/transform.h>

#include "mocap_defs.h"
#include "simulation.h"
#include "optimization.h"
#include "visualization.h"
#include "cluster.h"
#include "evaluation.h"
#include "groundtruth.h"
#include "extrinsic_calibration.h"


#include <algorithm>
#include <time.h>
#include <fstream>



using namespace sensor_msgs;


int  main (int argc, char** argv)
{




// #define NO_OUTPUT

#ifndef NO_OUTPUT
  ros::init(argc, argv, "g2o_mocap");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
#endif

#define QUADCOPTER

#ifdef QUADCOPTER


  Simulator sim_;

  vector<Camera> cams;
  Camera cam;
  pcl::getTransformation(0,0,5,M_PI,0,0,cam.pose);              cam.id = cams.size(); cams.push_back(cam);
  pcl::getTransformation(0,2,5,M_PI,0,0,cam.pose);              cam.id = cams.size(); cams.push_back(cam);
  pcl::getTransformation(-5,0,3,M_PI/2,M_PI,M_PI/2,cam.pose);   cam.id = cams.size(); cams.push_back(cam);
  pcl::getTransformation(0,5,3,M_PI/2,0,0,cam.pose);            cam.id = cams.size(); cams.push_back(cam);


#if 0
  ExtrinsicCalibration ec;
  ec.setBagFile("/home/engelhar/ros/mocap/data/2012-01-18-11-30-27.bag");
  ec.setPropFile("/home/engelhar/ros/mocap/data/quadrotor.prop");
  ec.setSimFile("/home/engelhar/ros/mocap/data/sim.prop");
  ec.addCameras(&cams);

  ec.init();
  ec.run(8000, 10);

  return 0;
#endif














  Groundtruth gt;
  gt.readBagPropFile("/home/engelhar/ros/mocap/data/quadrotor.prop");
  gt.readSimPropFile("/home/engelhar/ros/mocap/data/quadrotor.prop"); // sim.prop
  gt.loadBag("/home/engelhar/ros/mocap/data/2012-01-18-11-30-27.bag");



  assert(argc>2);


  gt.skip = 10;
  gt.file_idx = atoi(argv[1]);//8000; // skip boring start of file

  Mocap_object mo;


  ofstream gt_of("data/gt.txt");
  ofstream est_of("data/estimated.txt");

  Optimizer optimizer;
  optimizer.setCamParams(&cams[0]);

  optimizer.initOptimizer();
  optimizer.addCamerasToGraph(cams, false); // only fix first cam


  Mocap_object init_object = gt.bag_sim_object;

  while (gt.getNextInstance(mo) && gt.file_idx < atoi(argv[1]) + 10000 ){

    Mocap_object init_object = gt.bag_sim_object;

    cout << gt.file_idx << endl;

    if (!mo.gt_trafo_valid) continue;

    // quadcopter crashes afterwards :(
    if (gt.file_idx > 22000) break;

    Eigen::Affine3f trafo = mo.gt_trafo;

    Groundtruth::writePoseToFile(gt.file_idx, trafo, &gt_of);


    //for (int iter = 2000 ; iter<= 1000; iter += 200)
    int iter = 100;
    {

      Mocap_object mo_sim = gt.bag_sim_object;
      mo_sim.moveObject(trafo);

#ifndef NO_OUTPUT
      int id=0;
      id = sendObject(marker_pub, id, mo_sim, "gt_marker", 1,0,1);
#endif



      int total_obs = 0;

      for (uint i=0; i<cams.size(); ++i){

        int obs_cnt = sim_.computeObservations(&mo_sim,&cams[i],false);

        total_obs += obs_cnt;

//        if (obs_cnt > 0){
//          // optimizer.addCameraToGraph(cams[i], true);
//          optimizer.addProjections(cams[i], init_object);
//        }

#ifndef NO_OUTPUT
        id = sendCam(marker_pub, cams[i], id,  "gt_marker_cam", 1,0,0);
        id = sendProjectionRays(marker_pub, cams[i], id, "gt_marker_proj", 0,0,1);
#endif
      }

//      ROS_INFO("obs_cnt: %i", total_obs);

      if (total_obs>=4 && mo.gt_trafo_valid){

        optimizer.addMocapObjectToGraph(init_object);
        optimizer.addProjections(cams, init_object);



        // optimizer.optimize(iter);
//         init_object.getPoseFromVertices();


//        Eigen::Affine3f error;
//        Eigen::Affine3f t;
//        gt.bag_sim_object.getTrafoTo(init_object,t);
//        Groundtruth::writePoseToFile(gt.file_idx, t, &est_of);
//        error = trafo.inverse()*t;

//        double dist = error.translation().norm();
//        if (dist<0.02){
//         // ROS_INFO("err: %f", dist);
//        }
//        else
//          if (dist < 0.07)
//            ROS_WARN("err: %f", dist);
//          else {
//            ROS_ERROR("err: %f", dist);
//            cout << "est: " << endl << t.translation() << endl;
//            cout << "gt:  " << endl << trafo.translation() << endl;
//
//            char foo; cin >> foo;
//            // cv::waitKey(0);
//          }


#ifndef NO_OUTPUT
        id = sendObject(marker_pub, id, init_object, "gt_marker", 0,1,0);
#endif
      }else
      {
#ifndef NO_OUTPUT
        id = sendObject(marker_pub, id, gt.bag_sim_object, "gt_marker", 1,0,0);
#endif
      }

      //      if (!mo.gt_trafo_valid)
      //        ROS_WARN("gt-trafo invalid");

#ifndef NO_OUTPUT
      ros::Duration(atof(argv[2])).sleep();
#endif
    }
    //    cout << gt.getFilePosition() << endl;
    //    ros::Duration(atof(argv[2])).sleep();
  }


  // move cam poses a bit
  // don't move first cam!

  vector<Eigen::Matrix4f> orig;
  orig.resize(cams.size());
  for (uint i=0; i<cams.size(); ++i){
    orig[i] = cams[i].vertex->estimate().to_homogenious_matrix().cast<float>();
  }

  for (uint i=1; i<cams.size(); ++i){

    Eigen::Quaterniond eigen_quat(cams[i].pose.rotation().cast<double>());
    Eigen::Vector3d translation(cams[i].pose.translation()[0]+1, cams[i].pose.translation()[1]+0.5,cams[i].pose.translation()[2]+0.2);
    g2o::SE3Quat(eigen_quat, translation);

    cams[i].vertex->setEstimate(g2o::SE3Quat(eigen_quat, translation));
  }


  optimizer.optimize(1000, true);

  for (uint i=0; i<10; ++i)
    optimizer.optimize(100, true);


  vector<Eigen::Matrix4f> estimated;
  estimated.resize(cams.size());
  for (uint i=0; i<cams.size(); ++i){
    estimated[i] = cams[i].vertex->estimate().to_homogenious_matrix().cast<float>();
  }

  cout << "diff between cams" << endl;
  for (uint i=0; i<cams.size()-1; ++i){
    for (uint j=i+1; j<cams.size(); ++j){
      ROS_INFO("dist %i %i", i,j);
      Eigen::Matrix4f d_orig =  orig[i].inverse()*orig[j];
      Eigen::Matrix4f d_est =  estimated[i].inverse()*estimated[j];
      cout << d_orig << endl;
      cout << d_est << endl;

      cout << "dist between rel poses:" << endl;
      cout << d_orig.inverse()*d_est << endl;

    }
  }




  return 0;

#else

  srand ( 0 );

  // random goal pose
  float goal_pose[6];
  float init_pose[6]  = {0,0,15,0,0,0};
  float start_pose[6] = {0,0,10,0,0,0};


  int start_pose_list[5] = {5,10,20,50,100};
  int g2o_iter_list[5]  = {100,500,1000,5000,10000};


  int goal_pose_cnt  = 50; // number of tested poses
  //  int start_pose_cnt = 20; // number of initial poses for each testes pose
  //  int g2o_iter_cnt  = 10000;



  ofstream of_time;
  of_time.open("/export/home/engelhar/timing.txt");


  for (int g2l = 0; g2l < 5; ++g2l){


    int g2o_iter_cnt = g2o_iter_list[g2l];

    for (int spl = 0; spl < 5; ++spl){

      int start_pose_cnt = start_pose_list[spl];




      ROS_INFO("start_pose: %i, g2o: %i:", start_pose_cnt, g2o_iter_cnt);


      int found_track_cnt = 0;


      vector<int> iter_cnt;
      //      for (int i=0; i<=20; i+=2) iter_cnt.push_back(i);
      //      for (int i=25; i<100; i+=20) iter_cnt.push_back(i);
      //      for (int i=100; i<=1000; i+=150) iter_cnt.push_back(i);
      iter_cnt.push_back(g2o_iter_cnt);


      long opt_time_ms=0;

      for (int gpc = 0; gpc<goal_pose_cnt; gpc++){

        ROS_ERROR("gpc: %i (valid: %i) / %i", gpc, found_track_cnt, goal_pose_cnt);


        Simulator sim;
        Optimizer optimizer;
        Mocap_object mo;
        Mocap_object mo_opt;
        ClusterManager cl_man;
        Camera cam;

        Mocap_object original;
        sim.createRect(&original, 0, 3, 2);

        // goal pose is created once
        Simulator::createRandomPose(3,2*M_PI, start_pose, goal_pose); // zero-centered
        sim.createRect(&mo, 0, 3, 2);
        sim.trafoObject(&mo,goal_pose);
        //        ROS_WARN("goal_pose: %.2f %.2f %.2f r: %.2f %.2f %.2f ", goal_pose[0],goal_pose[1],goal_pose[2],goal_pose[3]/M_PI*180,goal_pose[4]/M_PI*180,goal_pose[5]/M_PI*180);

        int cnt = sim.computeObservations(&mo, &cam,  false);

        // last_trafo
        float tr[6];

        for (int spc = 0; spc<start_pose_cnt; spc++)
        {


          //      ROS_WARN("Init with start_pose %i", spc);


          if (spc == 0){
            Simulator::createRandomPose(3,M_PI, init_pose, init_pose);
          }
          else{
            //            ROS_WARN("last_poses: %.2f %.2f %.2f r: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);
            Simulator::createRandomPose(0,2*M_PI, tr, init_pose);
            //            ROS_WARN("new init: %.2f %.2f %.2f r: %.2f %.2f %.2f \n", init_pose[0],init_pose[1],init_pose[2],init_pose[3]/M_PI*180,init_pose[4]/M_PI*180,init_pose[5]/M_PI*180);
          }

          //      ofstream off;
          //      off.open("/home/engelhar/ros/mocap/error.txt");


          //    // same pertubation for all iterations
          //    double sigma = 3;
          //    double err_dx = Simulator::getGaussianSample(0,sigma); // zero centered normal
          //    double err_dy = Simulator::getGaussianSample(0,sigma);
          //
          //    ROS_WARN("observation moved by %f %f", dx,dy);

          for(uint i=0; i<iter_cnt.size(); ++i)
          {


            ROS_INFO("iterations: %i",iter_cnt[i]);

            // for each optimization, moved object is positioned at the same pose
            sim.createRect(&mo_opt, 0, 3, 2);
            //            ROS_WARN("starting with: %.2f %.2f %.2f r: %.2f %.2f %.2f \n", init_pose[0],init_pose[1],init_pose[2],init_pose[3]/M_PI*180,init_pose[4]/M_PI*180,init_pose[5]/M_PI*180);

            sim.trafoObject(&mo_opt,init_pose);



            //      if (false){
            //        // new error for each iteration (show amount of error better)
            //        Simulator::perturbProjections(prj, sigma);
            //      }else{
            //        // introduce same error: correct
            //        for (uint i=0; i<prj.size(); i++){
            //          prj[i].x += err_dx;
            //          prj[i].y += err_dy;
            //        }
            //      }

            optimizer.setCamParams(&cam);
            //            optimizer.setOberservations(obs);
            optimizer.setObject(&mo_opt);
            //            optimizer.InitOptimization();

            optimizer.initOptimizer();

            optimizer.addCameraToGraph(cam);

            ros::Time begin = ros::Time::now();

            optimizer.optimizer->initializeOptimization();

            optimizer.optimize(iter_cnt[i]);

            ros::Time end = ros::Time::now();
            opt_time_ms+= int((end-begin).toNSec()/1000.0/1000);

            //        printf("opt took %i ms\n", int((end-begin).toNSec()/1000.0/1000));

            mo_opt.getPoseFromVertices();


            Eigen::Affine3f t;

            //
            //            original.getTrafoTo(mo,t, tr);
            //            printf("goal  : %.2f %.2f %.2f r: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);
            original.getTrafoTo(mo_opt,t, tr);
            //            printf("result: %.2f %.2f %.2f r: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);


            cl_man.addTrafo(t);

            //            printf("t: %.2f %.2f %.2f \nr: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);

            //            get max distance between corresponding points:
            //            double max_dist = mo.max_point_dist(mo_opt);
            //
            //            ROS_WARN("%i iter, %.2f max_error",iter_cnt[i], max_dist);
            // off << iter_cnt[i] << " " << max_dist << endl;

            sendMarker(marker_pub, optimizer,&mo);
            //            ros::Duration(0.01).sleep();

          }


          //      mo_opt.getPoseFromVertices();
          //
          //      float max_d;
          //      mo.isSameObject(mo_opt,0.1, &max_d);
          //      printf("max_dist: %f\n",max_d);

          // off.close();
        }


        // check for cluster:
        int cl = cl_man.findClusters(0.1,10);
        //    ROS_WARN("found %i cluster", cl);



        // compare goal_pose with medians:
        bool good_track = false;
        for (int i=0; i<cl; ++i){

          float error;
          Eigen::Affine3f med = cl_man.getMed(i);

          Eigen::Affine3f g_;
          xyzRpyToAffine3f(goal_pose,g_);
          bool similar = isSimilar(med,g_,0.1, 10/M_PI*180, &error);

          if (similar){
            //        ROS_INFO("HIT: dist to med %i: %f", i,error);
            good_track = true;
          }
          else{
            //        ROS_INFO("FAIL: dist to med %i: %f", i,error);
          }

        }


        if (good_track){
          ROS_ERROR("found valid track");
          found_track_cnt++;
        }
        else
          ROS_ERROR("Found NO valid track");

        //  for (uint i=0; i<cl_man.clusters.size(); ++i){
        //    for (uint j=0; j<cl_man.clusters[i].size(); ++j){
        //      float t[6];
        //      affine3fToXyzRpy(cl_man.trafos[cl_man.clusters[i][j]],t);
        //      printf("Pose: %i:  %f %f %f %f %f %f \n",i, t[0], t[1], t[2], t[3], t[4], t[5]);
        //    }
        //  }

        printf("goal:  %f %f %f %f %f %f \n", goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5]);
        for (uint i=0; i<cl_man.medians.size(); ++i){
          float t[6];
          affine3fToXyzRpy(cl_man.getMed(i),t);
          printf("Med: %i:  %f %f %f %f %f %f \n",i, t[0], t[1], t[2], t[3], t[4], t[5]);
        }


      } // loop over many start poses


      ROS_ERROR("found valid track for %i of %i trials", found_track_cnt, goal_pose_cnt);
      ROS_ERROR("Time for opt: %i s (%i ms per goal)", int(opt_time_ms/1000.0), int(opt_time_ms*1.0/goal_pose_cnt));


      of_time << start_pose_cnt << " " << g2o_iter_cnt << " ";
      of_time << int(found_track_cnt*100.0/goal_pose_cnt) << " " << int(opt_time_ms*1.0/goal_pose_cnt) << endl;

    }
  }

  of_time.close();

  return 0;
#endif

}


