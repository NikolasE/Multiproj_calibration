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
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


#include "mocap_defs.h"
#include "simulation.h"
#include "optimization.h"
#include "visualization.h"
#include "cluster.h"
#include "evaluation.h"


#include <algorithm>

#include <time.h>
#include <fstream>



using namespace sensor_msgs;

void mouseHandler(int event, int x, int y, int flags, void* param){


  IplImage* img = (IplImage*) param;

  if (img->nChannels == 1){
    cout << "value " << cvGet2D(img,y,x).val[0] << endl;
  }
  else
  {
    CvScalar c = cvGet2D(img,y,x);
    cout << "col: " << c.val[0] << " " << c.val[1] << " " << c.val[2] << endl;
  }
}

void markGreen(IplImage* img){
  //  for (int x=0; x<img->width; x++)
  //    for (int y=0; y<img->height; y++){
  //      CvScalar c = cvGet2D(img,y,x);
  //
  //      double b = c.val[0];
  //      double g = c.val[1];
  //      double r = c.val[2];
  //
  //     if (b>120 && g > 120 && r < (g+b)/2.0-50)
  ////      if (b < (r+g)/(2+2.5) && r > 140 && g > 160)
  //        cvSet2D(img,y,x,cvScalar(255,0,0  ));
  //
  //    }


}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;

  IplImage *cv_image = bridge.imgMsgToCv(msg, "mono8"); // bgr8
  try
  {
    cvSetMouseCallback("view", mouseHandler, cv_image);




    //    markGreen(cv_image);

    cvThreshold(cv_image,cv_image,250,255,CV_THRESH_BINARY);

    cvShowImage("view", cv_image);
    cvWaitKey(0);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}




int  main (int argc, char** argv)
{


  srand ( time(NULL) );
  //  ros::Time::init();
  //
  //
  //  //  checkStartingPose();
  //  return -1;


  ros::init(argc, argv, "g2o_mocap");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);




  //  assert(argc>1);





  //  sim.createTriangle(&mo_fix);
  //  optimizer.setObject(&mo);
  //  optimizer.testVertex2Edge();
  //  mo.getPoseFromVertices();


  vector<int> iter_cnt;
  //  for (int i=0; i<=20; i+=2) iter_cnt.push_back(i);
  //  for (int i=25; i<100; i+=20) iter_cnt.push_back(i);
  //  for (int i=100; i<=1000; i+=150) iter_cnt.push_back(i);
  iter_cnt.push_back(1000);


  // random goal pose
  float goal_pose[6];
  float init_pose[6]  = {0,0,10,0,0,0};
  float start_pose[6] = {0,0, 5,0,0,0};


  int goal_pose_cnt = 100;
  int found_track_cnt = 0;

  for (int gpc = 0; gpc<goal_pose_cnt; gpc++){


    Simulator sim;
    Optimizer optimizer;
    Mocap_object mo;
    Mocap_object mo_opt;
    ClusterManager cl_man;


    // goal pose is created once
    Simulator::createRandomPose(3,M_PI, start_pose, goal_pose);
    sim.createRect(&mo, 5, 3, 2);
    sim.trafoObject(&mo,goal_pose);

    for (int start_pose_cnt = 0; start_pose_cnt<20; start_pose_cnt++)
    {


      ROS_WARN("Init with start_pose %i", start_pose_cnt);


      Simulator::createRandomPose(3,M_PI, init_pose, init_pose);

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

        // for each optimization, moved object is positioned at the same pose
        sim.createRect(&mo_opt, 5, 3, 2);
        sim.trafoObject(&mo_opt,init_pose);


        printf("OPTIMIZING WITH %i ITERATIONS \n",iter_cnt[i]);

        // z = 5, w = 3, h = 2


        //      sim.createTriangle(&mo);
        //      sim.createTriangle(&mo_opt);


        // try different starting poses for optimization
        //      if (start_pose == 0)
        //        sim.trafoObject(&mo_opt,0,0,10,0,0,0);
        //      if (start_pose == 1)
        //        sim.trafoObject(&mo_opt,0,0,10,M_PI,0,0);
        //      if (start_pose == 2)
        //        sim.trafoObject(&mo_opt,0,0,10,0,M_PI,0);
        //      if (start_pose == 3)
        //        sim.trafoObject(&mo_opt,0,0,10,0,0,M_PI);





        Observations obs = sim.computeProjections(&mo, false);

        //
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

        optimizer.setCamParams(sim.c_x,sim.c_y,sim.f_x, sim.f_y);
        optimizer.setOberservations(obs);
        optimizer.setObject(&mo_opt);
        optimizer.InitOptimization();



        ros::Time begin = ros::Time::now();

        optimizer.optimize(iter_cnt[i]);

        //      optimizer.optimize(atoi(argv[1]));
        ros::Time end = ros::Time::now();

        printf("opt took %i ms\n", int((end-begin).toNSec()/1000/1000));

        mo_opt.getPoseFromVertices();
//        sendMarker(marker_pub, optimizer);

        Eigen::Affine3f t;
        float tr[6];
        mo.getTrafoTo(mo_opt,t, tr);


        cl_man.addTrafo(t);

        printf("t: %.2f %.2f %.2f \nr: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);

        // get max distance between corresponding points:
        double max_dist = mo.max_point_dist(mo_opt);

        ROS_WARN("%i iter, %.2f max_error",iter_cnt[i], max_dist);
       // off << iter_cnt[i] << " " << max_dist << endl;


        ros::Duration(0.1).sleep();

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
    ROS_WARN("found %i cluster", cl);



    // compare goal_pose with medians:
    bool good_track = false;
    for (int i=0; i<cl; ++i){

      float error;
      Eigen::Affine3f med = cl_man.getMed(i);

      float g[6] = {0,0,0,0,0,0}; // poses are from optimal pose to converged pose (best case: 0,0,0,0,0,0)
      Eigen::Affine3f g_; xyzRpyToAffine3f(g,g_);
      bool similar = isSimilar(med,g_,0.1, 10/M_PI*180, &error);

      if (similar){
        ROS_INFO("HIT: dist to med %i: %f", i,error);
        good_track = true;
      }
      else
        ROS_INFO("FAIL: dist to med %i: %f", i,error);

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

    for (uint i=0; i<cl_man.medians.size(); ++i){
      float t[6];
      affine3fToXyzRpy(cl_man.getMed(i),t);
      printf("Med: %i:  %f %f %f %f %f %f \n",i, t[0], t[1], t[2], t[3], t[4], t[5]);
    }


  } // loop over many start poses


  ROS_ERROR("found valid track for %i of %i trials", found_track_cnt, goal_pose_cnt);

  //  printf("Goal:   %f %f %f %f %f %f \n", goal_pose[0],goal_pose[1],goal_pose[2],goal_pose[3],goal_pose[4],goal_pose[5]);


  // TODO: ugly
  //  while (ros::ok())
  //    sendMarker(marker_pub, optimizer, &mo);



}




//  ros::init(argc, argv, "led_finder");
//
//  nh = new ros::NodeHandle();
//
//  image_transport::ImageTransport it(*nh);
//
////  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_img(*nh,"/camera/rgb/image_color", 5);
//  image_transport::Subscriber sub = it.subscribe("/camera/ir/image_rect", 1, imageCb);
//
//  cvNamedWindow("view");
//  cvStartWindowThread();
//
//
//  ros::spin();
//  cvDestroyWindow("view");
