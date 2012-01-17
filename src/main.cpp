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



  //  ros::Time::init();
  //
  //
  //  //  checkStartingPose();
  //  return -1;


  ros::init(argc, argv, "g2o_mocap");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);




  //  assert(argc>1);

  Simulator sim;
  Optimizer optimizer;
  Mocap_object mo;
  Mocap_object mo_opt;




  //  sim.createTriangle(&mo_fix);
  //  optimizer.setObject(&mo);
  //  optimizer.testVertex2Edge();
  //  mo.getPoseFromVertices();


  //  float z = 10;
  //    for (float z = 1; z<=30; z++)
  {
    //      printf("z: %f \n", z);



    vector<int> iter_cnt;
    for (int i=0; i<=20; ++i) iter_cnt.push_back(i);
    for (int i=25; i<100; i+=10) iter_cnt.push_back(i);
    for (int i=100; i<=700; i+=100) iter_cnt.push_back(i);
    //    iter_cnt.push_back(10000);



    srand ( time(NULL) );

    // 1.5m in all directions
    float D = 3;
    float dx = D*(rand()*1.0/RAND_MAX-0.5);
    float dy = D*(rand()*1.0/RAND_MAX-0.5);
    float dz = D*(rand()*1.0/RAND_MAX-0.5);

    // 90deg in each direction
    float D_rad = M_PI;
    float d_roll  = D_rad*(rand()*1.0/RAND_MAX-0.5);
    float d_pitch = D_rad*(rand()*1.0/RAND_MAX-0.5);
    float d_yaw   = D_rad*(rand()*1.0/RAND_MAX-0.5);

    //    dx = dy = dz = 0;
    //    d_roll = d_pitch = d_yaw = 0;


    printf("Start pose: t: %.2f %.2f %.2f \nr: %.2f %.2f %.2f \n ", dx,dy,dz,d_roll/M_PI*180, d_pitch/M_PI*180, d_yaw/M_PI*180);

    ofstream off;
    off.open("/home/engelhar/ros/mocap/error.txt");


    // same pertubation for all iterations
    double sigma = 3;
    double err_dx = Simulator::getGaussianSample(0,sigma); // zero centered normal
    double err_dy = Simulator::getGaussianSample(0,sigma);

    ROS_WARN("observation moved by %f %f", dx,dy);


    for(uint i=0; i<iter_cnt.size(); ++i)
    {

      printf("OPTIMIZING WITH %i ITERATIONS \n",iter_cnt[i]);

      sim.createRect(&mo, 5, 3, 2);
      sim.createRect(&mo_opt, 5, 3, 2);

      //      sim.createTriangle(&mo);
      //      sim.createTriangle(&mo_opt);

      // constant starting pose
      sim.trafoObject(&mo_opt,0,0,15,0,0,0);
      // random goal pose
      sim.trafoObject(&mo,dx,dy,5+dz,d_roll,d_pitch,d_yaw);


      std::vector<CvPoint2D32f> prj = sim.computeProjections(&mo, false);


//      if (true){
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
      optimizer.setOberservations(prj);
      optimizer.setObject(&mo_opt);
      optimizer.InitOptimization();



      ros::Time begin = ros::Time::now();

      optimizer.optimize(iter_cnt[i]);

      //      optimizer.optimize(atoi(argv[1]));
      ros::Time end = ros::Time::now();

      printf("opt took %i ms\n", int((end-begin).toNSec()/1000/1000));

      mo_opt.getPoseFromVertices();
      sendMarker(marker_pub, optimizer);

      Eigen::Affine3f t;
      float tr[6];
      mo.getTrafoTo(mo_opt,t, tr);

      printf("t: %.2f %.2f %.2f \nr: %.2f %.2f %.2f \n", tr[0],tr[1],tr[2],tr[3]/M_PI*180,tr[4]/M_PI*180,tr[5]/M_PI*180);

      // get max distance between corresponding points:
      double max_dist = mo.max_point_dist(mo_opt);

      ROS_WARN("%i iter, %.2f max_error",iter_cnt[i], max_dist);
      off << iter_cnt[i] << " " << max_dist << endl;

      sendMarker(marker_pub, optimizer, &mo);

      ros::Duration(0.05).sleep();

    }


    //      mo_opt.getPoseFromVertices();
    //
    //      float max_d;
    //      mo.isSameObject(mo_opt,0.1, &max_d);
    //      printf("max_dist: %f\n",max_d);

    off.close();
  }


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
