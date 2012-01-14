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

#include "mocap_defs.h"
#include "simulation.h"
using namespace sensor_msgs;

ros::NodeHandle* nh;


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

  Mocap_object mo;
  createRect(&mo, 10, 20,5);
  mo.printObject();







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
}
