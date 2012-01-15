/*
 * visualization.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: engelhar
 */



#include "visualization.h"


void sendMarker(ros::Publisher& marker_pub, Optimizer& optimizer){

  visualization_msgs::Marker spheres, line_list;



  spheres.header.frame_id = line_list.header.frame_id = "/mocap_frame";
  spheres.header.stamp = line_list.header.stamp = ros::Time::now();
  spheres.ns = line_list.ns = "mocap_shapes";
  spheres.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  spheres.action = line_list.action = visualization_msgs::Marker::ADD;
  spheres.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  spheres.id = 0;
  line_list.id = 1;



  spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.2;
  line_list.scale.x = 0.05;
  spheres.color.r = spheres.color.a = 1.0;
  line_list.color.g = line_list.color.a = 1.0;


  for (uint i=0; i<optimizer.m_obj->cloud.points.size(); ++i){
    point_type p = optimizer.m_obj->cloud.points[i];
    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = p.z;
    printf("pt %i: %f %f %f \n", i, p.x,p.y,p.z);
    spheres.points.push_back(gp);
  }


  // TODO: does not work for a cam not in 0,0,0,0,0,0,1
  for (uint i=0; i<optimizer.obs.size(); ++i){

    float a = (optimizer.obs[i].x-optimizer.cx)/optimizer.fx;
    float b = (optimizer.obs[i].y-optimizer.cy)/optimizer.fy;
    geometry_msgs::Point p;
    p.x=p.y=p.z=0;
    line_list.points.push_back(p);

    float n = sqrt(a*a+b*b+1);
    float l = 10;
    p.x = a/n*l; p.y = b/n*l; p.z = 1/n*l;
    line_list.points.push_back(p);
  }



//  geometry_msgs::Point p;
//  p.x=p.y=p.z=0;
//  line_list.points.push_back(p);
//  spheres.points.push_back(p);
//  p.z=1;
//  spheres.points.push_back(p);
//  line_list.points.push_back(p);

  marker_pub.publish(spheres);
  marker_pub.publish(line_list);

}
