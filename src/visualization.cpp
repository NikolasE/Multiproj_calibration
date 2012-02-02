/*
 * visualization.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: engelhar
 */



#include "visualization.h"





int sendProjectionRays(ros::Publisher& marker_pub, Camera& cam, int id, string ns, float r, float g, float b){

  visualization_msgs::Marker rays;

  rays.header.frame_id = "/mocap_frame";
  rays.ns = ns;
  rays.header.stamp = ros::Time::now();
  rays.pose.orientation.w = 1.0;
  rays.id = id;


  // true: show only observations in the z=1-plane
  // false: show ray from cam-center through the observations
  bool show_observations = false;

  rays.action = visualization_msgs::Marker::ADD;

  if (show_observations){
    rays.type = visualization_msgs::Marker::SPHERE_LIST;
    rays.scale.x = rays.scale.y = rays.scale.z = 0.02;
  }else{
    rays.type = visualization_msgs::Marker::LINE_LIST;
    rays.scale.x = rays.scale.y = rays.scale.z = 0.01;
  }


  rays.color.a = 1;
  rays.color.r = r; rays.color.g = g; rays.color.b = b;


  Eigen::Affine3f c2w = cam.pose.inverse();


  for (Observations::iterator it = cam.obs.begin(); it !=cam.obs.end(); ++it){
    Eigen::Vector3f ev;
    geometry_msgs::Point p;
    if (!show_observations){
      ev = cam.pose*Eigen::Vector3f(0,0,0);
      p.x = ev.x(); p.y = ev.y();p.z = ev.z();
      rays.points.push_back(p);
    }

    // z = 1;
    float a = (it->second.x-cam.c_x)/cam.f_x;
    float b = (it->second.y-cam.c_y)/cam.f_y;

    float n,l;
    if (show_observations)
      n = l = 1;
    else {
      n = sqrt(a*a+b*b+1);
      l = 10; // length of projection ray
    }
    ev.x() = a/n*l; ev.y() = b/n*l; ev.z() = 1/n*l;

    ev = cam.pose*ev;
    p.x = ev.x(); p.y = ev.y(); p.z = ev.z();
    rays.points.push_back(p);
  }


  marker_pub.publish(rays);

  return id+1;
}



int sendCam(ros::Publisher& marker_pub, Camera& cam, int id, string ns, float r, float g, float b){

  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "/mocap_frame";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = ns;
  arrow.pose.orientation.w = 1.0;
  arrow.action = visualization_msgs::Marker::ADD;

  arrow.type = visualization_msgs::Marker::LINE_LIST;

  arrow.id = id;


  arrow.scale.x = 0.01; // line width
  arrow.color.a = 1; arrow.color.r = r; arrow.color.g = g; arrow.color.b = b;



  Eigen::Affine3f pose = cam.pose;
  geometry_msgs::Point gp,gp2;


  // compute corners of field of view
  CvPoint2D32f corners[4]; // pos in pixel coordinates

  corners[0].x = 0;
  corners[0].y = 0;

  corners[1].x = cam.c_width;
  corners[1].y = 0;

  corners[2].x = cam.c_width;
  corners[2].y = cam.c_height;

  corners[3].x = 0;
  corners[3].y = cam.c_height;

  Eigen::Vector3f d3_pos[4]; // has to be eigen for pose-multiplication
  float Z = 1;

  for (uint i=0; i<4; ++i){
    d3_pos[i].z() = Z;
    d3_pos[i].x() = (corners[i].x - cam.c_x)*d3_pos[i].z()/cam.f_x;
    d3_pos[i].y() = (corners[i].y - cam.c_y)*d3_pos[i].z()/cam.f_y;

    d3_pos[i] = pose*d3_pos[i];
  }




  // origin of camera
  Eigen::Vector3f p(0,0,0);
  p = pose*p;
  gp.x = p.x(); gp.y = p.y(); gp.z = p.z();

  // cam to extremal points
  for (uint i=0; i<4; ++i){
    arrow.points.push_back(gp);
    gp2.x = d3_pos[i].x(); gp2.y = d3_pos[i].y(); gp2.z = d3_pos[i].z();
    arrow.points.push_back(gp2);
  }

  // edges of field of view
  for (uint i=0; i<4; ++i){
    gp2.x = d3_pos[i].x(); gp2.y = d3_pos[i].y(); gp2.z = d3_pos[i].z();
    arrow.points.push_back(gp2);
    gp2.x = d3_pos[(i+1)%4].x(); gp2.y = d3_pos[(i+1)%4].y(); gp2.z = d3_pos[(i+1)%4].z();
    arrow.points.push_back(gp2);

  }

  marker_pub.publish(arrow);

  return id+1;
}


int sendObject(ros::Publisher& marker_pub, int id, Mocap_object& obj, string ns, float r, float g, float b)
{

  visualization_msgs::Marker vertices, connections;

  vertices.header.frame_id = "/mocap_frame";
  vertices.header.stamp = ros::Time::now();
  vertices.ns = ns;
  vertices.pose.orientation.w = 1.0;
  vertices.action = visualization_msgs::Marker::ADD;
  connections = vertices;
  vertices.type = visualization_msgs::Marker::SPHERE_LIST;
  connections.type = visualization_msgs::Marker::LINE_LIST;

  vertices.id = id;
  connections.id = id+1;


  vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.02;
  vertices.color.a = 1;
  vertices.color.r = r; vertices.color.g = g; vertices.color.b = b;

  connections.scale.x = 0.01;
  connections.color.a = 1;
  connections.color.r = r; connections.color.g = g; connections.color.b = b;


  uint vertex_cnt = obj.points.size();
  geometry_msgs::Point gp;
  // add all vertices
  for (uint i=0; i<vertex_cnt; ++i){
    if (obj.point_valid[i]){
      Eigen::Vector3f p = obj.points[i];
      gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
      vertices.points.push_back(gp);
    }
  }

  // and also all connections
  for (uint i=0; i<vertex_cnt-1; ++i){
    if (!obj.point_valid[i]) continue;

    Eigen::Vector3f p = obj.points[i];
    gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
    for (uint j=i+1; j<vertex_cnt; ++j){
      if (!obj.point_valid[j]) continue;

      connections.points.push_back(gp);
      p = obj.points[j];
      geometry_msgs::Point gp2; gp2.x = p.x(); gp2.y = p.y(); gp2.z = p.z();
      connections.points.push_back(gp2);
    }
  }


  marker_pub.publish(vertices);
  marker_pub.publish(connections);

  return id+2;

}

void sendMarker(ros::Publisher& marker_pub, Optimizer& optimizer, Mocap_object* fixed){

  int id = 0;

  //  id = sendProjectionRays(marker_pub,optimizer, id, "ns_projections", 0,0,1); // blue
  id = sendObject(marker_pub,id, *optimizer.m_obj, "ns_moved_object", 1,0,0); // red

  if (fixed!=NULL){
    id = sendObject(marker_pub,id, *fixed, "ns_static_object", 0,1,0); // green
  }



}







//  visualization_msgs::Marker projections;
//
//
//
//  projections.header.frame_id = proj_ll.header.frame_id = obj_ll.header.frame_id  =  "/mocap_frame";
//  spheres.header.stamp = proj_ll.header.stamp = obj_ll.header.stamp = ros::Time::now();
//
//  spheres.ns = proj_ll.ns = "mocap_shapes";
//  obj_ll.ns = "foo_shapes";
//
//  spheres.pose.orientation.w = proj_ll.pose.orientation.w = obj_ll.pose.orientation.w = 1.0;
//  spheres.action = proj_ll.action = obj_ll.action =  visualization_msgs::Marker::ADD;
//  spheres.type = visualization_msgs::Marker::POINTS;
//  proj_ll.type = visualization_msgs::Marker::LINE_LIST;
//  obj_ll.type = visualization_msgs::Marker::LINE_LIST;
//
//  foo = spheres;
//
//
//
//  spheres.id = 0;
//  proj_ll.id = 1;
//  obj_ll.id = 2;
//
//
//
//  spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.2;
//  spheres.color.r = spheres.color.a = 1.0;
//
//  proj_ll.scale.x = 0.05;
//  proj_ll.color.g = proj_ll.color.a = 1.0;
//
//  obj_ll.scale.x = 0.02;
//  obj_ll.color.r = obj_ll.color.a = 1.0;
//
//
//  uint vertex_cnt = optimizer.m_obj->cloud.points.size();
//
//  for (uint i=0; i<vertex_cnt; ++i){
//    point_type p = optimizer.m_obj->cloud.points[i];
//    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = p.z;
////    printf("pt %i: %f %f %f \n", i, p.x,p.y,p.z);
//    spheres.points.push_back(gp);
//    foo.points.push_back(gp);
//  }
//
//  for (uint i=0; i<vertex_cnt-1; ++i)
//    for (uint j=i+1; j<vertex_cnt; ++j){
//    point_type p = optimizer.m_obj->cloud.points[i];
//    geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = p.z;
//    obj_ll.points.push_back(gp);
//
//    p = optimizer.m_obj->cloud.points[j];
//    gp.x = p.x; gp.y = p.y; gp.z = p.z;
//    obj_ll.points.push_back(gp);
//  }
//
//
//  // TODO: does not work for a cam not in 0,0,0,0,0,0,1
//  for (uint i=0; i<optimizer.obs.size(); ++i){
//
//    float a = (optimizer.obs[i].x-optimizer.cx)/optimizer.fx;
//    float b = (optimizer.obs[i].y-optimizer.cy)/optimizer.fy;
//    geometry_msgs::Point p;
//    p.x=p.y=p.z=0;
//    proj_ll.points.push_back(p);
//
//    float n = sqrt(a*a+b*b+1);
//    float l = 10;
//    p.x = a/n*l; p.y = b/n*l; p.z = 1/n*l;
//    proj_ll.points.push_back(p);
//  }
//
//
//  marker_pub.publish(foo);
//  marker_pub.publish(proj_ll);
//  marker_pub.publish(obj_ll);
//}
