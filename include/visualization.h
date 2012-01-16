#ifndef VISUALIZATION_MOCAP_H
#define VISUALIZATION_MOCAP_H


#include "mocap_defs.h"
#include "optimization.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



void sendMarker(ros::Publisher& marker_pub, Optimizer& optimizer, Mocap_object* fixed = NULL);

// returns next free id (=id+2)
int sendObject(ros::Publisher& marker_pub, int id, Mocap_object& obj, string ns);

// returns next free id (=id+1)
int sendProjectionRays(ros::Publisher& marker_pub, Optimizer& optimizer, int id, string ns, float r, float g, float b);

#endif
