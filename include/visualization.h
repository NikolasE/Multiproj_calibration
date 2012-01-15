#ifndef VISUALIZATION_MOCAP_H
#define VISUALIZATION_MOCAP_H


#include "mocap_defs.h"
#include "optimization.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



void sendMarker(ros::Publisher& marker_pub, Optimizer& optimizer);





#endif
