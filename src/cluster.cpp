/*
 * cluster.cpp
 *
 *  Created on: Jan 17, 2012
 *      Author: engelhar
 */

#include "cluster.h"
#include "evaluation.h"


int ClusterManager::findMedian(vector<int>& poses){

  int med = -1;
  float min_error = 0;
  for (uint i=0; i<poses.size(); ++i){

    float error = 0;
    // sum error to all other poses
    for (uint j=0; j<poses.size(); ++j){
      float current_error;
      isSimilar(trafos[poses[i]],trafos[poses[j]],100,100, &current_error);
      // ROS_INFO("error between %i and %i: %f",poses[i],poses[j],current_error);
      error += current_error;
    }

    if (med<0 || error < min_error){
      med = poses[i];
      min_error = error;
    }

  }

  ROS_INFO("med: %i, min error: %f",med, min_error );

  return med;
}


int ClusterManager::findClusters(float d_max, float deg_max){
  medians.clear();

  if(trafos.size()==0){
    ROS_WARN("findClusters with no trafos to cluster!");
    return 0;
  }

  if (trafos.size() == 1){
    medians.push_back(0);
    ROS_WARN("findClusters with only one trafo!");
    return 0;
  }


  int pose_cnt = (int) trafos.size();

  // agglomerative clustering

  // neighbour[i] consists of all nodes (with higher id) that are close to node i
  set<int> neighbour[pose_cnt];

  // for each pose find all poses which have similar pose (O(n^2)..)
  // dist is symmetric
  for (int i=0; i<pose_cnt-1; ++i){
    set<int> n;
    for (int j=i+1; j<pose_cnt; ++j){

      bool similar = isSimilar(trafos[i], trafos[j],d_max, deg_max/M_PI*180);

      if (similar) {
        n.insert(j);
        //        ROS_INFO("close: %i %i", i,j);
      }
    }
    neighbour[i] = n;
  }

  // cluster [i] contains cluster_id of pose i
  int cluster[pose_cnt];
  for (int i=0; i<pose_cnt; ++i) cluster[i] = -1;


  // find connected components
  for (int i=0; i<pose_cnt; ++i){

    if (cluster[i] > -1) continue;

    vector<int> new_cluster;
    int new_cluster_id = clusters.size();
    cluster[i] = new_cluster_id;


    //    ROS_INFO("starting cluster %i with pose %i", new_cluster_id, i);

    // single observation in this cluster
    if (neighbour[i].size() == 0){
      new_cluster.push_back(i);
      clusters.push_back(new_cluster);
      continue;
    }


    stack<int> stack_;
    for (set<int>::iterator it = neighbour[i].begin(); it != neighbour[i].end(); ++it){
      //      ROS_INFO("adding %i to initial stack", *it);
      stack_.push(*it);
    }

    new_cluster.push_back(i);

    set<int> visited;
    while (!stack_.empty()){

      // get next neighbour
      int n = stack_.top(); stack_.pop();

      // this neighbour should not be assigned yet to a different cluster
      //      assert (cluster[n] == -1 || cluster[n] == new_cluster_id);

      if (visited.find(n) != visited.end()) continue;

      cluster[n] = new_cluster_id;
      new_cluster.push_back(n);
      visited.insert(n);

      for (set<int>::iterator it = neighbour[n].begin(); it != neighbour[n].end(); ++it){
        if (visited.find(*it) != visited.end()) continue;
        //        ROS_INFO("adding to stack: %i", *it);
        stack_.push(*it);
      }
    }

    clusters.push_back(new_cluster);
  }


  for (uint i=0; i<clusters.size(); ++i){
    cout << "cluster " << i << " ( " <<clusters[i].size()<< " ) ";
    for (uint j=0; j<clusters[i].size(); j++){
      cout << " " << clusters[i][j];
    }
    cout << endl;

    int med = findMedian(clusters[i]);
    medians.push_back(med);

  }

  return medians.size();
}
