/*
 * cluster.h
 *
 *  Created on: Jan 17, 2012
 *      Author: engelhar
 */

#ifndef MOCAP_CLUSTER_H_
#define MOCAP_CLUSTER_H_


#include "mocap_defs.h"
#include "optimization.h"
#include "simulation.h"
#include <fstream>



class ClusterManager {


  void reset() {
    trafos.clear();
    medians.clear();
    clusters.clear();
  }




  // returns position of median
  int findMedian(vector<int>& poses);

public:

  // list of all trafos found with different initializations of the optimizer
  vector<Eigen::Affine3f> trafos;
  vector<int> medians; // points to elements in trafos
  vector<vector<int> > clusters; // each cluster points to trafos

  void addTrafo(Eigen::Affine3f& t) {trafos.push_back(t);}
  // find clusters in trafos, returns numer of clusters
  // using agglomerative clustering:
  // start cluster with single pose and add similar poses
  // two poses are similar if max dist in translation is less than d_max
  // and in rotation less than deg_max (in deg!)
  int findClusters(float d_max, float deg_max);


  Eigen::Affine3f getMed(int i){return trafos[medians[i]];}

};






#endif /* CLUSTER_H_ */
