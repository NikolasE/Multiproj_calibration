/*
 * extrinsic_calibration.h
 *
 *  Created on: Feb 9, 2012
 *      Author: engelhar
 */

#ifndef EXTRINSIC_CALIBRATION_H_
#define EXTRINSIC_CALIBRATION_H_

#include "groundtruth.h"
#include "optimization.h"
#include "simulation.h"

struct ExtrinsicCalibration {

  void setBagFile(string filename){bagFileName = filename;}
  void setPropFile(string filename){propFileName = filename;}
  void setSimFile(string filename){simPropFileName = filename; simProp_set = true;}
  void addCameras(vector<Camera>* cams){cams_ = cams;}


  void init();

  void run(int file_skip, int step);



  string bagFileName, propFileName, simPropFileName;

  ExtrinsicCalibration(){
    simProp_set = false;
    cout << bagFileName.length() << endl;
  }


private:
  Groundtruth gt;
  bool simProp_set;
  vector<Camera>* cams_;
  Simulator sim_;


};




#endif /* EXTRINSIC_CALIBRATION_H_ */
