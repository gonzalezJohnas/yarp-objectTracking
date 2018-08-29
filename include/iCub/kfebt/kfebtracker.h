#ifndef KFEBTRACKER_H
#define KFEBTRACKER_H

#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "trackers/tcbt.h"
#include "trackers/tmosse.h"
#include "trackers/tvdp.h"
#include "trackers/tncc.h"
#include "kfebt.h"

class KFebTracker
{
public:
    KFebTracker();
    void init(std::string initPar);
    void initTrackers(cv::Mat image, cv::Rect region);
    cv::Rect track(cv::Mat image);

    void setThresholdUncertainty(double thr){this->thresholdUncertainty = thr;}
    double getThresholdUncertainty() {return this->thresholdUncertainty;}

 private:
  float ajuste = 0.15;
  double thresholdUncertainty = 3.;
  // Trackers
    tASMS asms;
    tKCF kcf;
    tCBT cbt;
    tVDP vdp;
    tncc ncc;

    std::vector<BTracker*> trackers;

    // Kalman Filter
    KFEBT fusion;

    std::vector<float> uncertainty, trackersResults;


};

#endif // KFEBTRACKER_H
