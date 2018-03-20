// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2017  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: jonas.gonzalez@iit.it
  * email: 
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file objectTrackingRateThreadRatethread.cpp
 * @brief Implementation of the eventDriven thread (see objectTrackingRateThreadRatethread.h).
 */

#include "../include/iCub/objectTrackingRateThread.h"
#include <cstring>
#include <utility>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

//********************interactionEngineRatethread******************************************************

objectTrackingRateThread::objectTrackingRateThread() : RateThread(THRATE) {
    robot = "icub";
    trackerType = "CSRT";
}

objectTrackingRateThread::objectTrackingRateThread(yarp::os::ResourceFinder &rf) : RateThread(THRATE) {
    trackerType = rf.check("trackerType",
            Value("CSRT"),
            "tracker type (string)").asString();

}

objectTrackingRateThread::objectTrackingRateThread(string _robot, string _configFile) : RateThread(THRATE) {
    robot = std::move(_robot);
}

objectTrackingRateThread::~objectTrackingRateThread() {
    // do nothing
}

bool objectTrackingRateThread::threadInit() {

    if(!templateInputPort.open(this->name+"/templateImage:i")){
        yInfo("Unable to open /templateImage:i port");
        return false;
    }

    if(!inputImagePort.open(this->name+"/inputImagePort:i")){
        yInfo("Unable to open /inputImagePort:i port");
        return false;
    }

    if(!trackerOutputPort.open(this->name+"/trackerOutputPort:o")){
        yInfo("Unable to open /trackerOutputPort:o port");
        return false;
    }


    setTracker();
    trackingMode = false;

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void objectTrackingRateThread::setName(string str) {
    this->name = std::move(str);
}


std::string objectTrackingRateThread::getName(const char *p) {
    string str(name);
    str.append(p);
    return str;
}

void objectTrackingRateThread::setInputPortName(string InpPort) {

}

void objectTrackingRateThread::run() {

    if(inputImagePort.getInputCount() && trackingMode){


        ImageOf<yarp::sig::PixelBgr> *inputImage = inputImagePort.read(true);
        cv::Mat inputImageMat = cv::cvarrToMat(inputImage->getIplImage());

        const bool successTracking = tracker->update(inputImageMat, currentTrackRect);

        if (successTracking){
            //currentTrackRect = objectTemplateRectToTrack;
            cv::rectangle(inputImageMat, currentTrackRect, cv::Scalar( 255, 0, 0 ), 2, 1 );
            // Display frame.
            IplImage outputImageTrackerIPL = (IplImage) inputImageMat;

            yarp::sig::ImageOf<yarp::sig::PixelRgb> * outputTrackImageIPL = &trackerOutputPort.prepare();
            inputImage->resize(outputImageTrackerIPL.width, outputImageTrackerIPL.height);

            outputTrackImageIPL->wrapIplImage(&outputImageTrackerIPL);
            trackerOutputPort.write();

        }


    }
}


void objectTrackingRateThread::threadRelease() {
    // nothing

}

bool objectTrackingRateThread::setTemplateFromImage() {

    if(templateInputPort.getInputCount() && inputImagePort.getInputCount()){
        ImageOf<yarp::sig::PixelBgr> *templateImage = templateInputPort.read(true);
        const cv::Mat templateMat = cv::cvarrToMat(templateImage->getIplImage());

        ImageOf<yarp::sig::PixelBgr> *inputImage = inputImagePort.read(true);
        const cv::Mat inputImageMat = cv::cvarrToMat(inputImage->getIplImage());

        //objectTemplateRectToTrack = cv::Rect2d(0, 0, templateMat.size().width, templateMat.size().height);
        objectTemplateRectToTrack = cv::selectROI(inputImageMat, false);
        trackingMode = true;
        tracker->init(inputImageMat, objectTemplateRectToTrack);
        cv::destroyAllWindows();

        return  true;
    }

    return false;
}

void objectTrackingRateThread::setTracker() {

    using namespace cv;
    if (trackerType == "CSRT")
        tracker = TrackerCSRT::create()  ;
    if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    if (trackerType == "GOTURN")
        tracker = TrackerGOTURN::create();
    if (trackerType == "MOSSE")
        tracker = TrackerMOSSE::create();

}




