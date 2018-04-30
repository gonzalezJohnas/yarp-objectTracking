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
    trackerType = "KF-EBT";
    enableSaccade = false;

}

objectTrackingRateThread::objectTrackingRateThread(yarp::os::ResourceFinder &rf) : RateThread(THRATE) {
    robot =  rf.check("robot",
                      Value("icub"),
                      "robot name (string)").asString();;

    trackerType = rf.check("trackerType",
            Value("KF-EBT"),
            "tracker type (string)").asString();

    enableSaccade = rf.check("saccade", Value(false)).asBool();

}

objectTrackingRateThread::objectTrackingRateThread(string _robot, string _configFile) : RateThread(THRATE) {
    robot = std::move(_robot);
}

objectTrackingRateThread::~objectTrackingRateThread() {
    // do nothing
}

bool objectTrackingRateThread::threadInit() {

    if(!templateImageInputPort.open(this->name+"/templateImage:i")){
        yInfo("Unable to open /templateImage:i port");
        return false;
    }

    if(!inputImagePort.open(this->name+"/inputImage:i")){
        yInfo("Unable to open /inputImage:i port");
        return false;
    }

    if(!trackerOutputPort.open(this->name+"/trackerOutput:o")){
        yInfo("Unable to open /trackerOutput:o port");
        return false;
    }

    if(!templateImageOutputPort.open(this->name+"/templateOutput:o")){
        yInfo("Unable to open /templateOutput:o port");
        return false;
    }

    if(!inputTargetCoordinate.open(this->name+"/inputTargetCoordinate:i")){
        yInfo("Unable to open /inputTargetCoordinate:i port");
        return false;
    }




    setTracker();
    trackingMode = false;


    previousImagePosX =.0;
    previousImagePosY = 0;
    previousPosZ = 0;

    yInfo("Initialization of the processing thread correctly ended");
    yInfo("Using tracker : %s", trackerType.c_str());
    const bool ret = openIkinGazeCtrl();
    return ret;
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

        ImageOf<yarp::sig::PixelBgr> *inputImage = inputImagePort.read();
        cv::Mat inputImageMat = cv::cvarrToMat(inputImage->getIplImage());

        const bool successTracking = trackingPrediction(inputImageMat, &currentTrackRect);


        if (successTracking) {


            trackIkinGazeCtrl(currentTrackRect);
            cv::Mat outputTemplate = inputImageMat(currentTrackRect);


            if (templateImageOutputPort.getOutputCount()) {
                // Display frame.                
                yarp::sig::ImageOf <yarp::sig::PixelBgr> &outputTemplateImage = templateImageOutputPort.prepare();

                IplImage outputTemplateIPL = outputTemplate;
                outputTemplateImage.resize(outputTemplateIPL.width, outputTemplateIPL.height);

                cvCopy(&outputTemplateIPL, (IplImage *) outputTemplateImage.getIplImage());
                templateImageOutputPort.write();
            }


            if (trackerOutputPort.getOutputCount()) {
                // Display frame.

                cv::rectangle(inputImageMat, currentTrackRect, cv::Scalar(255, 0, 0), 2, 1);
                IplImage outputImageTrackerIPL = inputImageMat;

                yarp::sig::ImageOf <yarp::sig::PixelBgr> *outputTrackImage = &trackerOutputPort.prepare();
                inputImage->resize(outputImageTrackerIPL.width, outputImageTrackerIPL.height);

                outputTrackImage->wrapIplImage(&outputImageTrackerIPL);
                trackerOutputPort.write();
            }
        }





    }
}


void objectTrackingRateThread::threadRelease() {
    yInfo("Releasing thread");
    yInfo("Restoring iKinGaze context");
    iGaze->stopControl();
    iGaze->restoreContext(ikinGazeCtrl_Startcontext);
    clientGaze->close();

    templateImageInputPort.close();
    inputImagePort.close();
    trackerOutputPort.close();
}

bool objectTrackingRateThread::setTemplateFromImage() {

    if(templateImageInputPort.getInputCount() && inputImagePort.getInputCount()){
        ImageOf<yarp::sig::PixelBgr> *templateImage = templateImageInputPort.read(true);
        const cv::Mat templateMat = cv::cvarrToMat(templateImage->getIplImage());

        ImageOf<yarp::sig::PixelBgr> *inputImage = inputImagePort.read(true);
        const cv::Mat inputImageMat = cv::cvarrToMat(inputImage->getIplImage());

        ROITemplateToTrack = cv::selectROI(inputImageMat, false);

        initializeTracker(inputImageMat, ROITemplateToTrack);

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
    if (trackerType == "KF-EBT") {
        kalmanFilterEnsembleBasedTracker.init("AKNC");
    }

}

bool objectTrackingRateThread::openIkinGazeCtrl() {


    //---------------------------------------------------------------
    yDebug("Opening the connection to the iKinGaze");
    Property optGaze; //("(device gazecontrollerclient)");
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/objectTracking/gaze");

    clientGaze = new PolyDriver();
    clientGaze->open(optGaze);
    iGaze = nullptr;
    yDebug("Connecting to the iKinGaze");
    if (!clientGaze->isValid()) {
        return false;
    }

    clientGaze->view(iGaze);
    iGaze->storeContext(&ikinGazeCtrl_Startcontext);

    iGaze->blockNeckRoll(0.0);


    iGaze->setSaccadesMode(enableSaccade);

    //Set trajectory time:
    iGaze->setNeckTrajTime(0.9);
    iGaze->setEyesTrajTime(0.5);
    iGaze->setTrackingMode(true);
    iGaze->setVORGain(1.3);
    iGaze->setOCRGain(1.0);

    yDebug("Initialization of iKingazeCtrl completed");
    return true;
}

bool objectTrackingRateThread::trackIkinGazeCtrl(const cv::Rect2d t_trackZone) {


    // Calcul the center of the Rectangle
    const double imagePositionX = t_trackZone.tl().x + (t_trackZone.width / 2);
    const double imagePositionY = t_trackZone.tl().y + (t_trackZone.height / 2);

    yarp::sig::Vector imageFramePosition(2);
    yarp::sig::Vector rootFramePosition(3);

    imageFramePosition[0] = imagePositionX;
    imageFramePosition[1] = imagePositionY;

    // On the 3D frame reference of the robot the X axis is the depth
    const bool ret = iGaze->get3DPoint(0, imageFramePosition, 1.0 , rootFramePosition );

    // Calcul the Euclidian distance in image plane
    const double distancePreviousCurrent = sqrt(pow((imagePositionX - previousImagePosX),2)  + pow(( imagePositionY - previousImagePosY ),2));

    yInfo("Distance is %f", distancePreviousCurrent);

    if(ret && distancePreviousCurrent > 30 ){

        // Storing the previous coordinate in the Robot frame reference

        previousImagePosX = imagePositionX;
        previousImagePosY = imagePositionY;
        yInfo("Position  is %f %f %f", rootFramePosition[0], rootFramePosition[1], rootFramePosition[2]);

        iGaze->lookAtFixationPoint(rootFramePosition);

    }

    return false;
}

bool objectTrackingRateThread::initializeTracker(const cv::Mat t_image, const cv::Rect2d t_ROIToTrack) {


    currentTrackRect = t_ROIToTrack;
    if(trackerType == "KF-EBT"){

        kalmanFilterEnsembleBasedTracker.initTrackers(t_image, t_ROIToTrack);
        trackingMode = true;
    }

    else {
        tracker->init(t_image, t_ROIToTrack);
        trackingMode = true;

    }
    return false;
}

bool objectTrackingRateThread::trackingPrediction(cv::Mat t_image, cv::Rect2d *t_ROITrackResult) {

    bool ret = false;
    if(tracker){
        ret = tracker->update(t_image, *t_ROITrackResult);
    }

    else if(trackerType == "KF-EBT"){
        *t_ROITrackResult = kalmanFilterEnsembleBasedTracker.track(t_image);
        ret = true;
    }

    return ret;

}

bool objectTrackingRateThread::setTemplateFromCoordinate(const int xMin, const int yMin, const int xMax, const int yMax ) {

    if(inputImagePort.getInputCount()) {

        ImageOf <yarp::sig::PixelBgr> *inputImage = inputImagePort.read(true);
        const cv::Mat inputImageMat = cv::cvarrToMat(inputImage->getIplImage());

        widthInputImage = inputImageMat.cols;
        heightInputImage = inputImageMat.rows;

        ROITemplateToTrack = cv::Rect2d(xMin, yMin, (xMax - xMin), (yMax - yMin));
        initializeTracker(inputImageMat, ROITemplateToTrack);

    }

    return true;
}

bool objectTrackingRateThread::checkROI(cv::Rect2d *t_ROI) {
    if (0 <= t_ROI->x
        && 0 <= t_ROI->width
        && t_ROI->x + t_ROI->width <= widthInputImage
        && 0 <= t_ROI->y
        && 0 <= t_ROI->height
        && t_ROI->y + t_ROI->height <= heightInputImage){

        return false;
    } else{
        return true;
    }
}





