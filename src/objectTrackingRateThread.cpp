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

#include <cstring>
#include <utility>

#include <iCub/objectTrackingRateThread.h>
#include <sys/stat.h>
#include <yarp/cv/Cv.h>

#include "eco/eco.hpp"
#include "eco/parameters.hpp"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 50 //ms

//********************interactionEngineRatethread******************************************************

objectTrackingRateThread::objectTrackingRateThread() : RateThread(THRATE) {
    robot = "icub";
    trackerType = "KF-EBT";
    enableSaccade = false;



}

objectTrackingRateThread::objectTrackingRateThread(yarp::os::ResourceFinder &rf) : RateThread(THRATE) {
    robot = rf.check("robot",
                     Value("icub"),
                     "robot name (string)").asString();;

    trackerType = rf.check("trackerType",
                           Value("ECO"),
                           "tracker type (string)").asString();

    enableSaccade = rf.check("saccade", Value(false)).asBool();

    doHabituation = rf.check("habituation", Value(false)).asBool();
    habituationCpt = rf.check("habituation_cpt", Value(25), "habituation threshold in seconds").asInt();

    enableTracking = rf.check("tracking", Value(true)).asBool();
    drivingCamera = rf.check("camera",
                             Value("0"),
                             "driving camera (string)").asInt();

    const double thresholdUncertaintyTracker = rf.check("threshold_tracker", Value(5.0)).asDouble();
    kalmanFilterEnsembleBasedTracker.setThresholdUncertainty(thresholdUncertaintyTracker);

//    ecoParameters.max_score_threshhold = 0.15;

    azimuth = -99;
    elevation = -99;
    vergence = -99;

}

objectTrackingRateThread::~objectTrackingRateThread() = default ;

bool objectTrackingRateThread::threadInit() {

    if (!templateImageInputPort.open(this->name + "/templateImage:i")) {
        yInfo("Unable to open /templateImage:i port");
        return false;
    }

    if (!inputImagePort.open(this->name + "/inputImage:i")) {
        yInfo("Unable to open /inputImage:i port");
        return false;
    }

    if (!trackerOutputPort.open(this->name + "/trackerOutput:o")) {
        yInfo("Unable to open /trackerOutput:o port");
        return false;
    }

    if (!templateImageOutputPort.open(this->name + "/templateOutput:o")) {
        yInfo("Unable to open /templateOutput:o port");
        return false;
    }

    if (!inputTargetCoordinate.open(this->name + "/inputTargetCoordinate:i")) {
        yInfo("Unable to open /inputTargetCoordinate:i port");
        return false;
    }

    if (!anglePositionPort.open(getName("/anglePositionPort:i"))) {
        yInfo("Unable to open port /anglePositionPort");
        return false;  // unable to open; let RFModule know so that it won't run
    }






    setTracker();
    trackingState = false;

    previousImagePosX = .0;
    previousImagePosY = 0;

    yInfo("Initialization of the processing thread correctly ended");
    yInfo("Using tracker : %s", trackerType.c_str());

    logFileName = logPath + "/data_tracker.csv";
    enableLog = false;
    writeHeader = true;
    counterFile = 0;
    frequencyAcquisitionCounter = 0;
    new_interaction_counter = 0;

    currentTime = 0;

    if(enableTracking) {    return openIkinGazeCtrl();    }

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

void objectTrackingRateThread::run() {

    if (inputImagePort.getInputCount() && trackingState) {
        timeDiff = timer.now() - currentTime;

        ImageOf<yarp::sig::PixelBgr> *inputImage = inputImagePort.read(true);
        inputImageMat = yarp::cv::toCvMat(*inputImage);

        const bool successTracking = trackingPrediction(inputImageMat, currentTrackRect);

        if (successTracking  ) {

            if(doHabituation && timeDiff < habituationCpt){

                yInfo("TimeDiff is %f", timeDiff);
                yInfo("Stop tracking because of fixed target");
                stopTracking();
            }

            if(enableTracking){

                bool targetMoove = trackIkinGazeCtrl(currentTrackRect);
            }

            if (enableLog && frequencyAcquisitionCounter > -1 ) {
                logTrack(inputImageMat, currentTrackRect);
                frequencyAcquisitionCounter = 0;
            } else {
                ++frequencyAcquisitionCounter;
            }

            if (templateImageOutputPort.getOutputCount()) {

                templateOutputMat = inputImageMat(currentTrackRect).clone();
                // Display frame.

                yarp::sig::ImageOf<yarp::sig::PixelBgr> &outputTemplateImage = templateImageOutputPort.prepare();


                outputTemplateImage = yarp::cv::fromCvMat<PixelBgr>(templateOutputMat);
                templateImageOutputPort.write();

            }

            if (trackerOutputPort.getOutputCount()) {
                // Display frame.
                outputMat = inputImageMat.clone();

                cv::rectangle(outputMat, currentTrackRect, cv::Scalar(255, 0, 0), 2, 1);
                yarp::sig::ImageOf<yarp::sig::PixelBgr> &outputTrackImage = trackerOutputPort.prepare();

                outputTrackImage =  yarp::cv::fromCvMat<PixelBgr>(outputMat);

                trackerOutputPort.write();
            }

        }

        else{
            yInfo("Loose target stop tracking ");

            stopTracking();
        }

    }

}

void objectTrackingRateThread::threadRelease() {
    yInfo("Releasing thread");
    yInfo("Restoring iKinGaze context");
    iGaze->stopControl();
    iGaze->restoreContext(ikinGazeCtrl_Startcontext);
    clientGaze->close();

    this->anglePositionPort.close();

    templateImageInputPort.close();
    inputImagePort.close();
    trackerOutputPort.close();
}



void objectTrackingRateThread::setTracker() {

    using namespace cv;
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

    if (trackerType == "KF-EBT") {
        kalmanFilterEnsembleBasedTracker.init("ACK");
        yInfo("Using ensemble based tracker with threshold uncertainty %f",
              kalmanFilterEnsembleBasedTracker.getThresholdUncertainty());

    }


}

bool objectTrackingRateThread::openIkinGazeCtrl() {


    //---------------------------------------------------------------
    yInfo("Opening the connection to the iKinGaze");
    Property optGaze; //("(device gazecontrollerclient)");
    optGaze.put("device", "gazecontrollerclient");
    optGaze.put("remote", "/iKinGazeCtrl");
    optGaze.put("local", "/objectTracking/gaze");

    clientGaze = new PolyDriver();
    clientGaze->open(optGaze);
    iGaze = nullptr;
    yInfo("Connecting to the iKinGaze");
    if (!clientGaze->isValid()) {
        return false;
    }

    clientGaze->view(iGaze);
    iGaze->storeContext(&ikinGazeCtrl_Startcontext);


    iGaze->setSaccadesMode(enableSaccade);
    //Set trajectory time:
    iGaze->blockNeckRoll(0.0);
    iGaze->clearNeckPitch();

//    iGaze->setNeckTrajTime(0.5);
//    iGaze->setEyesTrajTime(0.2);
//    iGaze->setTrackingMode(true);
//    iGaze->setVORGain(1.3);
//    iGaze->setOCRGain(0.7);

    iGaze->storeContext(&gaze_context);

    yInfo("Initialization of iKingazeCtrl completed");
    return true;
}

bool objectTrackingRateThread::trackIkinGazeCtrl(const cv::Rect2d &t_trackZone) {


    // Calcul the center of the Rectangle
    const double imagePositionX = t_trackZone.tl().x + (t_trackZone.width / 2);
    const double imagePositionY = t_trackZone.tl().y + (t_trackZone.height / 2);

    yarp::sig::Vector imageFramePosition(2);
    yarp::sig::Vector rootFramePosition(3);

    imageFramePosition[0] = imagePositionX;
    imageFramePosition[1] = imagePositionY;

    // On the 3D frame reference of the robot the X axis is the depth
    iGaze->get3DPoint(0, imageFramePosition, 1.0, rootFramePosition);

    // Calcul the Euclidian distance in image plane
    const double distancePreviousCurrent = sqrt(
            pow((imagePositionX - previousImagePosX), 2) + pow((imagePositionY - previousImagePosY), 2));

    yInfo("Distance is %f", distancePreviousCurrent);

    previousImagePosX = imagePositionX;
    previousImagePosY = imagePositionY;

    if (distancePreviousCurrent > 20 ) {

        // Storing the previous coordinate in the Robot frame reference

        yInfo("Position  is %f %f %f", rootFramePosition[0], rootFramePosition[1], rootFramePosition[2]);
//        iGaze->lookAtFixationPoint(rootFramePosition);

        iGaze->lookAtMonoPixel(drivingCamera, imageFramePosition );

       return true;

    }

    return false;
}

bool objectTrackingRateThread::initializeTracker(const cv::Mat t_image, const cv::Rect2d t_ROIToTrack) {


    if(enableTracking) iGaze->restoreContext(gaze_context);


    currentTrackRect = t_ROIToTrack;
    if (trackerType == "KF-EBT") {

        kalmanFilterEnsembleBasedTracker.initTrackers(t_image, t_ROIToTrack);
        trackingState = true;
    }


    else if (trackerType == "ECO"){
        ecotracker.init(const_cast<Mat &>(t_image), t_ROIToTrack, ecoParameters);
        trackingState = true;

    }
    else {
        tracker->init(t_image, t_ROIToTrack);
        trackingState = true;

    }
    return false;
}

bool objectTrackingRateThread::trackingPrediction(cv::Mat &t_image, cv::Rect2d &t_ROITrackResult) {

    bool ret = true;
    if (tracker) {
        ret = tracker->update(t_image, t_ROITrackResult);
    }
    else if (trackerType == "KF-EBT") {
        t_ROITrackResult = kalmanFilterEnsembleBasedTracker.track(t_image);
        if (t_ROITrackResult.width == 0) {
            ret = false;
        }
    }

    else if (trackerType == "ECO"){
        cv::Rect2f ecoBox;
        ret = ecotracker.update(t_image, ecoBox);
        t_ROITrackResult.x = ecoBox.x ;
        t_ROITrackResult.y = ecoBox.y ;
        t_ROITrackResult.width = ecoBox.width;
        t_ROITrackResult.height = ecoBox.height ;



    }

    return ret;

}


bool objectTrackingRateThread::setTemplateFromImage() {

    if ( inputImagePort.getInputCount()) {
        ImageOf<yarp::sig::PixelBgr> *yarp_templateImage = templateImageInputPort.read(true);
        const cv::Mat templateMat = yarp::cv::toCvMat(*yarp_templateImage);


        ImageOf<yarp::sig::PixelBgr> *yarp_image = inputImagePort.read(true);
        inputImageMat = yarp::cv::toCvMat(*yarp_image);

        cv::Mat result_match;
        int result_cols =  inputImageMat.cols - templateMat.cols + 1;
        int result_rows = inputImageMat.rows - templateMat.rows + 1;
        result_match.create( result_rows, result_cols, CV_32FC1 );


        cv::matchTemplate( inputImageMat, templateMat, result_match, cv::TM_CCOEFF_NORMED );
        cv::normalize( result_match, result_match, 0, 1, cv::NORM_MINMAX, -1, Mat() );
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
        cv::Point matchLoc;
        cv::minMaxLoc( result_match, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
        matchLoc = maxLoc;

        yInfo("Maximun value %f", maxVal);
        widthInputImage = inputImageMat.cols;
        heightInputImage = inputImageMat.rows;

        ROITemplateToTrack = cv::Rect2d(matchLoc, cv::Point( matchLoc.x + templateMat.cols , matchLoc.y + templateMat.rows ));
        initializeTracker(inputImageMat, ROITemplateToTrack);


        return true;
    }

    return false;
}

bool objectTrackingRateThread::setTemplateFromCoordinate(const int xMin, const int yMin, const int xMax, const int yMax) {



    if (inputImagePort.getInputCount()) {

        currentTime = timer.now();

        ImageOf<yarp::sig::PixelBgr> *yarp_image = inputImagePort.read(true);
        inputImageMat = yarp::cv::toCvMat(*yarp_image);

        widthInputImage = inputImageMat.cols;
        heightInputImage = inputImageMat.rows;

        ROITemplateToTrack = cv::Rect2d(xMin, yMin, std::abs(xMax - xMin), std::abs(yMax - yMin));
        initializeTracker(inputImageMat, ROITemplateToTrack);
        getAnglesHead();

        return true;
    }

    return false;
}

bool objectTrackingRateThread::checkROI(cv::Rect2d *t_ROI) {
    return (0 < t_ROI->x
            && 0 <= t_ROI->width
            && t_ROI->x + t_ROI->width <= widthInputImage
            && 0 < t_ROI->y
            && 0 <= t_ROI->height
            && t_ROI->y + t_ROI->height <= heightInputImage);
}

bool objectTrackingRateThread::isTrackingState() const {
    return trackingState;
}

void objectTrackingRateThread::setTrackingState(bool p_trackingState) {
    objectTrackingRateThread::trackingState = p_trackingState;
}

void objectTrackingRateThread::stopTracking() {

    trackingState = false;


    if(enableTracking){
        Vector anglesHome(3);
        anglesHome[0] = 0.0;
        anglesHome[1] = 0.0;
        anglesHome[2] = 0.0;
        iGaze->restoreContext(ikinGazeCtrl_Startcontext);
        iGaze->lookAtAbsAngles(anglesHome);

    }
}

void objectTrackingRateThread::setEnable_log(bool enable_log) {
    this->enableLog = enable_log;
}

void objectTrackingRateThread::logTrack(cv::Mat &image, cv::Rect2d &roi) {


    time_t current_time = time(nullptr);
    const tm *currentTimeStruct = localtime(&current_time);

    std::ofstream logging;

    logging.open(logFileName, std::ofstream::out | std::ios::app);

    if (writeHeader) {
        logging << "filename,xmin,ymin,xmax,ymax,azimuth,elevation,vergence\n";
        writeHeader = false;

    }

    const string dateTime = to_string(currentTimeStruct->tm_mday) + "-" + to_string(currentTimeStruct->tm_mon + 1) \
 + "-" + to_string(1900 + currentTimeStruct->tm_year) + "_" + to_string(currentTimeStruct->tm_hour) \
 + to_string(currentTimeStruct->tm_min);

    const string image_name = "log" + to_string(counterFile) + "_" + dateTime + "-" + to_string(new_interaction_counter)+".jpeg";

    cv::imwrite(this->logPath + "/images/" + image_name, image);

    logging << image_name + "," + to_string(roi.tl().x) + "," + to_string(roi.tl().y) + ","
               + to_string(roi.br().x) + ","
               + to_string(roi.br().y) + ","
               + to_string(azimuth) +  ","
               + to_string(elevation) +  ","
               + to_string(vergence)+ "\n";
    logging.close();
    ++counterFile;
}

bool objectTrackingRateThread::checkLogDirectory() {
    struct stat st{};
    const string image_dir = this->logPath + "/images";
    return (stat(this->logPath.c_str(), &st) == 0 && stat(image_dir.c_str(), &st) == 0);

}

void objectTrackingRateThread::setLog_path(const string &log_path) {
    objectTrackingRateThread::logPath = log_path;
}

std::string objectTrackingRateThread::getLog_path() {
    return this->logPath;
}

void objectTrackingRateThread::incrementInteractionCounter() {
    this->new_interaction_counter++;

}

void objectTrackingRateThread::getAnglesHead(

        ) {
    if (anglePositionPort.getInputCount()) {
        Bottle *anglesBottle = anglePositionPort.read();

        azimuth = anglesBottle->get(0).asDouble();
        elevation =anglesBottle->get(1).asDouble();
        vergence =anglesBottle->get(2).asDouble();

    }
}





