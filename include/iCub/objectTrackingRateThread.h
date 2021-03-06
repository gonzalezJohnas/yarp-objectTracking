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
 * @file objectTracking.h
 * @brief Definition of a thread that receives an data from input port and sends it to the output port.
 */


#ifndef _objectTracking_RATETHREAD_H_
#define _objectTracking_RATETHREAD_H_


#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <iCub/kfebt/kfebtracker.h>

#include "eco/eco.hpp"

class objectTrackingRateThread : public yarp::os::RateThread {

 public:
    /**
    * constructor default
    */
    objectTrackingRateThread();

    /**
     *
     * @param rf
     */
    objectTrackingRateThread(yarp::os::ResourceFinder &rf);



    /**
     * destructor
     */
    ~objectTrackingRateThread() override;

    /**
    *  initialises the thread
    */
    bool threadInit() override;

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run();

    /**
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);

    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char *p);

    bool setTemplateFromImage();

    bool setTemplateFromCoordinate(const int xMin, const int yMin, const int xMax, const int yMax );

    bool isTrackingState() const;

    void setTrackingState(bool trackingState);

    bool checkLogDirectory();

    void setLog_path(const string &log_path);

    void setEnable_log(bool enable_log);

    std::string getLog_path();

    void stopTracking();

    void incrementInteractionCounter();


private :

    std::string robot;              // name of the robot
    std::string name;               // rootname of all the ports opened by this thread
    int drivingCamera{};
    int widthInputImage{};
    int heightInputImage{};

    //Tracker parameters
    std::string trackerType;
    cv::Ptr<cv::Tracker> tracker;
    KFebTracker kalmanFilterEnsembleBasedTracker;
    eco::ECO ecotracker;
    eco::EcoParameters ecoParameters;
    cv::Rect2d ROITemplateToTrack, currentTrackRect;
    bool trackingState{};
    yarp::os::SystemClock timer;
    double timeDiff{};



    yarp::os::BufferedPort <yarp::os::Bottle> inputTargetCoordinate;                                //
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelRgb> > templateImageInputPort;                                // input template Image  of the object to track
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > templateImageOutputPort;                                // input template Image  of the object to track
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > trackerOutputPort;                                // output Image with the ROI tracked
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > inputImagePort;
    yarp::os::BufferedPort<yarp::os::Bottle> anglePositionPort;

    // input Image in which the tracking is perform
    cv::Mat inputImageMat, templateOutputMat, outputMat;

    //iKinGazeCtrl parameters
    int ikinGazeCtrl_Startcontext{}, gaze_context{};
    yarp::dev::PolyDriver* clientGaze{};
    yarp::dev::IGazeControl *iGaze{};
    bool enableSaccade, enableTracking{};

    double previousImagePosX{}, previousImagePosY{};

    // Log parameters
    bool enableLog{}, writeHeader{};
    std::string logPath;
    std::string logFileName;
    int counterFile{}, frequencyAcquisitionCounter{}, new_interaction_counter{};
    double azimuth{}, elevation{}, vergence{};


    // Habituation decay factor
    int habituationCpt{};
    double currentTime{};
    bool doHabituation{};


    //******************************************************************************************************************


    void setTracker();

    bool openIkinGazeCtrl();

    bool trackIkinGazeCtrl(const cv:: Rect2d &track_roi);

    bool initializeTracker(cv::Mat t_image, cv::Rect2d t_ROIToTrack);

    bool trackingPrediction(cv::Mat &t_image, cv::Rect2d &t_ROIToTrack);

    bool checkROI(cv::Rect2d *t_ROI);


    void logTrack(cv::Mat &image, cv::Rect2d &roi);

    void getAnglesHead();

};

#endif  //_objectTracking_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

