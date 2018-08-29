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

class objectTrackingRateThread : public yarp::os::RateThread {
private:
    bool result;                    //result of the processing

    std::string robot;              // name of the robot
    std::string inputPortName;      // name of input port for incoming events,
    std::string name;               // rootname of all the ports opened by this thread
    int widthInputImage;
    int heightInputImage;

    //Tracker parameters
    std::string trackerType;
    cv::Ptr<cv::Tracker> tracker;
    KFebTracker kalmanFilterEnsembleBasedTracker;
    cv::Rect2d ROITemplateToTrack, currentTrackRect;
    bool trackingState;

    yarp::os::BufferedPort <yarp::os::Bottle> inputTargetCoordinate;                                //
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > templateImageInputPort;                                // input template Image  of the object to track
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > templateImageOutputPort;                                // input template Image  of the object to track
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > trackerOutputPort;                                // output Image with the ROI tracked
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > inputImagePort;                                // input Image in which the tracking is perform


    //iKinGazeCtrl parameters
    int ikinGazeCtrl_Startcontext;
    yarp::dev::PolyDriver* clientGaze;
    yarp::dev::IGazeControl *iGaze;
    bool enableSaccade;

    double previousImagePosX, previousImagePosY;

    // Log parameters
    bool enableLog, writeHeader;
    std::string logPath;
    std::string logFileName;
    int counterFile, frequencyAcquisitionCounter;

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

    /**
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    bool setTemplateFromImage();

    bool setTemplateFromCoordinate(const int xMin, const int yMin, const int xMax, const int yMax );

    bool isTrackingState() const;

    void setTrackingState(bool trackingState);

    bool checkLogDirectory();

    void setLog_path(const string &log_path);

    void setEnable_log(bool enable_log);

    std::string getLog_path();

private :


    //******************************************************************************************************************


    void setTracker();

    bool openIkinGazeCtrl();

    bool trackIkinGazeCtrl( cv:: Rect2d);

    bool initializeTracker(cv::Mat t_image, cv::Rect2d t_ROIToTrack);

    bool trackingPrediction(cv::Mat t_image, cv::Rect2d *t_ROIToTrack);

    bool checkROI(cv::Rect2d *t_ROI);

    void stopTracking();

    void logTrack(cv::Mat image, cv::Rect2d roi);

};

#endif  //_objectTracking_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

