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
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


class objectTrackingRateThread : public yarp::os::RateThread {
private:
    bool result;                    //result of the processing

    std::string robot;              // name of the robot
    std::string inputPortName;      // name of input port for incoming events,
    std::string name;               // rootname of all the ports opened by this thread


    //Tracker parameters
    std::string trackerType;
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d objectTemplateRectToTrack, currentTrackRect;
    bool trackingMode;

    yarp::os::BufferedPort <yarp::os::Bottle> outputPort;                                // port necessary to send the gaze command to the gazeArbiter
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > templateInputPort;                                // port necessary to send the gaze command to the gazeArbiter
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelRgb> > trackerOutputPort;                                // port necessary to send the gaze command to the gazeArbiter
    yarp::os::BufferedPort <yarp::sig::ImageOf<yarp::sig::PixelBgr> > inputImagePort;                                // port necessary to send the gaze command to the gazeArbiter


public:
    /**
    * constructor default
    */
    objectTrackingRateThread();

    /**
     *
     * @param rf
     */
    explicit objectTrackingRateThread(yarp::os::ResourceFinder &rf);

    /**
    * constructor 
    * @param robotname name of the robot
    */
    objectTrackingRateThread(std::string robotname, std::string configFile);

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


    //******************************************************************************************************************

    bool setTemplateFromImage();

    void setTracker();


};

#endif  //_objectTracking_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
