#include "iCub/objectTrackingModule.h"
#include <iostream>


using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char *argv[]) {

    Network yarp;
    yarp.init();

    objectTrackingModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("objectTrackingRateThread.ini");      //overridden by --from parameter
    rf.setDefaultContext("objectTrackingRateThread");              //overridden by --context parameter
    rf.configure(argc, argv);


    yInfo("resourceFinder: %s", rf.toString().c_str());

    module.runModule(rf);
    return 0;
}
