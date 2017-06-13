#include "roboy_realsense/roboyRealsense.hpp"

int main(int argc, char* argv[])
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyRealsense");
    }
    RoboyRealsense realsense;
//    vector<int> arucoIDs = {233,553,627,628,1010,153,292,62};
//    RoboyRealsense realsense(arucoIDs);
    realsense.arucoDetection();

    return 0;
}

