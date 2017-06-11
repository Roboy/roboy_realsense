#include "roboy_realsense/roboyRealsense.hpp"

int main(int argc, char* argv[])
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyRealsense");
    }
    RoboyRealsense realsense;
    realsense.arucoDetection();

    return 0;
}

