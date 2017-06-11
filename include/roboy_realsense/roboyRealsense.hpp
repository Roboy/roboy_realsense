#pragma once

#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <librealsense/rs.hpp>

using namespace std;
using namespace cv;

class RoboyRealsense{
public:
    RoboyRealsense();

    void arucoDetection();

private:
    ros::NodeHandlePtr nh;
    double setpoint = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber steer_sub;
    boost::shared_ptr<rs::context> realsense_ctx;
    rs::device * realsense_dev;
    rs::intrinsics color_intrin;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    float markerLength = 0.058f;
    float K[9];
};
