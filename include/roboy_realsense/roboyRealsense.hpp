#pragma once

#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <librealsense/rs.hpp>
#include <roboy_communication_middleware/ArucoPose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

class RoboyRealsense{
public:
    RoboyRealsense();

    /**
     * Alternative constructor
     * @param arucoIDs react only to these ids
     */
    RoboyRealsense(vector<int> &arucoIDs);

    void arucoDetection();

private:
    ros::NodeHandlePtr nh;
    double setpoint = 0;
    string joint_name;
    ros::NodeHandle n;
    ros::Subscriber steer_sub;
    ros::Publisher aruco_pose_pub, visualization_pub, video_pub;
    boost::shared_ptr<rs::context> realsense_ctx;
    rs::device * realsense_dev;
    rs::intrinsics color_intrin;
    Mat camMatrix, distCoeffs;
    Ptr<aruco::DetectorParameters> detectorParams;
    Ptr<aruco::Dictionary> dictionary;
    vector<int> arucoIDs;
    float markerLength = 0.062f;
    float K[9];
};
