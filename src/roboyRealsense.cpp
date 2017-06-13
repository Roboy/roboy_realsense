#include "roboy_realsense/roboyRealsense.hpp"

RoboyRealsense::RoboyRealsense() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    aruco_pose_pub = nh->advertise<roboy_communication_middleware::ArucoPose>("/roboy/middleware/ArucoPose", 1);
    visualization_pub = nh->advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    realsense_ctx = boost::shared_ptr<rs::context>(new rs::context);
    ROS_INFO("There are %d connected RealSense devices.\n", realsense_ctx->get_device_count());
    if(realsense_ctx->get_device_count() == 0) {
        ROS_ERROR("no realsense connected");
        return;
    }else {
        realsense_dev = realsense_ctx->get_device(0);
        ROS_INFO("\nUsing device 0, an %s\n     Serial number: %s\n     Firmware version: %s\n",
                 realsense_dev->get_name(), realsense_dev->get_serial(), realsense_dev->get_firmware_version());

        // Configure all streams to run at VGA resolution at 60 frames per second
        realsense_dev->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);

        color_intrin = realsense_dev->get_stream_intrinsics(rs::stream::color);
        float k[9] = {color_intrin.fx, 0, color_intrin.ppx, 0, color_intrin.fy, color_intrin.ppy, 0, 0, 1};
        memcpy(K,k,sizeof(float)*9);
        camMatrix = Mat(3,3,CV_32FC1,K);
        distCoeffs = Mat (1,5,CV_32FC1,color_intrin.coeffs);

        cout << camMatrix << endl;
        cout << distCoeffs << endl;

        detectorParams = aruco::DetectorParameters::create();
        detectorParams->doCornerRefinement = true;
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_ARUCO_ORIGINAL));

        realsense_dev->start();
    }
};

RoboyRealsense::RoboyRealsense(vector<int> &arucoIDs):arucoIDs(arucoIDs){
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    aruco_pose_pub = nh->advertise<roboy_communication_middleware::ArucoPose>("/roboy/middleware/ArucoPose", 1);
    visualization_pub = nh->advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    realsense_ctx = boost::shared_ptr<rs::context>(new rs::context);
    ROS_INFO("There are %d connected RealSense devices.\n", realsense_ctx->get_device_count());
    if(realsense_ctx->get_device_count() == 0) {
        ROS_ERROR("no realsense connected");
        return;
    }else {
        realsense_dev = realsense_ctx->get_device(0);
        ROS_INFO("\nUsing device 0, an %s\n     Serial number: %s\n     Firmware version: %s\n",
                 realsense_dev->get_name(), realsense_dev->get_serial(), realsense_dev->get_firmware_version());

        // Configure all streams to run at VGA resolution at 60 frames per second
        realsense_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);

        color_intrin = realsense_dev->get_stream_intrinsics(rs::stream::color);
        float k[9] = {color_intrin.fx, 0, color_intrin.ppx, 0, color_intrin.fy, color_intrin.ppy, 0, 0, 1};
        memcpy(K,k,sizeof(float)*9);
        camMatrix = Mat(3,3,CV_32FC1,K);
        distCoeffs = Mat (1,5,CV_32FC1,color_intrin.coeffs);

        cout << camMatrix << endl;
        cout << distCoeffs << endl;

        detectorParams = aruco::DetectorParameters::create();
        detectorParams->doCornerRefinement = true;
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_100));

        realsense_dev->start();
    }
}

void RoboyRealsense::arucoDetection() {
    while(ros::ok()){
        realsense_dev->wait_for_frames();
        const uint8_t *color_frame = reinterpret_cast<const uint8_t *>(realsense_dev->get_frame_data(rs::stream::color));
        Mat image = Mat(1080, 1920, CV_8UC3, (uint8_t*)color_frame), imageCopy;
        cv::cvtColor(image, image, CV_RGB2BGR);
        image.copyTo(imageCopy);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        // draw results
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            roboy_communication_middleware::ArucoPose msg;
            msg.id = ids;
            for(unsigned int i = 0; i < ids.size(); i++) {
                if(!arucoIDs.empty()) {
                    if (std::find(arucoIDs.begin(), arucoIDs.end(), ids[i]) == arucoIDs.end())
                        continue;
                }
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
                geometry_msgs::Pose pose;
                double theta = sqrt(pow(rvecs[i][0],2.0) + pow(rvecs[i][1],2.0) + pow(rvecs[i][2],2.0));
                pose.orientation.x = rvecs[i][0]/theta;
                pose.orientation.y = rvecs[i][1]/theta;
                pose.orientation.z = rvecs[i][2]/theta;
                pose.orientation.w = theta;
                pose.position.x = tvecs[i][0];
                pose.position.y = tvecs[i][1];
                pose.position.z = tvecs[i][2];
                msg.pose.push_back(pose);

                visualization_msgs::Marker msg;
                msg.header.frame_id = "world";
                msg.ns = "aruco_marker";
                msg.type = visualization_msgs::Marker::CUBE;
                msg.color.r = 1.0f;
                msg.color.g = 1.0f;
                msg.color.b = 1.0f;
                msg.color.a = 1.0f;
                msg.scale.x = markerLength;
                msg.scale.y = markerLength;
                msg.scale.z = markerLength;
                msg.lifetime = ros::Duration(0.1);
                msg.header.stamp = ros::Time::now();
                msg.action = visualization_msgs::Marker::ADD;
                msg.pose.position = pose.position;
                msg.pose.orientation = pose.orientation;
                msg.id = ids[i];
                visualization_pub.publish(msg);
            }
            aruco_pose_pub.publish(msg);
        }

        imshow("imageCopy", imageCopy);
        waitKey(1);
        ROS_INFO_THROTTLE(5, "aruco ids visible: %d", (int)ids.size());
    }
}