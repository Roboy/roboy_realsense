#include "roboy_realsense/roboyRealsense.hpp"

RoboyRealsense::RoboyRealsense() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

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
};

void RoboyRealsense::arucoDetection() {
    while(ros::ok()){
        realsense_dev->wait_for_frames();
        const uint8_t *color_frame = reinterpret_cast<const uint8_t *>(realsense_dev->get_frame_data(rs::stream::color));
        Mat image = Mat(480, 640, CV_8UC3, (uint8_t*)color_frame), imageCopy;
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
            for(unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
            }

        }

        imshow("imageCopy", imageCopy);
        waitKey(1);
        ROS_INFO_THROTTLE(1, "aruco ids visible: %d", (int)ids.size());
    }
}