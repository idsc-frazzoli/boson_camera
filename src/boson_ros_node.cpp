//
// Created by andya on 05.07.18.
//

#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "boson_camera/boson.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <boson_camera/boson.h>



int main(int argc, char *argv[]) {
    // TODO rewrite node in clean code format
    // Default frame rate of 10 Hz
    float frame_rate = 10.0;

    // Initialize node
    ros::init(argc, argv, "boson_ros_node");

    ros::NodeHandle nh("boson");
    ros::NodeHandle nh_private("~");

    printf("Video device set to: %s\n", argv[1]);

    // Initialize camera
    boson::BosonCamera *camera = new boson::BosonCamera(nh, nh_private, argv[1]);

    ros::spin();
    //return  0;


    ros::Rate loop_rate(frame_rate);
    while (ros::ok()) {
        cv::Mat img = camera.captureRawFrame();
        cv::Mat img_norm;
        img.copyTo(img_norm);

        // Normalize for visualization
        cv::normalize(img, img_norm, 65536, 0, cv::NORM_MINMAX);
        framecount++;

        // Convert to image_msg & publish msg
        sensor_msgs::ImagePtr msg[2];
        msg[0] = cv_bridge::CvImage(std_msgs::Header(), "mono16", img).toImageMsg();
        msg[1] = cv_bridge::CvImage(std_msgs::Header(), "mono16", img_norm).toImageMsg();

        for (int i = 0; i < 2; i++) {
            msg[i]->width = camera.width;
            msg[i]->height = camera.height;
            msg[i]->header.stamp.sec = camera.last_ts.tv_sec + epoch_time.tv_sec;
            msg[i]->header.stamp.nsec = camera.last_ts.tv_usec * 1e3 + epoch_time.tv_nsec;
        }

        boson_raw_pub.publish(msg[0]);
        boson_normalized_pub.publish(msg[1]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spinOnce();

    boson_raw_pub.shutdown();

    camera.stopStream();
    camera.closeConnection();
    return 0;
}
