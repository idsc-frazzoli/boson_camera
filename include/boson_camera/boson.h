//
// Created by andya on 02.07.18.
//

#ifndef BOSON_CAMERA_BOSON_CAMERA_H
#define BOSON_CAMERA_BOSON_CAMERA_H
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 512

#include <string>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

namespace boson {
    class BosonCamera {

    public:
        BosonCamera(ros::NodeHandle &nh, ros::NodeHandle nh_private, std::string device_id);

        ~BosonCamera();

        static void onDisconnectUSB(void *);

    private:
        ros::NodeHandle nh_;
        ros::Publisher image_pub_;
        ros::Publisher camera_info_pub_;
        std::string ns;

        int fd;
        std::string device;
        struct v4l2_capability cap;
        struct v4l2_format format;
        struct v4l2_buffer buffer;

        cv::Mat raw_input;

        ros::Time reset_time_;
        int width;
        int height;
        timeval last_ts;

        // ~BosonCamera();
        void init();

        void allocateBuffer();

        void startStream();

        void stopStream();

        void closeConnection();

        cv::Mat captureRawFrame();
    };
}// namespace

#endif //BOSON_CAMERA_BOSON_CAMERA_H
