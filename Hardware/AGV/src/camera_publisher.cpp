#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher {
public:
    CameraPublisher() {
        ros::NodeHandle nh;
        image_pub = nh.advertise<sensor_msgs::CompressedImage>("/camera/image/compressed", 10);

        // Find an available camera index
        cap = nullptr;
        for (int i = 0; i < 5; ++i) {  // Try up to 5 indices (0-4)
            cap = new cv::VideoCapture(i);
            if (cap->isOpened()) {
                ROS_INFO("Camera opened at index %d", i);
                cap->set(cv::CAP_PROP_FRAME_WIDTH, 640);
                cap->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                cap->set(cv::CAP_PROP_FPS, 30);
                cap->set(cv::CAP_PROP_EXPOSURE, -6);
                break;
            }
            delete cap;
            cap = nullptr;
        }

        if (cap == nullptr) {
            ROS_ERROR("Cannot open any camera.");
            ros::shutdown();
        }
    }

    ~CameraPublisher() {
        if (cap != nullptr) {
            cap->release();
            delete cap;
        }
    }

    void publish_frames() {
        while (ros::ok()) {
            cv::Mat frame;
            *cap >> frame;

            if (frame.empty()) {
                ROS_ERROR("Cannot receive frame (stream end?). Exiting ...");
                break;
            }

            try {
                // Convert the frame to a ROS compressed image message
                sensor_msgs::CompressedImage msg;
                msg.header.stamp = ros::Time::now();
                msg.format = "jpeg";
                std::vector<uchar> buffer;
                cv::imencode(".jpg", frame, buffer);
                msg.data = buffer;
                image_pub.publish(msg);
            } catch (std::exception& e) {
                ROS_ERROR("Exception caught: %s", e.what());
            }

            ros::spinOnce();
        }
    }

private:
    ros::Publisher image_pub;
    cv::VideoCapture* cap;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    CameraPublisher camera_publisher;
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        camera_publisher.publish_frames();
        loop_rate.sleep();
    }

    return 0;
}
