#include <gtest/gtest.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "perception.hpp"

bool messageReceived;




bool waitFor_message(const bool& message_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

TEST(TestDetector, detectorTest) {
    // Arrange
    cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));
    messageReceived = false;
    ros::NodeHandle nh;
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(
                                                "xtion/rgb/image_raw", 10);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "bgr8", image).toImageMsg();

    // Act
    img_pub.publish(msg);
    waitFor_message(messageReceived, 3);
    messageReceived = true;

    // Assert
    ASSERT_TRUE(waitFor_message(messageReceived, 3));
}


TEST(TestDetector, findObjectTest) {
    // Arrange

    bool expected_value = true;

    ros::NodeHandle nh;

    PackageDetector detector(&nh);


    cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));

    detector.image_ = image;

    ASSERT_EQ(detector.find_obj(), expected_value);
    
}