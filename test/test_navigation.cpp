#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "navigation.hpp"

// Global variables
bool message_received;
geometry_msgs::PoseStamped pose_message;
geometry_msgs::Twist velocity_message;

bool wait_for_message(const bool& message_received, double timeout = 5) {
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

void keep_publishing(const bool& message_received, ros::Publisher* pose_pub,
            geometry_msgs::PoseWithCovarianceStamped msg, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        pose_pub->publish(msg);
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return;
        }
    }
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    message_received = true;
    pose_message.header = msg->header;
    pose_message.pose = msg->pose;
}

void velocity_callback(const geometry_msgs::TwistConstPtr& msg) {
    message_received = true;
    velocity_message.angular = msg->angular;
}




TEST(test_navigation_class, test_object_pose_goal_setting) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10,
                                                            pose_callback);
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 5.4;
    object_pose.position.y = 1.2;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    geometry_msgs::PoseStamped expected_msg;
    expected_msg.header.frame_id = "map";
    expected_msg.pose = object_pose;

    // Act
    Navigation navigator(&nh);
    navigator.set_pkgloc_as_goal(object_pose);

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(pose_message.pose.orientation, expected_msg.pose.orientation);
    EXPECT_EQ(pose_message.pose.position, expected_msg.pose.position);
    EXPECT_EQ(pose_message.header.frame_id, expected_msg.header.frame_id);
}

TEST(test_navigation_class, test_turn_around) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::
                                                    PoseWithCovarianceStamped>(
                                                            "/robot_pose", 10);
    ros::Subscriber sub = nh.subscribe("/mobile_base_controller/cmd_vel", 10,
                                                            velocity_callback);
    double expected_velocity = 0.6;
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation.w = 1;

    // Act
    Navigation navigator(&nh);
    navigator.rot_state_ = Navigation::rotation::ROTATING;
    keep_publishing(message_received, &pose_pub, msg, 3);
    navigator.turn_robot();

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(velocity_message.angular.z, 0.6);
}
