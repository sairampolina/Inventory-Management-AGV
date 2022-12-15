#include <gtest/gtest.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>

#include "object.hpp"

// Global variables
bool msg_received = false;
gazebo_msgs::ModelState pose_msg;

bool waitForMessage(const bool& msg_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!msg_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

void pose_cb(const gazebo_msgs::ModelStateConstPtr& msg) {
    msg_received = true;
    pose_msg.reference_frame = msg->reference_frame;
    pose_msg.pose = msg->pose;
}


TEST(test_object_spawner_class, test_constructor) {
    // Arrange
    msg_received = false;
    bool expected_value = false;
    ros::NodeHandle nh;

    // Act
    Object spawner(&nh);

    // Assert
    ASSERT_EQ(spawner.if_picked_up_pkg, expected_value);
}

TEST(test_object_spawner_class, test_set_object_pose) {
    // Arrange
    msg_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/set_model_state", 10,
                                                            pose_cb);
    geometry_msgs::Pose object_pose;
    object_pose.position.x = -3.5;
    object_pose.position.y = -12;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    gazebo_msgs::ModelState expected_msg;
    expected_msg.pose = object_pose;
    expected_msg.reference_frame = "world";

    // Act
    ros::NodeHandle n_h;
    Object spawner(&n_h);
    spawner.set_pose_of_pkg(object_pose);

    // Assert
    ASSERT_TRUE(waitForMessage(msg_received, 3));
    EXPECT_EQ(pose_msg.reference_frame, expected_msg.reference_frame);
    EXPECT_EQ(pose_msg.pose, expected_msg.pose);
}

TEST(test_object_spawner_class, set_object_pose_test) {
    // Arrange
    geometry_msgs::Pose object_pose;
    object_pose.position.x = -5.4;
    object_pose.position.y = -1.2;
    object_pose.position.z = 0.025;
    object_pose.orientation.w = 1;

    gazebo_msgs::ModelState expected_msg;
    expected_msg.pose = object_pose;
    expected_msg.reference_frame = "world";

    // Act
    msg_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/set_model_state", 10,
                                                            pose_cb);
    // Assert
    ASSERT_TRUE(waitForMessage(msg_received, 3));
    // EXPECT_EQ(pose_msg.reference_frame, expected_msg.reference_frame);
    // EXPECT_EQ(pose_msg.pose, expected_msg.pose);
}