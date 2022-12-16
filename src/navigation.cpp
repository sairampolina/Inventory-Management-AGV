// Copyright Venkata Sairam Polina.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file navigation.cpp
 * @author sairam polina (polinavenkatasairam@gmail.com)
 * @brief Implementation of navigation.cpp
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/navigation.hpp"
#include <cmath>

Navigation::Navigation(ros::NodeHandle* nh): waypoint_counter_(0) {
    nh_ = nh;
    rot_state_ = ROT_COMPLETE;
    pose_flag_ = false;
    drop_loc_.position.x = -4;
    drop_loc_.position.y = 0;

    goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>
        ("/move_base_simple/goal", 10);
    vel_pub_ = nh_->advertise<geometry_msgs::Twist>
        ("/mobile_base_controller/cmd_vel", 10);
    cancel_goal_pub_ = nh_->advertise<actionlib_msgs::GoalID>
        ("/move_base/cancel", 5);

    pre_pose_sub_ = nh_->subscribe("/robot_pose", 10,
        &Navigation::pose_callback, this);

    kill_costmap_client_ = nh_->serviceClient<std_srvs::Empty>
        ("/move_base/clear_costmaps");

    set_waypoints();
}

void Navigation::set_goal() {
    if (waypoint_counter_ < 4) {
        geometry_msgs::PoseStamped new_goal;
        new_goal.pose.position = waypoints_[waypoint_counter_].position;
        new_goal.pose.orientation.w = 1.0;

        new_goal.header.frame_id = "map";

        goal_pub_.publish(new_goal);
        goal_pub_.publish(new_goal);
        goal_pose_ = new_goal.pose;
        ROS_INFO_STREAM("[Navigation Stack]: Navigating to next waypoint ");
    } else {
        ROS_INFO_STREAM
            ("[Navigation Stack]: No package left in the warehouse ");
    }

    waypoint_counter_++;
}

void Navigation::set_pkgloc_as_goal(geometry_msgs::Pose object) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position = object.position;
    target_pose.pose.orientation.w = 1.0;

    target_pose.header.frame_id = "map";

    goal_pub_.publish(target_pose);
    goal_pub_.publish(target_pose);
    goal_pose_ = target_pose.pose;
    ROS_INFO_STREAM("[Navigation Stack]: Navigating closer to the package ");
}

void Navigation::set_droploc_as_goal() {
    std_srvs::Empty srv;
    kill_costmap_client_.call(srv);

    geometry_msgs::PoseStamped target;
    target.pose.position = drop_loc_.position;
    target.pose.orientation.w = 1.0;

    target.header.frame_id = "map";

    goal_pub_.publish(target);
    goal_pub_.publish(target);
    goal_pose_ = target.pose;
    ROS_INFO_STREAM("[Navigation Stack]: Navigating to drop location ");
}

void Navigation::pose_callback
    (const geometry_msgs::PoseWithCovarianceStamped &agv_pose) {
    pre_pose_ = agv_pose.pose.pose;
    pose_flag_ = true;
}

bool Navigation::if_goal_reached() {
    auto x = pre_pose_.position.x - goal_pose_.position.x;
    x = std::pow(x, 2);
    auto y = pre_pose_.position.y - goal_pose_.position.y;
    y = std::pow(y, 2);

    if (std::sqrt(x + y) >= 0.1)
        return false;
    return true;
}

void Navigation::turn_robot() {
    if (pose_flag_) {
        if (rot_state_ == ROT_START) {
            tf2::fromMsg(pre_pose_.orientation, init_quaternion_);
            init_quaternion_ = init_quaternion_.inverse();

            rot_state_ = ROTATING;
        } else {
            tf2::Quaternion tf2_quaternion;

            tf2::fromMsg(pre_pose_.orientation, tf2_quaternion);

            tf2_quaternion *= init_quaternion_;

            auto rot_angle = tf2::getYaw(tf2_quaternion);

            set_rot_vel();

            if (rot_angle < 0 && rot_angle > -0.1) {
                rot_state_ = ROT_COMPLETE;
            }
        }
    } else {
        ROS_INFO_STREAM("[Navigation Stack]: Initial pose not published");
    }
}

void Navigation::stop_robot() {
    actionlib_msgs::GoalID msg;
    cancel_goal_pub_.publish(msg);

    actionlib_msgs::GoalStatusArrayConstPtr status;

    ROS_INFO_STREAM("[Navigation Stack]: Cancelling assigned goal");

    status = ros::topic::waitForMessage
        <actionlib_msgs::GoalStatusArray>
            ("/move_base/status", ros::Duration(2));
    ROS_INFO_STREAM("[Navigation Stack]: Robot has stopped.");
}

void Navigation::set_waypoints() {
    std::array<int, 5> waypoints_x = {0, 1, -1, 1, 2};
    std::array<int, 5> waypoints_y = {-4, -1, -5, -8, 0};
    for (int i = 0; i < 5; i++) {
        geometry_msgs::Pose point_pose;
        point_pose.position.x = waypoints_x[i];
        point_pose.position.y = waypoints_y[i];
        waypoints_.push_back(point_pose);
    }
}

void Navigation::set_rot_vel() {
    geometry_msgs::Twist msg;

    msg.angular.z = 0.6;
    vel_pub_.publish(msg);
}

