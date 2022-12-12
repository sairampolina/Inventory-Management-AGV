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

# pragma once


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <vector>

using namespace std::chrono_literals;

class Navigation: public rclcpp::Node {
    
    public:
        Navigation();
        geometry_msgs::Pose get_goal();
        void set_goal();
        void move_to_goal();
        void move_close_to_object();
    private:
        geometry_msgs::Pose goal_;
        geometry_msgs::Pose package_droplocation;
        rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<geometry_msgs::Twist>::SharedPtr vel_pub_;

        rclcpp::Subscription<geometry_msgs::PoseWithCovarianceStamped >::SharedPtr pres_pose_sub_;
        
};

