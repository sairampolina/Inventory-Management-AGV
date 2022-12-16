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
 * @file test_manipulation.cpp
 * @author sairam polina (polinavenkatasairam@gmail.com)
 * @brief test file to test manipulator
 * @version 0.1
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include<gtest/gtest.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/SetBool.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <string>
#include <vector>
#include <map>

#include "manipulation.hpp"

// bool message_received;
// geometry_msgs::PoseStamped pose_message;
// geometry_msgs::Twist velocity_message;

// TEST(TestManipulation, testPickPackage) {

//     ros::NodeHandle nh;
//     Manipulation manip(&nh);


//     ASSERT_EQ(manip.flag, false);

// }
