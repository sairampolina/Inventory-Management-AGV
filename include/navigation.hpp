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

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <vector>

class Navigation {
    
    public:
        Navigation(ros::NodeHandle*);

        void set_pkgloc_as_goal(geometry_msgs::Pose);

        void set_droploc_as_goal();

        void set_goal();


        bool if_goal_reached();

        //  turn rbot
        void turn_robot();

        void stop_robot();

        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped&);

        enum rotation {
        ROT_START,
        ROTATING,
        ROT_COMPLETE,
        };
        rotation rot_state_;

    private:

        void set_rot_vel();

        void set_waypoints();

        ros::NodeHandle* nh_;

        // subscriber to get present pose
        ros::Subscriber pre_pose_sub_;

        // Publisher to set goals 
        ros::Publisher goal_pub_;

        ros::Publisher vel_pub_;

        ros::Publisher cancel_goal_pub_;

        ros::ServiceClient kill_costmap_client_;

        bool pose_flag_;

        // present location of robot
        geometry_msgs::Pose pre_pose_;

        // approximate location of pkg 
        geometry_msgs::Pose goal_pose_;

        // Drop location of pkg
        geometry_msgs::Pose drop_loc_;

        // robot initial orientation
        tf2::Quaternion init_quaternion_;

        std::vector <geometry_msgs::Pose> waypoints_;

        std::vector<geometry_msgs::Pose>::size_type waypoint_counter_;
 
};

