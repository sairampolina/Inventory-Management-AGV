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


#include "../include/navigation.hpp"

Navigation::Navigation(ros::Nodehandle* nh) {
    nh_ = nh;
    rot_state_ = ROT_COMPLETE;
    pose_flag_ = false;
    waypoint_counter_ = 0;
    
    //change according to map
    drop_loc_.position.x = 0;
    drop_loc_.position.y = 0;
    
    goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
    cancel_goal_pub_ = nh_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 5);

    pre_pose_sub_ = nh_->subscribe("/robot_pose", 10, &Navigation::pose_callback, this);

    kill_costmap_client_ = nh_->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    //check this line of code
    set_waypoints();
}

void Navigation::set_goal() {

    //we can change number of waypoints
    if(waypoint_counter_ < 4) {
        geometry_msgs::PoseStamped new_goal;
        new_goal.pose.position = waypoints_[waypoint_counter].position;
        new_goal.pose.orientation.w = 1.0;
        //check 'map'
        new_goal.header.frame_id = 'map';
        //Check twice
        goal_pub_.publish(new_goal);
        goal_ = new_goal;
        ROS_INFO_STREAM(" Navigating to waypoint number : ", waypoint_counter_);
    }
    else {
        ROS_INFO_STREAM(" No package left in the warehouse ");
    }

    waypoint_counter_++;
}

void Navigation::set_pkgloc_as_goal(geometry_msgs::Pose object) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position = object.position;
    target_pose.pose.orientation.w = 1.0;
    //check 'map'
    target_pose.header.frame_id = 'map';
    //Check twice
    goal_pub_.publish(target_pose);
    goal_ = target_pose;
    ROS_INFO_STREAM(" Navigating closer to the package ");
}

void Navigation::set_droploc_as_goal() {
    std_srvs::Empty srv;
    kill_costmap_client_.call(srv);

    geometry_msgs::PoseStamped target;
    target.pose.position = drop_loc_.position;
    target.pose.orientation.w = 1.0;
    //check 'map'
    target.header.frame_id = 'map';
    //Check twice
    goal_pub_.publish(target);
    goal_ = target;
    ROS_INFO_STREAM(" Navigating to drop location ");
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped &agv_pose) {
    pre_pos_ = agv_pose.pose.pose;
    pose_flag_ = true;
}

bool if_goal_reached() {
    auto x = pre_pose_.position.x - goal_.position.x;
    x = std::pow(x, 2);
    auto y = pre_pose_.position.y - goal_.position.y;
    y = std::pow(y, 2);
    if (std::sqrt(x, y) > 0.1)
        return false;
    return true;
}

