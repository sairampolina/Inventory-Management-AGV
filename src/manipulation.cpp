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


#include "../include/manipulation.hpp"

Manipulation::Manipulation(ros::NodeHandle* nh) {
    nh_ = nh;
    set_obj_state_client_ = nh_->serviceClient<std_srvs::SetBool>("/setObjectState");
    ROS_INFO_STREAM("[Manipulation Stack]: Starting Manipulation node ");
}
//change goal
void Manipulation::pick_package() {
    geometry_msgs::Pose goal;
    tf2::Quaternion transform;
    transform.setRPY(0.011, 0.011, 1.57);

    goal.position.x = 0.2;
    goal.position.y = 0.0;
    goal.position.z = 0.45;
    goal.orientation = tf2::toMsg(transform);
    
    reach(goal);
}

//change again
void Manipulation::place_package() {
    geometry_msgs::Pose goal;
    tf2::Quaternion transform;
    // orientation while placing the object
    transform.setRPY(-0.011, 1.57, 1.57);
    goal.position.x = 0.65;
    goal.position.y = 0.0;
    goal.position.z = 0.26;
    goal.orientation = tf2::toMsg(transform);
    reach(goal);

    std_srvs::SetBool srv;
    srv.request.data = false;
    set_obj_state_client_.call(srv);
}

void Manipulation::reach(geometry_msgs::Pose my_pose) {
    //  create am asynchronous spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //  publishing robot arm goal pose
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "base_footprint";
    goal.pose = my_pose;
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(goal);

    ROS_INFO_STREAM("[Manipulation Stack]: Planning for Manipulation ");

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);

    //  Create moveit planner interface.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    group_arm_torso.setPlanningTime(5.0);
    bool flag = static_cast<bool>(group_arm_torso.plan(my_plan));

    if (!flag)
        throw std::runtime_error("No plan found");

    ROS_INFO_STREAM(" Plan found ");

    // Execute the plan
    ros::Time start = ros::Time::now();

    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!static_cast<bool>(e))
        throw std::runtime_error("[Manipulation Stack]: Error in manipulation");

    spinner.stop();
}

void Manipulation::move_to_object(geometry_msgs::Pose tar_pose)  {
    tf2::Quaternion transform;
    // orientation while picking up
    transform.setRPY(-0.011, 1.57, 1.57);
    tar_pose.position.z = 0.30;
    tar_pose.orientation = tf2::toMsg(transform);
    reach(tar_pose);

    std_srvs::SetBool srv;
    srv.request.data = true;
    set_obj_state_client_.call(srv);
}