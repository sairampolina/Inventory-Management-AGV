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

#include "../include/object.hpp"

Object::Object(ros::NodeHandle* nh): 
                        tflistener_(this->tfbuffer_){

pkg_name = "box";
if_spawned = false;

if_picked_up_pkg = false;

seed = 4;
map_range = {-6,-7,2,7};

ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
                                                  "/robot_pose",ros::Duration(10));
nh_ = nh;

pose_pub_ = nh_->advertise<gazebo_msgs::ModelState>(
                                            "/gazebo/set_model_state",10);
}


bool Object::spawn_pkg(){
    if(if_spawned) {
        ROS_INFO_STREAM("Package already spawned.");
        return 0;
    }

    spawn_pkg_client_ = 
        nh_-> serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = pkg_name;
    srv.request.model_xml = urdf_string_;  

    srv.request.initial_pose.position.x = -4;
    srv.request.initial_pose.position.y = -2;
    srv.request.initial_pose.position.z = 0.025;
    srv.request.initial_pose.orientation.w = 1;
    srv.request.reference_frame = "world";
    srv.response.success = true;
    spawn_pkg_client_.call(srv);


    if (srv.response.success) {
        ROS_INFO_STREAM("Package spawned successfully");
        if_spawned = true;

        pkg_pose_ = srv.request.initial_pose;


        pkg_pose_tf_timer = nh_->createTimer(ros::Duration(0.1),
                                            &Object::publish_pkg_loc, this);

        update_state_service_ = nh_->advertiseService("/setObjectState",
                                    &ObjectSpawner::set_pkg_state_callback, this);
    } else {
        ROS_ERROR_STREAM("[object_spawner] Failed to spawn object");
        return 1;
        
    }

    return true;
}


void Object::set_pose_of_pkg(geometry_msgs::Pose pkg_pose) {
    gazebo_msgs::ModelState msg_;
    msg_.model_name = pkg_name;
    msg_.pose = pkg_pose;
    msg_.reference_frame = "world";
    pose_pub_.publish(msg_); 
}

void Object::publish_pkg_loc(const ros::TimerEvent&) {

    geometry_msgs::trans_stamp trans_stamp;
    if (if_picked_up_pkg) {
        geometry_msgs::trans_stamp trans_stamp;
        trans_stamp = tfBuffer_.lookupTransform("map",
                                                    "gripper_grasping_frame",
                                                                ros::Time(0));
        pkg_pose_.position.x = trans_stamp.transform.translation.x;
        pkg_pose_.position.y = trans_stamp.transform.translation.y;
        pkg_pose_.position.z = trans_stamp.transform.translation.z;
        pkg_pose_.orientation = trans_stamp.transform.rotation;
    } else {
        pkg_pose_.position.z = 0.025;
    }
    set_object_pose(pkg_pose_);
    trans_stamp.header.stamp = ros::Time::now();
    trans_stamp.header.frame_id = "map";
    trans_stamp.child_frame_id = pkg_name;
    trans_stamp.transform.translation.x = pkg_pose_.position.x;
    trans_stamp.transform.translation.y = pkg_pose_.position.y;
    trans_stamp.transform.translation.z = pkg_pose_.position.z;
    trans_stamp.transform.rotation = pkg_pose_.orientation;
    broadcaster_.sendTransform(trans_stamp);
}

