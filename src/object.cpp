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
                        tflistener_(this->tfbuffer_) {
    pkg_name_ = "box";
    if_spawned = false;
    if_picked_up_pkg = false;

    ros::topic::waitForMessage
        <geometry_msgs::PoseWithCovarianceStamped>
            ("/robot_pose", ros::Duration(10));
    nh_ = nh;

    pose_pub_ = nh_->advertise<gazebo_msgs::ModelState>(
                                                "/gazebo/set_model_state", 10);

    urdf_string_ = R"(<robot name="simple_box"><link name="object_base_link">
    </link><joint name="object_base_joint" type="fixed">
    <parent link="object_base_link"/><child link="my_box"/>
    <axis xyz="0 0 1" /><origin xyz="0 0 0" rpy="0 0 0"/></joint>
    <link name="my_box"><inertial><origin xyz="0 0 0" />
    <mass value="0.1" /><inertia  ixx="0.0001" ixy="0.0"  
    ixz="0.0"  iyy="0.0001"  iyz="0.0"  izz="0.0001" /></inertial>
    <visual><origin xyz="0 0 0"/><geometry><box size="0.05 0.05 0.4" />
    </geometry></visual><collision><origin xyz="0 0 0"/><geometry>
    <box size="0.05 0.05 0.4" /></geometry></collision></link>
    <gazebo reference="my_box"><material>Gazebo/Blue</material>
    </gazebo><gazebo reference="object_base_link"><gravity>0</gravity>
    </gazebo></robot>)";
}

bool Object::spawn_pkg() {
    if (if_spawned) {
        ROS_INFO_STREAM("[Package Stack]: Package already spawned.");
        return false;
    }

    spawn_pkg_client_ = nh_-> serviceClient
        <gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = pkg_name_;
    srv.request.model_xml = urdf_string_;

    srv.request.initial_pose.position.x = -3.5;
    srv.request.initial_pose.position.y = -12;
    srv.request.initial_pose.position.z = 0.025;
    srv.request.initial_pose.orientation.w = 1;
    srv.request.reference_frame = "world";
    srv.response.success = true;
    spawn_pkg_client_.call(srv);


    if (srv.response.success) {
        ROS_INFO_STREAM("[Package Stack]: Package spawned successfully");
        if_spawned = true;

        pkg_pose_ = srv.request.initial_pose;


        pkg_pose_tf_timer_ = nh_->createTimer(ros::Duration(0.1),
                                            &Object::publish_pkg_loc, this);

        update_state_service_ = nh_->advertiseService("/setObjectState",
                                    &Object::set_pkg_state_callback, this);
    } else {
        ROS_ERROR_STREAM("[Package Stack]: Failed to spawn object");
        return true;
    }
    return false;
}

bool Object::set_pkg_state_callback(std_srvs::SetBool::Request &request,
                                    std_srvs::SetBool::Response &response) {
    if_picked_up_pkg = request.data;
    response.message = "ObjectStateUpdated";
    response.success = true;
    return true;
}

void Object::set_pose_of_pkg(geometry_msgs::Pose pkg_pose) {
    gazebo_msgs::ModelState msg;
    msg.model_name = pkg_name_;
    msg.pose = pkg_pose;
    msg.reference_frame = "world";
    pose_pub_.publish(msg);
}

void Object::publish_pkg_loc(const ros::TimerEvent&) {
    geometry_msgs::TransformStamped trans_stamp;
    if (if_picked_up_pkg) {
        geometry_msgs::TransformStamped trans_stamp;
        trans_stamp = tfbuffer_.lookupTransform("map",
                                                    "gripper_grasping_frame",
                                                                ros::Time(0));
        pkg_pose_.position.x = trans_stamp.transform.translation.x;
        pkg_pose_.position.y = trans_stamp.transform.translation.y;
        pkg_pose_.position.z = trans_stamp.transform.translation.z;
        pkg_pose_.orientation = trans_stamp.transform.rotation;
    } else {
        pkg_pose_.position.z = 0.025;
    }
    set_pose_of_pkg(pkg_pose_);
    trans_stamp.header.stamp = ros::Time::now();
    trans_stamp.header.frame_id = "map";
    trans_stamp.child_frame_id = pkg_name_;
    trans_stamp.transform.translation.x = pkg_pose_.position.x;
    trans_stamp.transform.translation.y = pkg_pose_.position.y;
    trans_stamp.transform.translation.z = pkg_pose_.position.z;
    trans_stamp.transform.rotation = pkg_pose_.orientation;
    broadcaster_.sendTransform(trans_stamp);
}


int main(int argc, char *argv[]) {
    // Initialize the node
    ros::init(argc, argv, "object_node");
    ROS_INFO_STREAM("[Package Stack]: Started object_node");
    ros::NodeHandle nh_;

    // spawn new object
    Object pkg(&nh_);
    pkg.spawn_pkg();

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
