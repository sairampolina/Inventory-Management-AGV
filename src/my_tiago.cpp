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

#include "../include/my_tiago.hpp"

MyTiago::MyTiago(ros::NodeHandle* nh):
                    navigator(nh),
                    manipulator(nh),
                    detector(nh),
                    tflistener_(this->tfbuffer_) {
    nh_ = nh;
    state_ = robotState::STARTING;
    ROS_INFO_STREAM(" Tiago object initialized");
}

void MyTiago::handle_states() {
    //  Checks if the state of the robot has been changed
    if (pre_state_ != state_) {
        ROS_INFO_STREAM("Tiago state: " << state_ << "________________");
        pre_state_ = state_;
    }

    //  State Switches to the respective state
    switch (state_) {
        case STARTING:
            // Create a point head action client to move the TIAGo's head
            create_point_head_client(point_head_client_);
            set_head_down();
            ROS_INFO_STREAM("Tiago Setting head position");
            state_ = IDLE;
            break;

        case IDLE:
            // Sets checkpoints for the robot
            navigator.set_next_goal();
            state_ = MOVING_TO_CHECKPOINT;
            break;

        case MOVING_TO_CHECKPOINT:
            // Moves the robot to the checkpoint
            if (detector.if_object_detected) {
                navigator.stop_robot();
                navigator.set_pkgloc_as_goal(get_object_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.if_goal_reached()) {
                navigator.turn_state = Navigation::rotation::ROTATING;
                state_ = TURNING_AROUND;
            }
            break;

        case TURNING_AROUND:
            // Turns the robot at the checkpoint
            if (detector.is_object_detected) {
                navigator.set_pkgloc_as_goal(get_object_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.turn_state ==
                                        Navigation::rotation::ROT_COMPLETE) {
                navigator.set_next_goal();
                state_ = MOVING_TO_CHECKPOINT;
            } else {
                navigator.turn_robot();
            }
            break;

        case MOVING_TOWARDS_OBJECT:
            //  Moves the robot towards the object
            if (is_obj_within_reach()) {
               navigator.stop_robot();
               state_ = PICKING_OBJECT;
            }
            break;

        case PICKING_OBJECT:
            // Acion of object picking
            pick_up_object();
            navigator.set_droploc_as_goal();
            state_ = MOVING_TO_BIN_LOCATION;
            break;

        case MOVING_TO_BIN_LOCATION:
            // Moving the object to the bin location
            if (navigator.if_goal_reached())
                state_ = PLACING_OBJECT;
            break;

        case PLACING_OBJECT:
            // Acion of object placing at the bin location
            place_object();
            ROS_INFO_STREAM(" Task accomplished!");
            state_ = STOP;
            break;
    }
}

bool MyTiago::is_obj_within_reach() {
    //  Checks if object is within reach of the robot arm.
    auto objectPose = get_object_pose("base_link");
    double dx = objectPose.position.x;
    double dy = objectPose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    ROS_INFO_STREAM(" object is at distance: " << distance);
    if (distance < 0.7) return true;
    return false;
}

geometry_msgs::Pose MyTiago::get_object_pose(std::string wrt) {
    //  Checks if object is within reach of the robot arm.
    geometry_msgs::Pose objectPose;
    if (detector.if_object_detected == false) {
        ROS_INFO_STREAM(" object has not been found yet.");
    } else {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer_.lookupTransform(wrt, "blue_box",
                                                            ros::Time(0));
        objectPose.position.x = transformStamped.transform.translation.x;
        objectPose.position.y = transformStamped.transform.translation.y;
        objectPose.position.z = transformStamped.transform.translation.z;
        // ROS_INFO_STREAM("[DCRobot] Got pose X: " << objectPose.position.x
        //                                 << ", Y: " << objectPose.position.y);
        objectPose.orientation = transformStamped.transform.rotation;
    }
    return objectPose;
}

void MyTiago::pick_up_object() {
    //  Picking up the object
    manipulator.move_to_object(get_object_pose("base_link"));
    manipulator.pick_package();
    ROS_INFO_STREAM(" Object Picked up");
}

void MyTiago::place_object() {
    //  Placing the object
    manipulator.place_package();
    manipulator.pick_package();
    ROS_INFO_STREAM(" Object Placed");
}

void MyTiago::set_head_down() {
    //  Setting the head orientation of the robot.
    std::string camera_frame = "/xtion_rgb_optical_frame";
    geometry_msgs::PointStamped pointStamped;
    pointStamped.header.frame_id = camera_frame;
    pointStamped.header.stamp    = ros::Time::now();
    pointStamped.point.x = 0.0;
    pointStamped.point.y = 0.8;    // value for looking down
    pointStamped.point.z = 1.0;

    // build the action goal
    control_msgs::PointHeadGoal goal;

    goal.pointing_frame = camera_frame;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    point_head_client_->sendGoal(goal);
    ros::Duration(0.5).sleep();
}

void MyTiago::create_point_head_client(PointHeadClientPtr& actionClient) {
    ROS_INFO(" Creating action client to head controller ...");
    actionClient.reset(
                    new PointHeadClient("/head_controller/point_head_action"));

    int iterations = 0, max_iterations = 3;
    // Wait for head controller action server to come up
    while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok()
                                            && iterations < max_iterations) {
        ROS_DEBUG(R"( Waiting for the point_head_action server to come
                                                                         up)");
        ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error(R"(Error in create_point_head_client: 
                                head controller action server not available)");
}
