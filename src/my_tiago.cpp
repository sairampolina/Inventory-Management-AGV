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
    state_ = robotState::START;
    ROS_INFO_STREAM("[Tiago Stack]: Tiago object initialized");
}

void MyTiago::execution_pipeline() {
    //  Updates the state when it has changed
    if (pre_state_ != state_) {
        ROS_INFO_STREAM("Tiago state: " << state_ << "________________");
        pre_state_ = state_;
    }

    //  Execution of pick and place pipeleine
    switch (state_) {
        case START:
            state_ = IDLE;
            break;

        case IDLE:
            // Sets waypoints for the robot
            navigator.set_goal();
            create_head_client(point_head_client_);
            state_ = MOVING_TO_WAYPOINT;
            break;

        case MOVING_TO_WAYPOINT:
            // Moves the robot to the assigned waypoint
            if (detector.if_obj_detected) {
                navigator.stop_robot();
                navigator.set_pkgloc_as_goal(get_pkg_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.if_goal_reached()) {
                navigator.rot_state_ = Navigation::rotation::ROT_START;
                state_ = ROTATE_AROUND;
            }
            break;

        case ROTATE_AROUND:
            // Turns the robot after reaching each waypoint
            if (detector.if_obj_detected) {
                navigator.set_pkgloc_as_goal(get_pkg_pose());
                state_ = MOVING_TOWARDS_OBJECT;
            } else if (navigator.rot_state_ ==
                                        Navigation::rotation::ROT_COMPLETE) {
                navigator.set_goal();
                state_ = MOVING_TO_WAYPOINT;
            } else {
                navigator.turn_robot();
            }
            break;

        case MOVING_TOWARDS_OBJECT:
            //  Moves the robot towards the object
            if (if_obj_within_reach()) {
               navigator.stop_robot();
               state_ = PICKING_OBJECT;
            }
            break;

        case PICKING_OBJECT:
            // picks the object
            pickup_pkg();
            navigator.set_droploc_as_goal();
            state_ = MOVING_TO_DROP_LOCATION;
            break;

        case MOVING_TO_DROP_LOCATION:
            // Moving the object to the drop location
            if (navigator.if_goal_reached())
                state_ = PLACING_OBJECT;
            break;

        case PLACING_OBJECT:
            // Placing the object
            place_pkg();
            ROS_ERROR("[Tiago Stack]: Task accomplished!");
            state_ = STOP;
            break;
    }
}

bool MyTiago::if_obj_within_reach() {
    //  Checks if object is within reach of the robot arm.
    auto pkgPose = get_pkg_pose("base_link");
    double dx = pkgPose.position.x;
    double dy = pkgPose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    ROS_INFO_STREAM(" object is at distance: " << distance);
    if (distance < 0.7) return true;
    return false;
}

geometry_msgs::Pose MyTiago::get_pkg_pose(std::string wrt) {
    //  Chcks if pks is closer to the robot
    geometry_msgs::Pose pkgPose;
    if (detector.if_obj_detected == false) {
        ROS_INFO_STREAM("[Tiago Stack]: object has not been found yet.");
    } else {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfbuffer_.lookupTransform(wrt, "box",
                                                            ros::Time(0));
        pkgPose.position.x = transformStamped.transform.translation.x;
        pkgPose.position.y = transformStamped.transform.translation.y;
        pkgPose.position.z = transformStamped.transform.translation.z;

        pkgPose.orientation = transformStamped.transform.rotation;
    }
    return pkgPose;
}

void MyTiago::pickup_pkg() {
    //  Picking up the package
    manipulator.move_to_object(get_pkg_pose("base_link"));
    manipulator.pick_package();
    ROS_INFO_STREAM("[Tiago Stack]: Object Picked up");
}

void MyTiago::place_pkg() {
    //  Placing the package
    manipulator.place_package();
    manipulator.pick_package();
    ROS_INFO_STREAM("[Tiago Stack]: Object Placed");
}

void MyTiago::move_head() {
    //  moves the head of the rbot down
    std::string camera_frame = "/xtion_rgb_optical_frame";
    geometry_msgs::PointStamped pointStamped;
    pointStamped.header.frame_id = camera_frame;
    pointStamped.header.stamp = ros::Time::now();
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

void MyTiago::create_head_client(PointHeadClientPtr& actionClient) {
    ROS_INFO("[Tiago Stack]: Creating action client to control head.");
    actionClient.reset(
                    new PointHeadClient("/head_controller/point_head_action"));

    int iterations = 0, max_iterations = 3;
    // Wait for head controller action server to come up
    while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok()
                                            && iterations < max_iterations) {
        ROS_DEBUG(R"([Tiago Stack]: Waiting for the point_head_action server to come
                                                                         up)");
        ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error(R"([Tiago Stack]: Error in create_head_client: 
                                head controller action server not available)");
}

int main(int argc, char *argv[]) {
    // Initialize the node
    ros::init(argc, argv, "tiago_node");
    ROS_INFO_STREAM(" Started tiago_node");
    ros::NodeHandle nh_;

    MyTiago tiago_agv(&nh_);  // Create MyTiago object

    ros::Duration(10).sleep();
    ros::Rate r(10);
    while (ros::ok()) {
        tiago_agv.execution_pipeline();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
