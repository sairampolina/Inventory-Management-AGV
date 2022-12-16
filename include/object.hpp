#ifndef INCLUDE_OBJECT_HPP_
#define INCLUDE_OBJECT_HPP_

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



#include <std_srvs/SetBool.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SpawnModel.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <string>


class Object {
 public:
  explicit Object(ros::NodeHandle*);
  bool spawn_pkg();
  void set_pose_of_pkg(geometry_msgs::Pose);
  bool if_picked_up_pkg;

 private:
  bool set_pkg_state_callback(std_srvs::SetBool::Request&,
                            std_srvs::SetBool::Response&);

  void publish_pkg_loc(const ros::TimerEvent &);

  bool if_spawned;
  int map_range[4] = {-6, -7, 2, 7};

  ros::NodeHandle* nh_;
  ros::ServiceServer update_state_service_;
  ros::ServiceClient spawn_pkg_client_;
  ros::Timer pkg_pose_tf_timer_;
  ros::Publisher pose_pub_;

  std::string urdf_string_;
  geometry_msgs::Pose pkg_pose_;
  std::string pkg_name_;

  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;
};
#endif  // INCLUDE_OBJECT_HPP_
