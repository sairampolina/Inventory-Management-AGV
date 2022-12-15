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

#ifndef INCLUDE_MANIPULATION_HPP_
#define INCLUDE_MANIPULATION_HPP_

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

class Manipulation {
    public:

        /**
         * @brief Construct a new Manipulation object
         * @param nh 
         */

        explicit Manipulation(ros::NodeHandle*);

        
        /**
        * @brief Handles pickup of picked object.
        * 
        */

        void pick_package();

        /**
         * @brief  Handles placing of picked object.
         * 
         */
        void place_package();

        /**
         * @brief  Handles movement of object and takes input the pose of detected object.
         * @param pose of the object
         */
        void move_to_object(geometry_msgs::Pose);

    private:

        
        ros::NodeHandle* nh_; // Node handle
        
        ros::ServiceClient set_obj_state_client_; // Service client to set object state

        /**
         * @brief Handles reaching of object and takes input the pose of detected object.
         * @param pose of the object
         */
        void reach(geometry_msgs::Pose); 


};