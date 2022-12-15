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
 * Ros headers
*/
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>


#include <string>

/**
 * boost library
*/
#include <boost/shared_ptr.hpp>

/**
 * custom libraries
*/
#include "../include/manipulation.hpp"
#include "../include/navigation.hpp"
#include "../include/perception.hpp"
#include "../include/object.hpp"



typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction>
                                                            PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

class MyTiago {
    public:
        /**
        * @brief Spawning MyTiago bot   
        * 
        */
        explicit MyTiago(ros::NodeHandle*);

        /**
        * @brief Starts the robot execution pipeline
        * 
        */
        void execution_pipeline();

        /**
        * @brief Checks if object is within reach of the robot, returns boolean.
        * 
        */
        bool if_obj_within_reach();

        /**
        * @brief Get the pose of the object using the perception stack.
        * @param wrt - string, frame of reference for the pose.
        */
            
        geometry_msgs::Pose get_pkg_pose(std::string wrt = "map");

        /**
        * @brief Handles pickup of detected object.
        * 
        */
        void pickup_pkg();

        /**
        * @brief Handles placement of picked object.
        * 
        */
        void place_pkg();

        /**
        * @brief Enumerates various robot's states.
        * 
        */
        enum robotState {
            START,
            IDLE,
            MOVING_TO_WAYPOINT,
            ROTATE_AROUND,
            OBJECT_FOUND,
            MOVING_TOWARDS_OBJECT,
            PICKING_OBJECT,
            MOVING_TO_DROP_LOCATION,
            PLACING_OBJECT,
            STOP
        };

        /** Instance for Navigation class*/
        Navigation navigator;
        /** Instance for Manipulator class*/
        Manipulation manipulator;
        /** Instance for PackageDetector class*/
        PackageDetector detector;

    private:
       /**
        * @brief Moves the head to look at a point in space.
        * 
        */
        void move_head();

        //using namespace PointHeadClient;

        /**
        * @brief Creates a head client for head movement actions.
        * @param point_head_client_ - PointHeadClientPtr
        */
        void create_head_client(PointHeadClientPtr&);

        /** Node Handle created*/
        ros::NodeHandle* nh_;
        /** TF buffer created*/
        tf2_ros::Buffer tfbuffer_;
        /** TF listener created*/
        tf2_ros::TransformListener tflistener_;
        /** PointHeadClientPtr for head movement actions*/
        PointHeadClientPtr point_head_client_;
        /** robotState for checking current state of the robot */
        robotState state_;
        /** robotState for storing previous states of the robot*/
        robotState pre_state_;
};