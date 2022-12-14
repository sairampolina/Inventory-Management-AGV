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


#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/Pose.h>

// C++ standard headers
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

#include "manipulation.hpp"
#include "navigation.hpp"
#include "perception.hpp"
#include "object.hpp"

// namespace client {
//     actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
// };

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction>
                                                            PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

class MyTiago {
    public:
        /**
        * @brief Construct a new DCRobot object
        * 
        */
        explicit MyTiago(ros::NodeHandle*);

        /**
        * @brief Synchronizes various tasks of the robot
        * 
        */
        void handle_states();

        /**
        * @brief Checks whether the object is within reach of the robot
        * 
        */
        bool if_obj_within_reach();

        /**
        * @brief Gets object pose from the world. 
        *    /**
        * @brief Construct a new DCRobot object
        * 
        */
    
        geometry_msgs::Pose get_object_pose(std::string wrt = "map");

        /**
        * @brief Handles object picking by the robot.
        * 
        */
        void pick_up_object();

        /**
        * @brief Handles object placement by the robot.
        * 
        */
        void place_object();

        /**
        * @brief Enumerations various robot functioning states.
        * 
        */
        enum robotState {
            STARTING,
            IDLE,
            MOVING_TO_CHECKPOINT,
            TURNING_AROUND,
            OBJECT_FOUND,
            MOVING_TOWARDS_OBJECT,
            PICKING_OBJECT,
            MOVING_TO_BIN_LOCATION,
            PLACING_OBJECT,
            STOP
        };

        /** Instance for Navigation class*/
        Navigation navigator;
        /** Instance for graspObj class*/
        Manipulation manipulator;
        /** Instance for DetectObject class*/
        PackageDetector detector;

    private:
    /**
        * @brief Enumerates various object states.
        * 
        */
        void set_head_down();

        //using namespace PointHeadClient;

        /**
        * @brief Enumerates various object states.
        * 
        */
        void create_point_head_client(PointHeadClientPtr&);

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