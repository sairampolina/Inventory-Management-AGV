
#include<gtest/gtest.h>
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

#include "manipulation.hpp"

// bool message_received;
// geometry_msgs::PoseStamped pose_message;
// geometry_msgs::Twist velocity_message;

// TEST(TestManipulation, testPickPackage) {

//     ros::NodeHandle nh;
//     Manipulation manip(&nh);


//     ASSERT_EQ(manip.flag, false);
    
// }