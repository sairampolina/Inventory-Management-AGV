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


# pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <string.h>
#include <vector>


// 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

class objectDetector {
    public:

        objectDetector(ros::NodeHandle*);

        bool find_obj();

        bool if_obj_detected;

        void image_callback();

    private:

    ros::NodeHandle* nh_;

    cv::Mat image_, image_hsv_, image_thresh_;

    image_transport::ImageTransport image_transport_;

    image_transport::Subscriber image_sub_;
    
    std::vector<std::vector<cv::Point>> contours_;

};