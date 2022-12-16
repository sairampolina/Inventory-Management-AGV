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

#include "../include/perception.hpp"

PackageDetector::PackageDetector(ros::NodeHandle* nh_):
                                image_transport_(*nh_) {
    image_sub_ = image_transport_.subscribe
        ("xtion/rgb/image_raw", 1, &PackageDetector::image_callback,
            this, image_transport::TransportHints("compressed"));
    if_obj_detected = false;
    cv::namedWindow("camera_feed", 0);
    ROS_INFO_STREAM("[Perception Stack]: Detector object initialized");
}

bool PackageDetector::find_obj() {
    cv::cvtColor(image_, image_hsv_, cv::COLOR_BGR2HSV);

    cv::inRange(image_hsv_, cv::Scalar(69, 50, 0),
        cv::Scalar(120, 255, 255), image_thresh_);

    cv::findContours(image_thresh_, contours_, CV_RETR_EXTERNAL,
                        CV_CHAIN_APPROX_SIMPLE);

    if (contours_.size() > 0) {
        if_obj_detected = true;
    }

    ROS_ERROR("[Perception Stack]: Object initialized");

    cv::imshow("camera_feed", image_thresh_);
    cv::waitKey(1);
    return true;
}

void PackageDetector::image_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cvPtr;

    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("erroe in retreiving image: %s", e.what());
        return;
    }

    cvPtr->image.copyTo(image_);
    this->find_obj();
}
