# Inventory-Management-AGV

[![Build Status](https://github.com/sairampolina/Inventory-Management-AGV/actions/workflows/ci_v2.yml/badge.svg)](https://github.com/sairampolina/Inventory-Management-AGV/actions/workflows/ci_v2.yml/)
[![codecov](https://codecov.io/gh/sairampolina/Inventory-Management-AGV/branch/phase3/graph/badge.svg?token=YUMKWMA7D0)](https://codecov.io/gh/sairampolina/Inventory-Management-AGV)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

## Overview
### Manfred - The Automated Guided Vehicle

With the increase in eCommerce, the need for logistics and supply chain has only gone up. Warehousing becomes very challenging because manual operators are required to restock and move stuff around. Though conveyer belts were a convenient solution a few years back their inability to be dynamic and cemented at a specific location makes it challenging to use them in increasingly complex scenarios. This solution is often replaced with humans handling forklifts and moving the stacks around. With improvements in robotics technology, solutions can be developed to automate these machines which will save numerous man-hours and can also work around the hours thus saving the cost of shipment for both companies and customers. Manfred (project name) is an AGV for the task of inventory management in the warehouse which is tasked to move objects from one location to another after the user inputs the pickup and drop location.

<!--
### Goal
The development of AGV tasked for inventory management requires multiple modules, these modules include, communication with the user for pickup and drop location, navigation(to localize and navigate the robot in the industry environment, which will also have the map information in the human-readable form to take the pickup and drop input from the user(for example using aisle number etc.)) module for taking robot from current location to pickup location and later from the pickup to drop location, perception module to detect an object that needs to be picked up, manipulator planning and control to pick the package detected. Each of the modules will be developed independently in a modular form allowing it to be reused for several other applications in inventory management.
-->

### Aim of the Product
The aim of the product is to reduce the human hours required for logistics in the warehouse. The development of the product will also facilitate the development of future products by independently using the already developed modules, for example, a similar AGV can be developed for transporting stuff in any other indoor environment like replacing waiters in restaurants or automated vacuums. Thus most of these modules can be directly used in other products facilitating quicker development and reducing time to market.

### Module Description
The Software Product will consist of 4 modules each tasked with different aspects of the product, these modules are:
1. **Communication Module**: The purpose of this module is to establish communication with the user, the robot needs the input of pickup and drop location from the user, this will serve as the input for the later modules. This module can also be used as the kill switch for the robot in case of any problems.
2. **Path planning, localization and navigation module**: This module uses several sensors like cameras and lidar to create a map(gmapping ) of the warehouse. This map will help in localizing the current location of the robot and navigating the object to the desired location. This module also consists of obstacle detection which will help in avoiding collisions and dynamically change the path robot takes in case any obstacle is encountered. This module also controls the actuators which control the motion of the robot.
3. **Perception module**: This module will be used to locate the pickup object and the drop off location in the 3d world coordinates which will be used by the manipulator module to pickup or drop the object from the exact location. This module includes object detection and poses estimation of the desired object to facilitate the pickup of the object without damaging the product that needs to be picked up. Different sensors will be used to locate the objects in 3d world coordinates.
4. **Manipulator planning and control**: This module will take the 3d location and pose of the desired object from the previous module. This input will be used to plan and move the manipulator (robotic hand) to the extracted location and place the object back in a bin, the module will also have to inform the bot when the task is executed so that object can move to the next task of navigating to the drop off location.

<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/48856345/205377455-878763b3-81f5-4133-9c8d-d718491c88e8.jpg"> 
</p>
<h4 align="center">AGV with inventory management capabilities</h1>

<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/48856345/209449530-cbeb7fa3-c225-48d2-8808-645c1a402fcf.png"> 
</p>

<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/48856345/209449533-d17efb8a-f52e-42d4-bc37-4d1d4e29d341.png"> 
</p>

### Team
The Members are Graduate students at The University of Maryland, College Park. The members each have a Bachelors in Mechanical Engineering from prestigious universities in India. Their Graduate Study is in the field of Robotics with each of them having a specialization. Sanchit is pursuing a career in Computer vision and perception, Sairam aims to build his career in Deep Learning and Computer Vision and Shelvin is developing expertise in SLAM and Autonomous Vehicles.

| `Phase` | `Sanchit Tanwar` | `Shelvin Pauly` | `Venkata Sairam Polina` |
| --- | --- | --- | --- |
| 1 | Design Keeper | Navigator | Driver |
| 2 | Driver | Design Keeper | Navigator |
| 3 | Navigator | Driver | Design Keeper |

## Tools and Technologies
`Ubuntu 18.04(LTS)` `ROS` `Melodic` `Gazebo` `Rviz` `CMake` `OpenCV` `GitHubCI` `Codecov`
`Makefile` `CMake` `cpplint` `cppcheck` `clangd` `Valgrind` `GTest` `VScode`

## Dependencies

### Software Dependencies
 - [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
 - [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

### ROS Package Dependencies
- Gazebo
- Rviz
- [Tiago robot package and all other dependencies](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)

## Command-line installations

### Code Coverage
```
sudo apt-get install -y -qq lcov
```
### Static Code Analysis
``` 
sudo apt install cpplint
sudo apt install cppcheck
```
### Doxygen
```
sudo apt install graphviz
sudo apt install doxygen-gui
```

## Content Tree
<pre>
├── CMakeLists.txt
├── docs
│   ├── html
│   └── latex
├── finalproject_enpm808x_proposal.pdf
├── include
│   ├── manipulation.hpp
│   ├── my_tiago.hpp
│   ├── navigation.hpp
│   ├── object.hpp
│   └── perception.hpp
├── launch
│   └── tiago_launch.launch
├── LICENSE
├── package.xml
├── Quadchart
│   └── quad_chart_.pdf
├── README.md
├── results
│   ├── cppcheck.txt
│   └── cpplint.txt
├── src
│   ├── manipulation.cpp
│   ├── my_tiago.cpp
│   ├── navigation.cpp
│   ├── object.cpp
│   └── perception.cpp
├── test
│   ├── main.cpp
│   ├── main_test.test
│   ├── test_manipulation.cpp
│   ├── test_navigation.cpp
│   ├── test_object.cpp
│   └── test_perception.cpp
└── UML
    ├── initial
    └── revised 
</pre>


## Software Project Management 
- ```Process, Tools and Technologies, Risk Management, References``` - [Project Proposal](finalproject_enpm808x_proposal.pdf)

- ``` Implementation, Class Diagram, Activity Diagram``` - [UML Diagrams](UML/revised)

## Development Aspects
Agile Iterative Development Process was used to develop the simulation along with Test-Driven Development.

[Product Backlog and Sprint Sheet](https://docs.google.com/spreadsheets/d/1BZ1SMS-pVFIOvnL44yTZs3nGOwIKBSiaRsw7p3ZLkmk/edit#gid=0)

[Sprint Review Sheet](https://docs.google.com/document/d/102MQqVZMxTfVCX7L_Uc2SSMKDLDpSjjp1_cofrB92XM/edit#)

## To run the simulation
- Make sure all the above dependencies are installed in your system.
- Navigate in to the src folder of of your ROS workspace.

- Clone the package

```
git clone  https://github.com/sairampolina/Inventory-Management-AGV.git
```
- Build the package

```
cd ..
catkin build im_agv
```
- Source the workspace
```
source ./devel/setup.bash
```
- Run the launch file
```
roslaunch im_agv tiago_launch.launch
```

## To run unit tests
```
. /opt/ros/melodic/setup.bash
catkin config --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wno-unused --coverage -fno-inline -fno-inline-small-functions -fno-default-inline" -DCMAKE_C_FLAGS="-Wall -Wno-unused --coverage -fno-inline -fno-inline-small-functions -fno-default-inline" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXE_LINKER_FLAGS="-lgcov"
catkin build
catkin run_tests -i && catkin_test_results
```

## Static Code Analysis
Navigate in to the ROS package.Then run

1. cppcheck
```
cppcheck --enable=all src/*.cpp include/*.hpp test/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=unreadVariable --suppress=useInitializationList > ./results/cppcheck.txt
```

2. cpplint
```
cpplint --filter=-build/c++11,-build/namespaces,-build/include_order src/*.cpp include/*.hpp test/*.cpp &> ./results/cpplint.txt
```

## Code Coverage
```
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```

## Doxygen Documentation
In the terminal execute the command to open a GUI.
```
doxywizard
```
## Results

[Demonstration Video](https://drive.google.com/file/d/1_zlkNh7n82LKpOkjOCAaItQpVTASn1nq/view?usp=sharing)

[Final Presentation](https://docs.google.com/presentation/d/1NeBHGJK7w3UahWF0we489h2Lg_fpibfpUOyoCq6geCc/edit#slide=id.p)
## License
Apache License 2.0
```
DISCLAIMER: Copyright 2022 Sanchit Tanwar, Shelvin Pauly, Venkata Sairam Polina

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
    
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
