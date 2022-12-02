# Inventory-Management-AGV

<!-- [![Build Status](https://app.travis-ci.com/spauly98/ENPM808X_Midterm_project.svg?branch=phase2)](https://app.travis-ci.com/github/bharadwaj-chukkala/ENPM808X_Midterm_project)
[![Coverage Status](https://coveralls.io/repos/github/spauly98/ENPM808X_Midterm_project/badge.svg?branch=phase2)](https://coveralls.io/github/spauly98/ENPM808X_Midterm_project?branch=phase2) -->

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview
### Manfred - The Automated Guided Vehicle

With the increase in eCommerce, the need for logistics and supply chain has only gone up. Warehousing becomes very challenging because manual operators are required to restock and move stuff around. Though conveyer belts were a convenient solution a few years back their inability to be dynamic and cemented at a specific location makes it challenging to use them in increasingly complex scenarios. This solution is often replaced with humans handling forklifts and moving the stacks around. With improvements in robotics technology, solutions can be developed to automate these machines which will save numerous man-hours and can also work around the hours thus saving the cost of shipment for both companies and customers. Manfred is an AGV for the task of inventory management in the warehouse which is tasked to move objects from one location to another after the user inputs the pickup and drop location.

### Goal
The development of AGV tasked for inventory management requires multiple modules, these modules include, communication with the user for pickup and drop location, navigation(to localize and navigate the robot in the industry environment, which will also have the map information in the human-readable form to take the pickup and drop input from the user(for example using aisle number etc.)) module for taking robot from current location to pickup location and later from the pickup to drop location, perception module to detect an object that needs to be picked up, manipulator planning and control to pick the package detected. Each of the modules will be developed independently in a modular form allowing it to be reused for several other applications in inventory management.

### Aim of the Product
The aim of the product is to reduce the human hours required for logistics in the warehouse. The development of the product will also facilitate the development of future products by independently using the already developed modules, for example, a similar AGV can be developed for transporting stuff in any other indoor environment like replacing waiters in restaurants or automated vacuums. Thus most of these modules can be directly used in other products facilitating quicker development and reducing time to market.

### Module Description
The Software Product will consist of 4 modules each tasked with different aspects of the product, these modules are:
1. Communication Module: The purpose of this module is to establish communication with the user, the robot needs the input of pickup and drop location from the user, this will serve as the input for the later modules. This module can also be used as the kill switch for the robot in case of any problems.
2. Path planning, localization and navigation module: This module uses several sensors like cameras and lidar to create a map(NAV2 library ROS2) of the warehouse. This map will help in localizing the current location of the robot and navigating the object to the desired location. This module also consists of obstacle detection which will help in avoiding collisions and dynamically change the path robot takes in case any obstacle is encountered. This module also controls the actuators which control the motion of the robot.
3. Perception module: This module will be used to locate the pickup object and the drop off location in the 3d world coordinates which will be used by the manipulator module to pickup or drop the object from the exact location. This module includes object detection and poses estimation of the desired object to facilitate the pickup of the object without damaging the product that needs to be picked up. Different sensors will be used to locate the objects in 3d world coordinates.
4. Manipulator planning and control: This module will take the 3d location and pose of the desired object from the previous module. This input will be used to plan and move the manipulator (robotic hand) to the extracted location and place the object back in a bin, the module will also have to inform the bot when the task is executed so that object can move to the next task of navigating to the drop off location.


<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/106445479/198857946-d84752f5-9db8-45dc-8b48-200fb6bf2f12.jpg">
</p>
<h4 align="center">Advanced AGV with Perception capabilities</h1>


---
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
---
### Team
The Members are Graduate students at The University of Maryland, College Park. The members each have a Bachelors in Mechanical Engineering from prestigious universities in India. Their Graduate Study is in the field of Robotics with each of them having a specialization. Sanchit is pursuing a career in Computer vision and perception, Sairam aims to build his career in Deep Learning and Computer Vision and Shelvin is developing expertise in SLAM and Autonomous Vehicles.

| `Shelvin Pauly`  | `Venkata Sairam Polina` | `Sanchit Tanwar` |
| ------------- | ------------- |------------- |
| 118426556 | 118436579 | 119167717 |
| Navigator | Driver |  Design Keeper |

---
<!-- ## Content Tree
<pre>├── <font color="#3465A4"><b>app</b></font>
│   ├── CMakeLists.txt
│   ├── main.cpp
│   └── ReadData.cpp
├── <font color="#3465A4"><b>cmake</b></font>
│   └── CodeCoverage.cmake
├── CMakeLists.txt
├── dependencies.sh
├── <font color="#3465A4"><b>Design_Neccesities</b></font>
│   ├── CRC_Cards_Mid_Term.pdf
│   └── ESC_midtermproposal.pdf
├── <font color="#3465A4"><b>include</b></font>
│   ├── HumanClassifier.hpp
│   ├── <font color="#4E9A06"><b>HumanDetector.hpp</b></font>
│   ├── PerceptionModule.hpp
│   ├── <font color="#4E9A06"><b>ReadData.hpp</b></font>
│   └── RectandConfidence.hpp
├── Project Proposal.pdf
├── <font color="#75507B"><b>quad_chart.jpg</b></font>
├── README.md
├── <font color="#3465A4"><b>results</b></font>
│   └── cppcheck
├── <font color="#3465A4"><b>test</b></font>
│   ├── CMakeLists.txt
│   ├── main.cpp
│   ├── tempCodeRunnerFile.cpp
│   └── test.cpp
├── <font color="#3465A4"><b>test_data</b></font>
│   └── <font color="#75507B"><b>Lenna.png</b></font>
├── <font color="#3465A4"><b>UML</b></font>
│   ├── <font color="#3465A4"><b>initial</b></font>
│   └── <font color="#3465A4"><b>revised</b></font>
└── <font color="#3465A4"><b>vendor</b></font>
   └── <font color="#3465A4"><b>googletest</b></font></pre>
---
-->

## Software Project Management Plan Aspects
- ```Process, Tools and Technologies, Risk Management, References``` - [Project Proposal](https://github.com/sairampolina/Enpm808x-finalproject/blob/phase1/finalproject_enpm808x_proposal.pdf)

- ``` Implementation, Class Diagram, Activity Diagram``` - [UML Diagrams]()
<!--
---
## Deliverables
- Project: Perception Module [Human obstacle detector and tracker]
- Overview of proposed work including timeline, risks, and mitigations.
- UML diagrams
- Github repository with [README](./readme.md)
- TravisCI setup with code coverage using Coveralls.
- Valgrind Check for Memory Leaks.
- Git Version Control Workflow.
- Developer-level documentation.
---
-->

## Results
<!-- ### Image Result
![image_result](https://user-images.githubusercontent.com/106445479/198862636-5c1d19d4-fd54-41fc-9838-d262e331e8c9.jpeg)

### Video Result
![gif1](https://user-images.githubusercontent.com/106445479/198862649-732aea17-6d19-4a50-9b71-3572f0513b56.gif)


---
-->

## Development Aspects
Agile Iterative Development Process will be used to develop the software along Test-Driven Development.

### [Product Backlog and Sprint Sheet](https://docs.google.com/spreadsheets/d/1BZ1SMS-pVFIOvnL44yTZs3nGOwIKBSiaRsw7p3ZLkmk/edit#gid=0)
### [Sprint Review Sheet](https://docs.google.com/document/d/102MQqVZMxTfVCX7L_Uc2SSMKDLDpSjjp1_cofrB92XM/edit#)

### Software Dependencies
- OpenCV 4.6.0 (covered under the open-source Apache 2 License)
- Eigen 3.4 the Mozilla Public License 2.0
- GTest BSD 3-Clause "New" or "Revised" License
### Tools and Technologies
`Ubuntu 22.04(LTS)` `C++ 14+` `CMake` `OpenCV` `TravisCI` `Coveralls`
`Makefile` `CMake` `cpplint` `cppcheck` `clangd` `Valgrind` `GTest` `VScode`


### Installation via Command Line
```
# Code Coverage
sudo apt-get install -y -qq lcov
```

```
# OpenCV install
sudo apt-get install -y build-essential
sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
# Download v4.6.0
curl -sL https://github.com/Itseez/opencv/archive/4.6.0.zip > opencv.zip
unzip opencv.zip
cd opencv-4.6.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
cd ../../
```

``` 
# Static Code Analysis
sudo apt install cpplint
sudo apt install cppcheck
```

```
# Valgrind
sudo apt install valgrind
sudo apt-get install -y kcachegrind
```

```
# Doxygen
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```

<!--
### Build and Code Coverage
```
# Clone
git clone https://github.com/bharadwaj-chukkala/ENPM808X_Midterm_project.git
cd <path to repository>
mkdir build
cd build
cmake ..
make

# Run
Run tests: ./test/cpp-test
Run the perception module: ./app/shell-app
```

```
# Static Code Analysis
1. cppcheck
bash run_cppcheck.sh
2. cpplint
bash run_cpplint.sh
```
Note: Static Code Analysis Results are stored in `./results`

```
# Valgrind
valgrind --leak-check=full <path of the executable>
valgrind --tool=callgrind  ./app/shell-app
kcachegrind
```

```
# Code Coverage
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
firefox index.html
```
-->

### Doxygen Documentation
```
cd ..
doxygen doxygen.config
doxywizard
```
---
<!--## Project Videos
### [Final Video](https://drive.google.com/drive/folders/1NprRueNb3u7tsn6D-Y5_eI7PGIxM4owS?usp=share_link)
Note: The Microphone had some issue, apologies beforehand
### [Sprint 1 Video](https://drive.google.com/file/d/1hh5f50VzA4mIPJmc9IJAOi4N9bs-uyPp/view?usp=sharing)
Note: The Microphone had some issue, apologies beforehand
### [Proposal Video](https://drive.google.com/file/d/1BUSOc8T6gLknhAt8TJ6uW9MLCY49dpPH/view?usp=sharing)

-->

