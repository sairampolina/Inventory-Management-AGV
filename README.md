# Inventory-Management-AGV

<!-- [![Build Status](https://app.travis-ci.com/spauly98/ENPM808X_Midterm_project.svg?branch=phase2)](https://app.travis-ci.com/github/bharadwaj-chukkala/ENPM808X_Midterm_project)
[![Coverage Status](https://coveralls.io/repos/github/spauly98/ENPM808X_Midterm_project/badge.svg?branch=phase2)](https://coveralls.io/github/spauly98/ENPM808X_Midterm_project?branch=phase2) -->

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview
### ACME Robotics Problem Statement
Obstacle Detection has become an essential need in today’s technologically advanced world and the need for Automation has fueled this application. In day to day life we notice many accidents on the shopfloor between humans and semi-autonomous machinery. Assuming an industrial environment/warehouse setup where there is constant human-robot interaction on the ground which is a likely place prone to accidents, ACME Robotics needs to upgrade their AGV such that these accidents are avoided as much as possible. The workfloor/shopfloor contains automated guided vehicles for the transportation of mechanical/electrical components and other products. There is an urgent need for a way to avoid imminent collisions between humans and AGVs. Although the pedestrians on the shopfloor/warehouse can be wary of the AGVs there has to be a way for the AGVs to detect human obstacles to avoid collision and collateral damage. 
### Goal
We are going to develop a perception module that will be interfaced with the ACME Robotics AGV to constantly detect and track moving humans. This module has to  be developed such that it seemlessly integrates with all the other subsystems of the AGV and provide continuous data feedback to avoid collision. The algorithm can be extended to detect other AGVs and Environment Variables so that the work floor is safe.
### Aim of the Product
This ACME Robotics Product will use the Perception Module vision data alongside the sensor feedback, to avoid collision and choose a different path to traverse through the environment based on human detection and tracking from the continuous video input data stream, thus avoiding human collision by maintaining safe distance.
### Perception Module Description
The Software Product will consist of 4 modules which will take a continuous video input and will provide bounded output after detecting humans and will track the humans as they move in real time.
  - Perception Module will initialize all the software modules and has a functionality to integrate everything together 
  - Data Reader Module will take continuous video input, resize the frame and perform image transformations and then return the output to Classifier Module.
  - Classifier Module works using an SVM Model, To train the model we generate a Histogram of Gradients for the given dataser. The generated feature vectors will be used to train the SVM Classifier. The Classifier will detect the location of the feature (Human) in the input frames with an error bound. The Output Error will be calculated between the ground truth and the classified output and it will be iterated through and through to minimize error and improve feature detection accuracy. After the desired accuracy is being reached, the Classifer Module will help the Human Detector Module to draw bounding boxes as it keeps classifying the human in the input.
  - Human Detector Module will detect the person in the input as frames are extracted, it will draw bounding boxes as identification of the detected person from the classifier output and return the bounding box output data for each frame continuously.


<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/106445479/198857946-d84752f5-9db8-45dc-8b48-200fb6bf2f12.jpg">
</p>
<h4 align="center">Advanced AGV with Perception capabilities</h1>


---
## License
MIT License

Copyright (c) 2022 Bharadwaj Chukkala, Shelvin Pauly, Venkata Sairam Polina
```
DISCLAIMER: Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
---
### Team
The Members are Graduate students at The University of Maryland, College Park. The members each have a Bachelors in Mechanical Engineering from prestigious universities in India. Their Graduate Study is in the field of Robotics with each of them having a specialization. Sanchit is pursuing a career in Control systems and Path planning, Sairam aims to build his career in Deep Learning and Computer Vision and Shelvin is developing expertise in SLAM and Autonomous Vehicles.

| `Shelvin Pauly`  | `Venkata Sairam Polina` | `Sanchit Tanwar` |
| ------------- | ------------- |------------- |
| 118426556 | 118436579 |  |
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
- ```Process, Tools and Technologies, Risk Management, References``` - [Project Proposal](https://github.com/bharadwaj-chukkala/ENPM808X_Midterm_project/blob/master/Project%20Proposal.pdf)
- ```Design, Initial Product Backlog (ESC), Assigning Responsibilities (CRC)``` - [Software Development utils](https://github.com/bharadwaj-chukkala/ENPM808X_Midterm_project/tree/master/Design_Neccesities)
- ``` Implementation, Class Diagram, Activity Diagram``` - [UML Diagrams](https://github.com/bharadwaj-chukkala/ENPM808X_Midterm_project/tree/master/UML%20Diagrams)
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

