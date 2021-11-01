[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

  
#  A simple ROS publisher & subscriber


C++ implementation of a simple ROS publisher & subscriber package. Vallidated on Ubuntu 18.04 LTS and ROS Melodic.

## Steps to run

Make a catkin workspace catkin_ws and run the following commands :
  
```
    cd <path_to_ws>/catkin_ws/src
    git clone https://github.com/llDev-Rootll/beginner_tutorials.git
    cd ../
    catkin_make
```

In a terminal run :
```
    roscore
```
This will initialize ROS

In a new terminal run : 
```
    source devel/setup.bash
    rosrun beginner_tutorials talker
```
This commands will start the publisher to a topic

In another terminal run : 
```
    source devel/setup.bash 
    rosrun beginner_tutorials listener
```
This command will start the subscriber to that topic 


## Running cpplint & cppcheck tests
Run the following command in the src directory of the project to generate cpplint results in **results** folder
 ```
    cpplint *
```
Run the following command in the src directory of the project to generate cppcheck results in **results** folder
```
    cppcheck *
```

