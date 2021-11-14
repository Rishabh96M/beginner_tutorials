[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

  
#  A simple ROS publisher & subscriber
C++ implementation of a simple ROS publisher & subscriber package. Vallidated on Ubuntu 20.04 LTS and ROS Noetic.

## Steps to run the publisher and subscriber
Make a catkin workspace catkin_ws and run the following commands :
  
```
cd <path_to_ws>/catkin_ws/src
git clone https://github.com/Rishabh96M/beginner_tutorials.git
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
source catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker <freq>
```
This commands will start the publisher to a topic, <freq> is an integer value to set the frequency of messages to be published
Example:
```
rosrun beginner_tutorials talker 10
```

In another terminal run: 
```
source catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```
This command will start the subscriber to that topic

## Servic call to change the string being published
With the roscore, talker and listener running, in a new terminal run:
```
rosservice call /change_output "xxx"
```
Replace xxx with your desired message.
Example:
```
rosservice call /change_output "Hi Robot!"
```
## To inspect TF frames
Talker is broadcasting a static TF frame /talk with parent /world, this can be inspected by running the talker node, and in a new terminal run:
```
rosrun tf tf_echo /world /talk
```
The TF tree can be viewed by running the following command in a new terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```

## Running the rostests
To run the test to check for ROS nodes, first we have to make the tests using:
```
catkin_make tests
```
after the tests are successfullt integrated, run the following command to test the node:
```
catkin_make test
```

## Steps to start the nodes using the launch file
You can launch the publisher subscriber using a launch file and set the frequency and recording status using arguments by running the command:
```
source catkin_ws/devel/setup.bash
roslaunch beginner_tutorials chatter.launch rate:=xx record:=yy
```
Replace xx by any desired integer, to change the frequency of publisher
Replace yy by either 'true' or 'false' depending on if you want to record the session for 15s
Example:
```
roslaunch beginner_tutorials chatter.launch rate:=10 record:=true
```

Note: The recorded file will be saved in **results** folder with the name "rosbag_chatter_results.bag"

## Checking the information of bag file
To check the information of the bag file, run the following command in the location where bag file is saved:
```
rosbag info rosbag_chatter_results.bag
```

## Playing from the bag file
With roscore and the listener node running, in a new terminal run:
```
source catkin_ws/devel/setup.bash
rosbag play rosbag_chatter_results.bag
```

## Running cpplint & cppcheck tests
Run the following command in the src directory of the project to generate cpplint results, the ouput is stored in the **results** folder
```
cpplint $( find . -name \*.h -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```
Run the following command in the src directory of the project to generate cppcheck results, the ouput is stored in the **results** folder
```
cppcheck --language=c++ --std=c++11 -I include/ --suppress=missingIncludeSystem  $( find . -name \*.h -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```

The **results** folder contains the results of cpplint, cppcheck, screenshot of rqt console (for logs) and frames.pdf
