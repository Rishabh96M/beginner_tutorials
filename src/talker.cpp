/**
 * Copyright (c) 2021 Rishabh Mukund
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <talker.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <sstream>
#include "std_msgs/String.h"
#include "beginner_tutorials/string.h"

// Default string
extern std::string message = "This is a robot!!";

/**
 * @brief A function that provides a service of changing the output string of the publisher
 * 
 * @param req service request
 * @param res service response
 * @return returns true once it changes the string
 */
bool changeOutput(beginner_tutorials::string::Request &req,
         beginner_tutorials::string::Response &res) {
  message = req.new_string;

  ROS_DEBUG_STREAM("String was changed to : " << message.c_str());
  res.res_s = req.new_string;
  return true;
}

/**
 * A simple ros publisher and subscriber.
 */

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // service created to change the output string
  ros::ServiceServer service = n.advertiseService("change_output",
  changeOutput);

  int freq = 10;
  freq = atoi(argv[1]);
  /// Checks if the rate is greater than 100
  if ( freq >= 100 ) {
    ROS_WARN_STREAM("high frequency, changing to 10");
    freq = 10;
  }
  /// Checks if the rate is equal to 0
  if ( freq == 0 ) {
    ROS_FATAL("low frequency, changing to 10");
    freq = 10;
  }
  /// Checks if the rate is negative
  if ( freq < 0 ) {
    ROS_ERROR("wrong frequency, changing to 10");
    freq = 10;
  }

  /// Set the frequency rate
  ROS_DEBUG_STREAM("Frequency set to : " << freq);
  ros::Rate loop_rate(freq);

  tf::TransformBroadcaster br;
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << message;
    msg.data = ss.str();

     ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    tf::Transform t;
    t.setOrigin(tf::Vector3(0.0, 10.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 90);
    t.setRotation(q);
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(),
    "world", "talk"));


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
