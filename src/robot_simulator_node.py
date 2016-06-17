#!/usr/bin/env python

# Jacqueline Kory Westlund May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy # ROS
from sar_robot_command_msgs.msg import RobotCommand # ROS msgs
from sar_robot_command_msgs.msg import RobotState # ROS msgs
from std_msgs.msg import Header # standard ROS Header

# The SAR robot simulator node simulates a robot for the SAR Year5 study. It
# receives RobotCommand messages, sleeps to simulate executing speech and
# action commands that take time, and publishes RobotState messages to indicate
# when it is "doing" actions or speaking.
class robot_simulator_node():
    """ Robot simulator node """

    def __init__(self):
        """ Initialize anything that needs initialization """
        pass

    def run_robot_simulator_node(self):
        """ wait for robot commands and pass them on """
        # start ros node
        rospy.init_node('robot_sim_node', anonymous=True)
        rospy.loginfo("Robot simulation node starting up!")

        # subscribe to /robot_sim_command topic to get commands
        # for the simulated robot
        rospy.Subscriber('robot_sim_command', RobotCommand,
                self.on_robot_command_msg)

        # publish status messages to /robot_state topic
        self.state_pub = rospy.Publisher('robot_state', RobotState,
                queue_size = 10)
 
        # keep python from exiting until this node is stopped
        # TODO replace with a loop where we send RobotState messages
        rospy.spin()


    def on_robot_command_msg(self, data):
        """ We got a command! Sleep to simulate executing commands
        that take time. """
        rospy.loginfo("Got message:\n " + str(data))
        # do different things based on the command 
        if data.command == RobotCommand.SLEEP:
            # do nothing, set doing_action and playing_sound to false
            pass
        elif data.command == RobotCommand.WAKEUP:
            # do nothing, set doing_action and playing_sound to false
            pass
        elif data.command == RobotCommand.DO:
            # If the command contains <>, there's an action.
            # Set doing_action and/or playing_sound to true, then
            # sleep/wait and send state messages periodically
            pass


    def send_state_msg(self):
        """ Send a RobotState message """
        msg = RobotState()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # fill in state
        msg.doing_action = self.doing_action
        msg.is_playing_sound = self.playing_sound
        # send message
        self.state_pub.publish(msg)
        rospy.loginfo("Sending:\n" + str(msg))


if __name__ == '__main__':
    # run the node!
    node = robot_simulator_node()
    node.run_robot_simulator_node()
