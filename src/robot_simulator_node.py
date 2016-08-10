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
import datetime # for getting time deltas for timeouts
import time # for sleep statements
import signal # for catching SIGINT
import os # so we can make a system call to the TTS client
import re # So we can remove actions from robot DO before calling TTS

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
        # Start ros node.
        rospy.init_node('robot_sim_node', anonymous=True)
        rospy.loginfo("Robot simulation node starting up!")

        # Subscribe to /robot_sim_command topic to get commands
        # for the simulated robot.
        rospy.Subscriber('robot_sim_command', RobotCommand,
                self.on_robot_command_msg)

        # Publish status messages to /robot_state topic.
        self.state_pub = rospy.Publisher('/sar/robot_state', RobotState,
                queue_size = 10)

        # Set up flags.
        self.doing_action = False
        self.playing_sound = False
        # Flag to indicate whether we should exit.
        self.stop = False
        # Initialize start time
        self.start_time = None

        # Set up signal handler to catch SIGINT (e.g., ctrl-c).
        signal.signal(signal.SIGINT, self.signal_handler)

        # Main loop.
        while not self.stop:
            # Check flags to see if we should switch their state.
            if (self.doing_action or self.playing_sound) and \
                self.start_time is not None and \
                datetime.datetime.now() - self.start_time >= self.timeout:
                self.doing_action = False
                self.playing_sound = False
                # Reset start time so we don't keep setting these flags
                # after they're already set. Also so the TTS, if it
                # starts or stops working after we've used the timeout,
                # will continue setting the flags correctly.
                self.start_time = None

            # Send state message.
            self.send_state_msg()

            # Sleep a little.
            time.sleep(1)


    def signal_handler(self, sig, frame):
        """ Handle signals caught """
        if sig == signal.SIGINT:
            rospy.loginfo("Got keyboard interrupt! Exiting.")
            self.stop = True


    def on_robot_command_msg(self, data):
        """ We got a command! Sleep to simulate executing commands
        that take time. """
        rospy.loginfo("Got message:\n " + str(data))
        # Do different things based on the command.

        # SLEEP command
        if data.command == RobotCommand.SLEEP:
            # Do nothing, set doing_action and playing_sound to false
            # TODO change if it turns out that sleeping is considered
            # an action (e.g., if a sleep animation is playing).
            self.doing_action = False
            self.playing_sound = False

        # WAKEUP command
        elif data.command == RobotCommand.WAKEUP:
            # Assume a short wakeup behavior with sound and action
            # is played on waking up.
            self.doing_action = True
            self.playing_sound = True

            # Set the timeout for the sound and action flags to an
            # arbitrarily picked value of 2 seconds.
            self.timeout = datetime.timedelta(seconds=2)
            self.start_time = datetime.datetime.now()

        # DO command
        elif data.command == RobotCommand.DO:
            # TODO Right now, we set the sound and action flags for
            # the whole arbitrarily set time. One could do more
            # sophisticated parsing of DO commands, such that command
            # flags (e.g., blocking or non-blocking actions) are taken
            # into account. Then the action flag could be set only for
            # certain times, rather than the whole time.

            # If the command contains angle brackets <>, there's an
            # action for the robot to do.
            if "<" in data.properties:
                self.doing_action = True

            # If the start/end of the string are not angle brackets,
            # or, if they are but there are multiple sets of brackets,
            # assume we probably have speech too.
            if (not data.properties.startswith("<") \
                    or not data.properties.endswith(">")) \
                or (data.properties.startswith("<") \
                        and data.properties.endswith(">") \
                        and data.properties.count("<") > 1):
                self.playing_sound = True
                # Use TTS to say the command, but remove any actions
                # first so it only says the speech part.
                self.tts(re.sub(r'<.+?>', '', data.properties))


    def send_state_msg(self):
        """ Send a RobotState message """
        msg = RobotState()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Fill in state.
        msg.doing_action = self.doing_action
        msg.is_playing_sound = self.playing_sound
        # Send message.
        self.state_pub.publish(msg)
        rospy.loginfo("Sending:\n" + str(msg))


    def tts(self, to_say):
        """ Use built-in text-to-speech to have a computer read aloud
        the robot's speech (simulating the robot saying it).
        """
        try:
            # We are using the spd-say TTS client, which is generally
            # installed by default on Ubuntu 14.04 systems. Feel free
            # to add other TTS clients here if they would work better
            # for you, such as different ones based on what OS we are
            # running on.
            rospy.loginfo("Going to say: " + to_say)
            os.system("spd-say -w -p 10 -t child_female " + to_say)
            rospy.loginfo("Done speaking!")
            # Done playing sounds and doing actions.
            self.playing_sound = False
            self.doing_action = False
        except Exception as e:
            rospy.logerror("Tried to use spd-say TTS but failed! This may " +
                "happen if you don't have spd-say installed. Exception: " + e)
            # If we don't have TTS to use, set the timeout for the
            # sound and action flags to an arbitrarily picked value of
            # 5 seconds.
            self.timeout = datetime.timedelta(seconds=5)
            self.start_time = datetime.datetime.now()


if __name__ == '__main__':
    # run the node!
    node = robot_simulator_node()
    node.run_robot_simulator_node()
