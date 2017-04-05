#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('robotic_arm_control')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

"""
Wrapper for UR5 robotic arm
Direct requests to the UR5 arm should be sent through this node

Before this, the UR driver needs to be brought online

rosrun ur_modern_driver ur5_bringup.launch robot_ip:=10.253.0.51
"""

def rad2pi(joint_states):
    """control tablet reads in joint positions in degrees
    JointTrajectoryPoint is in radians. we can define as either, but this will convert to radians"""
    return [(j*pi/180) for j in joint_states]

class Arm():
    def __init__(self):
        #Setting up the connection to UR5 server
        rospy.init_node("arm_node", anonymous=True, disable_signals=True)
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.JOINT_NAMES):
                self.JOINT_NAMES[i] = prefix + name

        #HOME position of the arm
        self.HOME = [0,-90,0,-90,0,0]

        #Gesture dictionary, and buliding it
        self.gestures = {}
        self.build_gesture_dict()

    def build_gesture_dict(self):
        """
        Gesture dictionary for UR arm
        format is
        [([JOINT_POSITIONS], GEST_DURATION), etc]
        """
        self.gestures["dance"] = [([83, -128, 40.75, 45,0,0], 2), ([83, -38, -50, 176.4, 0,0],2)]

    def run_gesture(self, gesture):
        """
        Runs a gesture based on what it finds in the dictionary
        """
        gest2run = self.gestures[gesture]
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            duration = 0

            #Add each gesture from the dictionary to the trajectory
            for gest in gest2run:
                duration += gest[1]
                g.trajectory.points.append(JointTrajectoryPoint(positions=rad2pi(gest[0]), velocities=[0]*6, time_from_start=rospy.Duration(duration)))

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed succesfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def move_to_point(self, point):
        """
        Creates trajectory to a single point
        """
        #TODO: implement
        pass

    def home_robot(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]

            #Add HOME position
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=rad2pi(self.HOME), velocities=[0]*6, time_from_start=rospy.Duration(3)))

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed succesfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def run(self):
        try:
            inp = raw_input("Ready to run? y/n: ")[0]
            if (inp == 'y'):
                self.run_gesture("dance")
            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

if __name__ == '__main__':
    a = Arm()
    a.run()
