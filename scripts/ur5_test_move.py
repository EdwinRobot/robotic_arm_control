#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('robotic_arm_control')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

class Arm():
    def __init__(self):
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
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

        print self.JOINT_NAMES

        self.HOME = [0,-90,0,-90,0,0]

        self.gestures = {}
        self.build_gesture_dict()

    def rad2pi(self, joint_states):
        """control tablet reads in degrees
        JointTrajectoryPoint is in radians. we can define as either, but this will convert to radians"""
        return [(j*pi/180) for j in joint_states]

    def build_gesture_dict(self):
        self.gestures["dance"] = [[83, -128, 40.75, 45,0,0], [83, -38, -50, 176.4, 0,0]]

    def run_gesture(self, gesture):
        gest = self.gestures["dance"]
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            d = 2.0
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            for i in range(3):
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=self.rad2pi(gest[0]), velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 2
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=self.rad2pi(gest[1]), velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 2
            g.trajectory.points.append(JointTrajectoryPoint(positions=self.rad2pi(self.HOME), velocities=[0]*6, time_from_start=rospy.Duration(d)))
            self.client.send_goal(g)
            print "sent goal, waiting for result"
            self.client.wait_for_result()
            print "wait over"
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def move_one(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=self.Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=self.Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
                JointTrajectoryPoint(positions=self.Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
                JointTrajectoryPoint(positions=self.Q1, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),
                JointTrajectoryPoint(positions=self.Q2, velocities=[0]*6, time_from_start=rospy.Duration(6.0)),
                JointTrajectoryPoint(positions=self.HOME, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
            self.client.send_goal(g)
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def move_repeated(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            d = 2.0
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            for i in range(10):
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=self.Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 1
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=self.Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 1
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=self.Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
                d += 2
            self.client.send_goal(g)
            print "sent goal, waiting for result"
            self.client.wait_for_result()
            print "wait over"
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def run(self):
        try:
            inp = raw_input("Continue? y/n: ")[0]
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
