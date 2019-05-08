import rospy
import actionlib
import os
import time
from geometry_msgs.msg import Twist
from math import pi, sin, cos
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal
from sensor_msgs.msg import Image, JointState, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import message_filters
import ctypes
from numpy.ctypeslib import ndpointer
import numpy as np
from std_msgs.msg import String


global fusion

class EF():

    def __init__(self):

        self.controller = ctypes.CDLL("/home/varun/Humanoid/Project/semanticfusion/elasticfusionpublic/GUI/build/libElasticFusion.so")
        self.sendFrames = self.controller.addFrames

        self.thread = threading.Thread(target = self.controller.fun_start)
        self.thread.start()
        time.sleep(5)

    def process( self, color, depth):

        self.sendFrames(ctypes.c_void_p(carr.ctypes.data), ctypes.c_void_p(darr.ctypes.data))


class BaseMovements:

    def __init__(self):

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self.rate = 50
        self.r = rospy.Rate(self.rate)


    def move(self, dist, speed):

        move_cmd = Twist()
        move_cmd.linear.x = speed
        duration = abs(dist / speed)
        ticks = int(duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        self.cmd_vel.publish( Twist() )

    def turn(self, angle, angular_speed):
        move_cmd = Twist()
        goal_angle = angle * (2*pi/360)
        # angular_speed = 1.0
        if angle < 0:
            angular_speed *= -1
        angular_duration = goal_angle / angular_speed

        move_cmd = Twist()

        move_cmd.angular.z = angular_speed

        ticks = int(angular_duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        self.cmd_vel.publish( Twist() )



class HeadMovements():

    def __init__(self):

        self.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.joint_positions = [ 0.0, 0.0]
        self.client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to head_controller")

    def move(self, inp, duration = 5.0):

        self.joint_positions = inp
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(6.0))

class TorsoMovements():

    def __init__(self):

        self.joint_names = ["torso_lift_joint"]
        self.joint_positions = [ 4.0]
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to torso_controller")

    def move(self, dist, duration = 5.0):

        self.joint_positions = [ dist ]
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(6.0))

class ArmMovements():

    def __init__(self):

        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.joint_positions = [0, 0, 0, 0, 0, 0, 0]
        self.client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to arm_controller")

    def move(self, inp, duration = 5.0):

        self.joint_positions = inp
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(9.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(9.0))

class GripMovements():

    def __init__(self):

        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.client.wait_for_server()

    def move(self, position, effort, duration = 5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = effort
        gripper_goal.command.position = position

        self.client.send_goal(gripper_goal)
        self.client.wait_for_result(rospy.Duration(5.0))


def image_callback(rgb, depth):

    global fusion
    
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(rgb, desired_encoding="passthrough")
    rate = rospy.Rate(10)
    rate.sleep()
    
    img_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    rate = rospy.Rate(10)
    rate.sleep()

    # timestamp = rgb.header.stamp
    # print(timestamp)

    fusion.process(img, img_depth)
    

def move_thread(task, head):
    #head.move( [0, pi/4.0])
    task.move( 1, 0.2 )
    head.move( [0, pi/6.0])
    task.turn(100, 1.0)
    task.turn(-120, 0.5)

'''
def sub1():
    rospy.Subscriber('head_camera/rgb/image_raw', Image, image_callback, ("1,"))
    rospy.spin()

def sub2():
    rospy.Subscriber('head_camera/depth_registered/image_raw', Image, image_callback_d, ("2",))
    rospy.spin()
'''

if __name__ == "__main__":

    global fusion
    fusion = EF()

    rospy.init_node("motion_demo")

    task = BaseMovements()
    head = HeadMovements()
    x = threading.Thread(target=move_thread, args=(task,head))
    x.start()

    depth_sub = message_filters.Subscriber('head_camera/depth_registered/image_raw', Image)
    rgb_sub = message_filters.Subscriber('head_camera/rgb/image_raw', Image)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)

    '''
    torso = TorsoMovements()
    torso.move(3.0)

    arm = ArmMovements()
    arm.move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    grip = GripMovements()
    grip.move(0.0, 10.0)
    grip.move(1.0, 10.0)
    '''
