#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def homo_transform (alpha, a, d, theta):
    T = Matrix([[           cos(theta),           -sin(theta),           0,             a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d], 
                [                    0,                     0,          0,              1]])
    return T

def rot_x (q):
    T = Matrix([[1,      0,       0, 0],
                [0, cos(q), -sin(q), 0],
                [0, sin(q),  cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_y (q):
    T = Matrix([[ cos(q), 0, sin(q), 0],
                [      0, 1,      0, 0],
                [-sin(q), 0, cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_z (q):
    T = Matrix([[cos(q), -sin(q), 0, 0],
                [sin(q),  cos(q), 0, 0],
                [     0,       0, 1, 0],
                [0,      0,       0, 1]])
    return T

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Define DH param symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            
        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

      
        # Modified DH params
        s = {alpha0:     0, a0:      0, d1:  0.75,               
             alpha1: -pi/2, a1:   0.35, d2:     0,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:   1.5,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}
            
        # Define Modified DH Transformation matrix


        # Create individual transformation matrices
        """
        T01 = homo_transform(alpha0, a0, d1, q1)
        T12 = homo_transform(alpha1, a1, d2, q2)
        T23 = homo_transform(alpha2, a2, d3, q3)
        T34 = homo_transform(alpha3, a3, d4, q4)
        T45 = homo_transform(alpha4, a4, d5, q5)
        T56 = homo_transform(alpha5, a5, d6, q6)
        T6G = homo_transform(alpha6, a6, d7, q7)

        T01 = T01.subs(s)
        T12 = T12.subs(s)
        T23 = T23.subs(s)
        T34 = T34.subs(s)
        T45 = T45.subs(s)
        T56 = T56.subs(s)
        T6G = T6G.subs(s)

        # T0G = simplify(T01 * T12 *T23 * T34 * T45 * T56 * T6G)
        """


        R01 = rot_x(alpha0) * rot_z(q1)            
        R12 = rot_x(alpha1) * rot_z(q2)            
        R23 = rot_x(alpha2) * rot_z(q3)            
        R03 = simplify(R01 * R12 * R23)
        R03 = R03.subs(s)

        # correction between urdf and DH parameters
        Tcorr = simplify(rot_z(pi) * rot_y(-pi/2))

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()


            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            Trpy = simplify(rot_z(yaw) * rot_y(pitch) * rot_x(roll) * Tcorr)

            wx = px - d7 * Trpy[0,2]   
            wy = py - d7 * Trpy[1,2]   
            wz = pz - d7 * Trpy[2,2]   

            wx = wx.subs(s)
            wy = wy.subs(s)
            wz = wz.subs(s)

            rad1 = atan2(wy, wx)
            theta1 = rad1

            u = a2
            v = sqrt(a3 * a3 + d4 * d4)
            c1 = sqrt(wx * wx + wy * wy) - a1
            c2 = wz - d1
            w2 = c1 * c1 + c2 * c2
            w = sqrt(w2)

            rad2 = -(atan2(c2, c1) + acos((u*u + w2- v*v)/(2*u*w)))
            rad2 = rad2.evalf(subs=s)
            theta2 = rad2 + np.pi / 2

            rad3 = pi/2 - acos((u*u + v*v - w2)/(2*u*v)) + atan2(a3, d4)
            rad3 = rad3.evalf(subs=s)
            theta3 = rad3

            R36 = simplify(R03.T * Trpy)
            R36 = R36.subs(s)
            R36 = R36.evalf(subs={q1: rad1, q2: rad2, q3: rad3})

            if R36[1,2] == 1:
               rad4 = 0
               rad5 = 0
               rad6 = atan2(-R36[0,1], R36[0,0])
            elif R36[1,2] == -1:
               rad4 = 0
               rad5 = pi
               rad6 = atan2(R36[0,1], -R36[0,0])
            else:
               rad4 = atan2( R36[2,2], -R36[0,2])
               rad6 = atan2(-R36[1,1],  R36[1,0])
               rad5 = atan2(R36[1,0]/cos(rad6), R36[1,2])

            theta4 = rad4
            theta5 = rad5
            theta6 = rad6


            # Populate response for the IK request
            # In the next line replace q1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
