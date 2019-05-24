#!/usr/bin/env python
# coding=utf-8
import baxter_interface
import rospy
from Baxter_env import move_baxter
from Baxter_env.geo import *

import tf
import math
from leap_hand_pos_ori_client import leap_pos_ori_client_srv
from geometry_msgs.msg import(
    Point,
    Quaternion
)
import time
rospy.init_node('leap_move')
rs = baxter_interface.RobotEnable()
rs.enable()
left = baxter_interface.Limb('left')
left.set_joint_position_speed(1.0)

left.move_to_joint_positions({'left_w0': 0.6699952259595108,
                                'left_w1': 1.030009435085784,
                                'left_w2': -0.4999997247485215,
                                'left_e0': -1.189968899785275,
                                'left_e1': 1.9400238130755056,
                                'left_s0': -0.08000397926829805,
                                'left_s1': -0.9999781166910306})
leftPose = left.endpoint_pose()

print 'init_pose:'
print leftPose

leap_2_base = FRAME(xyzrpy = [0.750000, 0.000000,
                    -0.129000, 1.57079632679, 0, -1.57079632679])
r = rospy.Rate(3)
while not rospy.is_shutdown():

    response = leap_pos_ori_client_srv()
    left_hand_in_leap_xyzrpy = [response.hand_palm_pos.x/500,
                                response.hand_palm_pos.y/500,
                                response.hand_palm_pos.z/500,
                                math.radians(response.hand_roll_pitch_yaw.x),
                                math.radians(response.hand_roll_pitch_yaw.y),
                                math.radians(response.hand_roll_pitch_yaw.z)]
    gripper = response.gripper

    left_end_in_base = leap_2_base.__mul__(FRAME(xyzrpy = left_hand_in_leap_xyzrpy)).xyzrpy()
    print "------------------------------------"
    print "left_end_in_base xyz: \nx = %f, \ny = %f,\nz = %f"%(left_end_in_base[0],left_end_in_base[1],left_end_in_base[1])

    # (left_ori_x, left_ori_y, left_ori_z, left_ori_w) = tf.transformations.quaternion_from_euler(
    #                                                                     left_end_in_base[3],
    #                                                                     left_end_in_base[4],
    #                                                                     left_end_in_base[5]
    #                                                                     )

    info = move_baxter.moveAbs(
        'left',
        Point(left_end_in_base[0],
              left_end_in_base[1],
              left_end_in_base[2],),
        Quaternion(leftPose['orientation'].x,
                   leftPose['orientation'].y,
                   leftPose['orientation'].z,
                   leftPose['orientation'].w))
    
    if info == 0:
        print 'move success.'
    elif info == -1:
            print 'ik_failed.'

    r.sleep()