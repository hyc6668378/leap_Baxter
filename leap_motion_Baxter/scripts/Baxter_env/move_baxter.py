#!/usr/bin/env python
#coding=utf-8

#######################
# 2018/08 Hiroki Asano
# Suehiro Kudoh lab
#
# 2019/05 Huang
# Suehiro Kudoh lab
#######################

import baxter_interface
import rospy
import ik_solver
import math
from geometry_msgs.msg import(
    Point,
    Quaternion,
)

def moveAbs(arm, goalP, goalQ):
    """
    arm: 选择执行end-effector  'left' or 'right'
    goalP: 目标位置 x,y,z
    goalQ: 目标姿势 四元组

    return:  0  move success
            -1  ik无解 or 呼叫ikservice失败
            -2  参数不合法
    """

    # 参数检验
    if isinstance(goalP, Point):
        locP = goalP
    else:
        print("error: 参数'goalP'不是'Point'类实体")
        return -2
    if isinstance(goalQ, Quaternion):
        locQ = goalQ
    else:
        print("error: 参数'goalQ'不是'Quaternion'类实体")
        return -2
    if arm != 'left' and arm != 'right':
        print('error: arm param invalid')
        return -2 # arm param error

    # 请求ik服务,若ik有解则执行action
    if arm == 'left':
        limb_joints = ik_solver.ik_solve('left', locP, locQ)
        # if ik have solution
        if limb_joints != 1 and limb_joints != -1:
            try:
                baxter_interface.Limb('left').move_to_joint_positions(limb_joints)
            except:
                print "some Error in  baxter_interface.moveAbs"
                return -1 # move failed.
            else:
                return 0 # move success
        else:
            return -1 # limb_joints 非法. move failed.

            
    if arm == 'right':
        limb_joints = ik_solver.ik_solve('right', locP, locQ)
        # if ik have solution
        if limb_joints != 1 and limb_joints != -1:
            baxter_interface.Limb('right').move_to_joint_positions(limb_joints)
            return 0 # move success
        else:
            return -1 # move failed.