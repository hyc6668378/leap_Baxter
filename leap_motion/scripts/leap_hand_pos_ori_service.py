#!/usr/bin/env python
# coding:utf-8

import rospy
import rosparam
from leap_motion.srv import *
from leap_motion.msg import leapros
from geometry_msgs.msg import (
    Point,
    Vector3
)

class leap_pos_ori_server:
    def __init__(self):
        # 订阅接收 'leapmotion/raw' Topic 
        self.leap_sub = rospy.Subscriber("leapmotion/data", leapros, self.get_leap_sub)
        
        # 定义service的server端，service名称为 "leapmotion/pos_ori_server"
        # service类型为 'hand_pos_ori' 
        # handle_function 为 'self.send_leap_pos_ori'
        self.leap_pos_ori_srv = rospy.Service('leapmotion/pos_ori_server', hand_pos_ori, self.send_leap_pos_ori)
        print 'leapmotion/pos_ori_server has Ready to handle the Leap request. '

    def get_leap_sub(self, data):
        # 更新服务端 frame 数据
        self.hand_direction = data.direction
        self.hand_normal = data.normal
        self.hand_palm_pos = data.palmpos
        self.hand_roll_pitch_yaw = data.ypr
        self.index_tip = data.index_tip
        self.thumb_tip = data.thumb_tip

    def send_leap_pos_ori(self, req):
        rospy.loginfo("send the leap_pos_ori of current Leap_Motion frame")
        if ((self.index_tip.x - self.thumb_tip.x)**2 + \
            	(self.index_tip.y - self.thumb_tip.y)**2 + \
            		(self.index_tip.z - self.thumb_tip.z)**2)< 0.03:
            gripper = -1.0
	else:
            gripper = 1.0
        # response
        return hand_pos_oriResponse(
				   hand_direction = Vector3(self.hand_direction.x,
                                                             self.hand_direction.y,
                                                             self.hand_direction.z),
                                    hand_normal   = Vector3(self.hand_normal.x,
                                                             self.hand_normal.y,
                                                             self.hand_normal.z),
                                    hand_palm_pos = Point(self.hand_palm_pos.x,
                                                          self.hand_palm_pos.y,
                                                          self.hand_palm_pos.z),
                                    hand_roll_pitch_yaw = Vector3(self.hand_roll_pitch_yaw.x,
                                                                  self.hand_roll_pitch_yaw.y,
                                                                  self.hand_roll_pitch_yaw.z),
                                    gripper=gripper
					)

def main():
    node_name = 'leap_pos_ori_server'
    lp_server = leap_pos_ori_server()
    rospy.init_node(node_name, anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main()

