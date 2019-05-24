#!/usr/bin/env python
# coding:utf-8

import rospy
from leap_motion.srv import *   #  hand_pos_ori.srv

def leap_pos_ori_client_srv():
    # 等待有可用的服务 "leapmotion/pos_ori_server"
    rospy.wait_for_service("leapmotion/pos_ori_server")
    try:
        leap_pos_ori_client = rospy.ServiceProxy("leapmotion/pos_ori_server", hand_pos_ori)

        # 向server端发送请求
        response = leap_pos_ori_client.call()

        return response
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

def main():
    rospy.init_node('leap_pos_ori_client', anonymous=True)
    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        leap_pos_ori_client_srv()
        r.sleep()

# 如果单独运行此文件，则将上面函数client_srv()作为主函数运行
if __name__=="__main__":
    main()
