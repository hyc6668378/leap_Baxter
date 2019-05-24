#!/usr/bin/env python
#coding=utf-8

import baxter_interface
import rospy
import move_baxter
import tf
import math
import threading
from suelab_bax_arm.srv import *   # 导入service  
from sensor_msgs import point_cloud2  # 点云msg 解析处理
from geometry_msgs.msg import(
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import numpy as np
import random
from cv_bridge import CvBridge
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

# --------------- hyper parameters -------------
POINTS_NUM = 4096   # 点云个数 2的12次方
IMAGE_HIGHT = 256
IMAGE_WIDTH = 256
MAX_DISS_TO_BOX = 0.8  # 手到box 所允许的最远距离 的平方
INIT_BOX_Z = 0.775

def pcl_client():    #  cloud_points.shape = (POINTS_NUM, 3)
    # 等待有可用的服务 "leapmotion/pos_ori_server"
    rospy.wait_for_service("/suelab_bax_arm/pcl_srv")
    try:
        pcl_client_obj = rospy.ServiceProxy("/suelab_bax_arm/pcl_srv", pcl_srv)
        # 向server端发送请求
        response = pcl_client_obj.call()
        cloud_points = list(point_cloud2.read_points(response.pcl2, skip_nans=True, field_names = ("x", "y", "z")))
        return np.array(random.sample(cloud_points , POINTS_NUM))   # cloud_points.shape = (POINTS_NUM, 3)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

def image_client():   #  iamge.shape = (IMAGE_WIDTH,IMAGE_HIGHT,3)
    # 等待有可用的服务 "leapmotion/pos_ori_server"
    rospy.wait_for_service("/suelab_bax_arm/image")
    try:
        image_client_obj = rospy.ServiceProxy("/suelab_bax_arm/image", image)
        # 向server端发送请求
        image_response = image_client_obj.call()
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(image_response.image, "bgr8") # obs图像 np.array 数组 (IMAGE_WIDTH,IMAGE_HIGHT,3)

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

def full_state_client():   #  full_state.shape = (14,)
    rospy.wait_for_service("/suelab_bax_arm/full_state")
    try:
        full_state_client_obj = rospy.ServiceProxy("/suelab_bax_arm/full_state", full_state)
        # 向server端发送请求
        full_state_response = full_state_client_obj.call()
        return np.array(full_state_response.full_state) 

    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

class Baxter_Env():
    """
    Class of Baxter RL environment
    """
    
    def __init__(self):
        """
            Initializes the Baxter_Env.
        """
        rospy.init_node('Baxter_Env')
        rs = baxter_interface.RobotEnable() 
        rs.enable()
        #记录 连续ik无解次数
        self.ik_no_solution = 0
        # arm
        self.left = baxter_interface.Limb('left')
        self.right = baxter_interface.Limb('right')
        # gripper
        self.left_gripper = baxter_interface.Gripper('left')
        self.right_gripper = baxter_interface.Gripper('right')

        # end_effector
        self.leftPose = self.left.endpoint_pose()
        self.rightPose = self.right.endpoint_pose()
        
        # ratio of maximum joint speed for exection default=0.3; range=[0.0-1.0]
        self.left.set_joint_position_speed(1.0)
        self.right.set_joint_position_speed(1.0)

        # Set the timeout in seconds for the joint controller 
        self.left.set_command_timeout(timeout=0.1)
        self.right.set_command_timeout(timeout=0.1)

        # 爪子一开始都打开
        self.left_gripper.open()
        self.right_gripper.open()
        # print "\n"
        # print "move baxter to initial state:"
        # 左右手各开一个线程,并发运动到初始位置
        self.left.move_to_joint_positions({'left_w0': 0.6699952259595108,
                                                        'left_w1': 1.030009435085784,
                                                        'left_w2': -0.4999997247485215,
                                                        'left_e0': -1.189968899785275,
                                                        'left_e1': 1.9400238130755056,
                                                        'left_s0': -0.08000397926829805,
                                                        'left_s1': -0.9999781166910306})
        # self.left_move_init_thread = threading.Thread(target=self.left.move_to_joint_positions,
        #                                          name='left_move_init_thread',
        #                                          args=)
        # self.right_move_init_thread = threading.Thread(target=move_baxter.moveAbs,
        #                                          name='right_move_init_thread',
        #                                          args=('right',
        #                                         Point(x=0.356982770038,
        #                                               y=-0.852598021641,
        #                                               z=0.0288609422173),
        #                                         Quaternion(x=0.367048116303,
        #                                                    y=0.885911751787,
        #                                                    z=-0.108908281936,
        #                                                    w=0.261868353356)))
        # # start initial Threads
        # self.left_move_init_thread.start()
        # self.right_move_init_thread.start()
        # # join initial Threads
        # self.left_move_init_thread.join()
        # self.right_move_init_thread.join()
        
        # 加载桌子 和 box
        load_gazebo_models()
        
        print ("-----------------------------------")
        print ("the Baxter_Env hsk been initalized.")
        print "\n"

    def reset(self):
        """
        Environment reset called at the beginning of an episode.
        return : pcl and rgb image
        """
        # 删除 baxter, 桌子 和 box
        delete_gazebo_models()

        self.ik_no_solution = 0
        
        # print "\n"
        # print "move baxter to initial state:"
        self.left.move_to_joint_positions({'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306})
        # 左右手各开一个线程,并发运动到初始位置
        # self.left_move_init_thread = threading.Thread(target=move_baxter.moveAbs,
        #                                          name='left_move_init_thread',
        #                                          args=('left',
        #                                         Point(x=0.357579481614,
        #                                               y=0.35281417433,
        #                                               z=0.0488352386502,),
        #                                         Quaternion(x=-0.366894936773,
        #                                                    y=0.885980397775,
        #                                                    z=0.108155782462,
        #                                                    w=0.262162481772)))
        # self.right_move_init_thread = threading.Thread(target=move_baxter.moveAbs,
        #                                          name='right_move_init_thread',
        #                                          args=('right',
        #                                         Point(x=0.656982770038,
        #                                               y=-0.852598021641,
        #                                               z=0.0388609422173),
        #                                         Quaternion(x=0.367048116303,
        #                                                    y=0.885911751787,
        #                                                    z=-0.108908281936,
        #                                                    w=0.261868353356)))
        # # start initial Threads
        # self.left_move_init_thread.start()
        # self.right_move_init_thread.start()
        # # join initial Threads
        # self.left_move_init_thread.join()
        # self.right_move_init_thread.join()
        
        # 加载桌子 和 box
        load_gazebo_models()

        # 返回观察结果(图像,点云)
        return image_client(), pcl_client()

    def get_observation(self):
        # rgb,  pointCloud
        return image_client(), pcl_client()

    def step(self, left_action=None): # right_action=None 右手占时没有
        """
                参数             数据类型          shape
            ------------      ------------      --------
            left_action         np.array          (5,)
            ------------------------------------------------
            return :  
                        observe: 返回执行该step后的观测(image, pcl)
                        reward:  返回执行该step后的奖励
                        done:    任务终止判断   0 or 1
                        info:    有用的消息字典  占时只有是否成功{'grasp_success':}
        
        """
        
        self.current_leftPose = self.left.endpoint_pose()
        #self.current_rightPose = self.right.endpoint_pose()
        self.left_gripper_action = left_action[4]
        # self.left_gripper_action = right_action[4]

        '''
            action 从 np.array 变成字典
            左手action {'dx':,'dy':,'dz':,'d_theta':}
        '''
        left_action = {'dx': left_action[0],'dy': left_action[1],'dz': left_action[2], 'd_theta':left_action[3]}
        left_current_ori_euler = tf.transformations.euler_from_quaternion((self.current_leftPose['orientation'].x,
                                                              self.current_leftPose['orientation'].y,
                                                              self.current_leftPose['orientation'].z,
                                                              self.current_leftPose['orientation'].w))
        left_action_ori_quaternion = tf.transformations.quaternion_from_euler(
                                                             3.1143499368018905, 
                                                             0.03892959135029865,
                                                             left_current_ori_euler[2] + left_action['d_theta'])

        self.check_move = move_baxter.moveAbs('left',
                                        Point(self.current_leftPose['position'].x + left_action['dx'], 
                                              self.current_leftPose['position'].y + left_action['dy'],
                                              self.current_leftPose['position'].z + left_action['dz']),
                                        Quaternion(left_action_ori_quaternion[0],
                                                   left_action_ori_quaternion[1],
                                                   left_action_ori_quaternion[2],
                                                   left_action_ori_quaternion[3])) 
        
        """ 
            self.check_move 用来检测action是否合理
            如果返回0 表示正常执行action.
            如果返回1 表示ik无解
            该变量将会在 'self.reward()' 中访问. 对不合理action将实施惩罚.
        """
        if self.check_move != 0:
            self.ik_no_solution += 1
        else:
            self.ik_no_solution = 0

        # 先动胳膊,再动爪子
        if self.left_gripper_action > 0:
            self.left_gripper.open()
        else:
            self.left_gripper.close()

        # # 左右手各开一个线程,并发执行action
        # self.left_move_action_thread = threading.Thread(target=move_baxter.moveAbs,
        #                                          name='left_move_action_thread',
        #                                          args=('left',
        #                                         Point(self.current_leftPose['position'].x + left_action['dx'], 
        #                                               self.current_leftPose['position'].y + left_action['dy'],
        #                                               self.current_leftPose['position'].z + left_action['dz']),
        #                                         Quaternion(self.current_leftPose['orientation'].x + left_action['dox'],
        #                                                    self.current_leftPose['orientation'].y + left_action['doy'],
        #                                                    self.current_leftPose['orientation'].z + left_action['doz'],
        #                                                    self.current_leftPose['orientation'].w + left_action['dow'])
        #                                          ))
        # self.right_move_action_thread = threading.Thread(target=move_baxter.moveAbs,
        #                                          name='right_move_action_thread',
        #                                          args=('right',
        #                                          Point(self.current_rightPose['position'].x + right_action['dx'],
        #                                                self.current_rightPose['position'].y + right_action['dy'],
        #                                                self.current_rightPose['position'].z + right_action['dz']),
        #                                          Quaternion(self.current_rightPose['orientation'].x + right_action['dox'],
        #                                                     self.current_rightPose['orientation'].y + right_action['doy'],
        #                                                     self.current_rightPose['orientation'].z + right_action['doz'],
        #                                                     self.current_rightPose['orientation'].w + right_action['dow'])
        #                                         ))

        # # start arm_move_action Threads
        # self.left_move_action_thread.start()
        # self.right_move_action_thread.start()
        # # join arm_move_action Threads
        # self.left_move_action_thread.join()
        # self.right_move_action_thread.join()

        # print "move finished."
        # print "\n"

        if self.reward() == 50:
            info = {'grasp_success': 1}
        else:
            info = {'grasp_success': 0}

        return image_client(), pcl_client(), self.reward(), self.termination(), info

    def termination(self):
        """
            0:  此回合未终止
            1:  回合终止
        """
        fs = full_state_client()
        fu_state = {'left.x': fs[0],
                    'left.y': fs[1],
                    'left.z': fs[2],
                    'box.x': fs[-7],
                    'box.y': fs[-6],
                    'box.z': fs[-5]}
        left_diss = (fu_state['left.x'] - fu_state['box.x'])**2 + (fu_state['left.y'] - fu_state['box.y'])**2 + (fu_state['left.z'] - fu_state['box.z'])**2
        OUT_OF_FIELD = (fu_state['left.x']< 0.4) or (fu_state['left.y']> 0.3) or (fu_state['left.z']> 1.6)

        if self.ik_no_solution > 8:
            RESET_ENV = True
            self.ik_no_solution = 0 # 清零
        else:
            RESET_ENV = False
        # 1 box 掉地上  2, tesk 成功  3,左手距离桌子中心太远
        return (fu_state['box.z'] < INIT_BOX_Z -0.2) or (fu_state['box.z'] > INIT_BOX_Z + 0.2) or OUT_OF_FIELD or (left_diss > MAX_DISS_TO_BOX) or RESET_ENV

    def reward(self):
        fs = full_state_client()
        fu_state = {'left.x': fs[0],
                    'left.y': fs[1],
                    'left.z': fs[2],
                    'box.x': fs[-7],
                    'box.y': fs[-6],
                    'box.z': fs[-5]}
        left_diss = (fu_state['left.x'] - fu_state['box.x'])**2 + (fu_state['left.y'] - fu_state['box.y'])**2 + (fu_state['left.z'] - fu_state['box.z'])**2

        OUT_OF_FIELD = (fu_state['left.x']< 0.4) or (fu_state['left.y']> 0.3) or (fu_state['left.z']> 1.6)

        if  fu_state['box.z'] > INIT_BOX_Z + 0.2:  # 桌子上的box 世界坐标系下 z = 0.775  
            return 50
        elif (self.check_move != 0) or OUT_OF_FIELD or (left_diss >  MAX_DISS_TO_BOX):
            # ik 无解, 左手出界(照相机看不到手), 左手距离桌子中心太远, 
            return -5
        else:
            return 0

    def get_full_state(self):
        # 左手位姿 + 右手位姿 + 小方格位姿 = 7*3 = 21维
        return full_state_client()
