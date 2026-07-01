#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import json
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_srvs.srv import Empty
import os
import random
class MoveBaseSquare():
    # 改动说明：本版仅调整等待/识别时间，不改导航点坐标；撞墙和速度主要由TEB与costmap参数解决。

    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
     
        # Create a list to hold the target quaternions (orientations)
        # 创建一个列表，保存目标的角度数据
        quaternions = list()
        # First define the corner orientations as Euler angles
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        #euler_angles = (0,pi/2, pi/2,-pi/2, -pi/2, pi/2,-pi/2,0,pi/4)
        euler_angles = (pi/2,pi, pi/2,-1.357, -pi/2, -0.684+pi/2,-1.238,0,-pi,0)
        # Then convert the angles to quaternions
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        
    
        #识别时间减小 停顿时间减小
        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(1.284, 2.160, 0), quaternions[0]))      #//C 1.484
        waypoints.append(Pose(Point(0.603, 2.620, 0), quaternions[1]))      #//A 0.675,2.393   0.65  0.703
        waypoints.append(Pose(Point(1.284, 3.000, 0), quaternions[2]))      # //B x 减小//1.35,2.899 B小一点1.40，3.171
        waypoints.append(Pose(Point(-1.075, 0.898, 0), quaternions[3]))     # //4(-0.830, 0.969, 0)
        waypoints.append(Pose(Point(-1.731, 1.296, 0), quaternions[4]))    # //3  -1.781 （-1.650, 1.232, 0)
        waypoints.append(Pose(Point(-1.080, 1.715, 0), quaternions[5]))    # //2  -1.140
        waypoints.append(Pose(Point(-1.734, 2.325, 0), quaternions[6]))    # //1  -1.834
        waypoints.append(Pose(Point(0.002, -0.056, 0), quaternions[7]))    #起点
        waypoints.append(Pose(Point(-0.496, 3.921, 0), quaternions[8]))  #答题区(识别板2) 靠近里侧（-0.704,3.963,0） （0）靠近内侧 y减小(-0.784,3.750,0.1)
        waypoints.append(Pose(Point(0.866, -0.002, 0), quaternions[9]))  #识别板1向前一点0.6 ()x:0.65 y:0.025

        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人

        #self.i = 0
        #self.j = 0
        self.count = 9  #状态
        self.windows_ABC = 0
        self.windows_A = 1
        self.windows_B = 1
        self.windows_C = 1
        self.windows_1234 = 3
        self.windows_count = 3

        # ================= 识别板1任务选择缓存 =================
        # scan_active=True 时才接收识别结果，避免运动中被新识别结果干扰。
        self.scan_active = False
        # 当前识别阶段累计到的候选任务，元素格式：{"code": "ABC", "window": 1}
        self.scan_candidates = []
        # 上一轮识别到的全部二维码集合，用于判断裁判软件是否刷新。
        self.last_seen_tasks = set()
        # 当前裁判画面中已经执行过的任务，防止同一批二维码反复跑。
        self.finished_tasks_for_scene = set()
        # 本圈锁定并正在执行的任务。完成送样后加入 finished_tasks_for_scene。
        self.current_action_key = None

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result,queue_size=10)     #订阅检测的药品和窗口数组
        self.cam_all_sub = rospy.Subscriber('/cam_all_qr', String, self.all_qr_callback, queue_size=10)  #订阅一帧内全部二维码
        self.ram_result = [0, 0, 1, 1, 2] # 11134
        rospy.sleep(1)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")
        # 初始化一个计数器，记录到达的顶点号
        
        while(not rospy.is_shutdown()):#如果ros系统没有关机的话
            #有限状态机
            if(self.count == 9):#从起点到识别区
                rospy.loginfo("从起点到识别区")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[9]     #self.i
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别区。。。。。。。。")
                    self.count = 10
                    # self.clear_costmaps_service()

                    # 开始收集识别板1结果。6秒内不直接执行，先累计多帧候选任务。
                    self.begin_scan_round()
                    rospy.sleep(6)     # 给6秒钟识别时间；若识别不稳定可改回10，若追求速度可继续降到4

                    # 识别结束后统一锁定一个任务，防止最后一帧把 ABC 覆盖成 AC。
                    if not self.finish_scan_round():
                        rospy.logwarn("本轮未锁定有效任务，留在识别区等待刷新/重新识别")
                        self.count = 9
                        rospy.sleep(2)
                        continue
                else:
                    rospy.logwarn("识别区导航失败")
                    self.count = 9
                    rospy.sleep(2)

            elif(self.count == 10):#从起点到配药区
                rospy.loginfo('10101010101010')
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                # goal.target_pose.pose = waypoints[10] #防倒车点
                #goal.target_pose.pose = waypoints[self.windows_ABC]

                if(self.windows_C == 1): #是否去c
                    goal.target_pose.pose = waypoints[0]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到c窗口中的")
                        self.count = 11
                        self.clear_costmaps_service()   #清除产生的偏移代价地图
                        rospy.sleep(1)
                        if ((self.windows_A == 0) and (self.windows_B == 0)):
                            
                            # os.system("play "+temp+C) 
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangC.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiC.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeC.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueC.wav')

                if(self.windows_A == 1):
                    goal.target_pose.pose = waypoints[1]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到a窗口中的")
                        self.count = 11
                        self.clear_costmaps_service()   #清除产生的偏移代价地图
                        rospy.sleep(1)
                        if ((self.windows_C == 0) and (self.windows_B == 0)):
                            #os.system("play "+temp+A)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangA.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiA.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeA.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueA.wav')
                        elif ((self.windows_C == 1) and (self.windows_B == 0)):
                            #os.system("play "+temp+CA)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAC.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAC.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAC.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAC.wav')

                        self.clear_costmaps_service()  

                if(self.windows_B == 1):
                    goal.target_pose.pose = waypoints[2]
                    if(self.move(goal) == True):
                        rospy.loginfo("取到b窗口中的")
                        self.count = 11
                        self.clear_costmaps_service()   #清除产生的偏移代价地图
                        rospy.sleep(1)
                        if ((self.windows_C == 1) and (self.windows_A == 1)):
                            #os.system("play "+temp+CAB)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangABC.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiABC.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeABC.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueABC.wav')
                            
                            
                        elif ((self.windows_C == 0) and (self.windows_A == 1)):
                            
                            #os.system("play "+temp+AB)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangAB.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiAB.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeAB.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueAB.wav')
                            
                        elif ((self.windows_C == 1) and (self.windows_A == 0)):
                            
                            #os.system("play "+temp+BC)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangBC.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiBC.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeBC.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueBC.wav')
                            
                        elif ((self.windows_C == 0) and (self.windows_A == 0)):
                            #os.system("play "+temp+B)
                            if(self.windows_1234 == 3): #4(激素检验窗口)
                                rospy.loginfo("血浆样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuejiangB.wav')
                            if(self.windows_1234 == 2): #3(免疫检测窗口)
                                rospy.loginfo("组织样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/zuzhiB.wav')
                            if(self.windows_1234 == 1): #2(体液窗口)
                                rospy.loginfo("唾液样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tuoyeB.wav')
                            if(self.windows_1234 == 0): #1(血常规窗口)
                                rospy.loginfo("静脉血样本") #后面加播报
                                os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jingmaixueB.wav')
                self.count = 11

            elif(self.count == 11):#从配药区到答题区
                rospy.loginfo("从配药区到答题区路上")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[8]     #self.i
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别板2。。。。。。。。")
                    self.count = 12
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.loginfo("化验区空闲") #后面加识别再改
                    #rospy.sleep(1)     #在答题区等5秒钟，后面可以加识别播报
                    # temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    # YX = "/yuyingwenjian/D.wav"#圆形药品
                    # SS = "/yuyingwenjian/E.wav"#双色胶囊
                    # YGY = "/yuyingwenjian/F.wav"#鱼肝油
                    # DS = "/yuyingwenjian/G.wav"#单色胶囊
                    # TY = "/yuyingwenjian/H.wav"#椭圆
                    # yaop2 = [YX, DS, YX, YGY,DS,TY,YX,SS,YGY,DS,TY,YX,SS,YGY,DS,TY,YX,SS,TY,DS,YGY,SS,DS]
                    # if self.i < 23:
                    #      os.system('play '+temp+yaop2[self.i])
                    # else:
                    #      yaop = [YX,YGY,TY,SS]
                    #      yaop1 = random.choice(yaop)
                    #      os.system('play '+temp+yaop1)
                        
                    # yaop2 = [YX,DS,YX,YGY]
                    # if self.i < 4:
                    #     os.system('play '+temp+yaop2[self.i])
                    # else:
                    #     yaop = [YX,YGY,TY,SS]
                    #     yaop1 = random.choice(yaop)
                    #     os.system('play '+temp+yaop1)


            elif(self.count == 12):#从答题区到数字区
                rospy.loginfo("从答题区到数字区路上")
                goal = MoveBaseGoal() 
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[(6 - self.windows_1234)]
                if(self.move(goal) == True):
                    rospy.loginfo("到达数字区。。。。。。。。")                    
                    self.count = 9 
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.sleep(1)     #在数字区等待1秒钟，后面可以加播报送药完成
                    # self.i = self.i + 1
                    #播报到达位置
                    # temp = '/home/EPRobot/robot_ws/src/pharmacy_pkg/'
                    # A4 = '/yuyingwenjian/jisu.wav' #要改语音播报
                    # A3 = '/yuyingwenjian/mianyi.wav'
                    # A2 = '/yuyingwenjian/tiye.wav'
                    # A1 = '/yuyingwenjian/xiechanggui.wav'
                    # C1 = '/yuyingwenjian/y11.wav'
                    # C2 = '/yuyingwenjian/y22.wav'
                    # C3 = '/yuyingwenjian/y33.wav'
                    if self.windows_1234 == 3: #4
                        rospy.loginfo("到达激素检验窗口")
                        if self.windows_count == 3:
                            rospy.loginfo("样本数为3")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu3.wav')
                        elif self.windows_count == 2:
                            rospy.loginfo("样本数为2")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu2.wav')
                        elif self.windows_count == 1:
                            rospy.loginfo("样本数为1")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/jisu1.wav')
                        # os.system('play '+temp+A4)
                    if self.windows_1234 == 2: #3
                        rospy.loginfo("到达免疫检验窗口")
                        if self.windows_count == 3:
                            rospy.loginfo("样本数为3")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi3.wav')
                        elif self.windows_count == 2:
                            rospy.loginfo("样本数为2")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi2.wav')
                        elif self.windows_count == 1:
                            rospy.loginfo("样本数为1")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/mianyi1.wav')
                        # os.system("play "+temp+A3)
                    if self.windows_1234 == 1: #2
                        rospy.loginfo("到达体液检验窗口")
                        if self.windows_count == 3:
                            rospy.loginfo("样本数为3")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye3.wav')
                        elif self.windows_count == 2:
                            rospy.loginfo("样本数为2")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye2.wav')
                        elif self.windows_count == 1:
                            rospy.loginfo("样本数为1")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/tiye1.wav')
                        # os.system("play "+temp+A2)
                    if self.windows_1234 == 0: #1
                        rospy.loginfo("到达血常规检验窗口")
                        if self.windows_count == 3:
                            rospy.loginfo("样本数为3")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui3.wav')
                        elif self.windows_count == 2:
                            rospy.loginfo("样本数为2")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui2.wav')
                        elif self.windows_count == 1:
                            rospy.loginfo("样本数为1")
                            os.system("play "+'/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian/xuechanggui1.wav')

                    self.mark_current_task_finished()

    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)
            # Allow 1 minute to get there
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(90))  # 单个导航目标最长等待90秒；地图较绕或倒车较多时比60秒更稳
            # If we don't get there in time, abort the goal
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
            return False        

    def begin_scan_round(self):
        """
        开始一次识别板1扫描。
        """
        self.scan_candidates = []
        self.scan_active = True
        rospy.logwarn("开始收集识别板1全部二维码结果")

    def finish_scan_round(self):
        """
        结束识别并锁定最终任务。
        排序规则：
        1. 样本数多优先：ABC > AB/AC/BC > A/B/C。
        2. 若本轮二维码集合与上一轮完全一致，不再执行已经做过的任务。
        3. 样本数相同则选择识别次数更多的任务。
        4. 仍相同则选择窗口编号靠前的任务。
        """
        self.scan_active = False

        if len(self.scan_candidates) == 0:
            rospy.logwarn("本轮没有收集到任何有效二维码")
            return False

        counter = {}
        for task in self.scan_candidates:
            key = (task["code"], task["window"])
            counter[key] = counter.get(key, 0) + 1

        seen_tasks = set(counter.keys())
        rospy.logwarn("本轮识别到的任务集合: %s", str(seen_tasks))
        rospy.logwarn("本轮识别计数: %s", str(counter))

        # 如果裁判软件画面已刷新，清空上一场景已完成任务。
        if seen_tasks != self.last_seen_tasks:
            rospy.logwarn("检测到二维码集合变化，认为裁判软件已刷新")
            self.finished_tasks_for_scene = set()

        candidate_keys = list(seen_tasks - self.finished_tasks_for_scene)

        if len(candidate_keys) == 0:
            rospy.logwarn("本轮二维码集合与上一轮相同，且其中任务均已执行，禁止重复运行")
            self.last_seen_tasks = seen_tasks
            return False

        def task_score(key):
            code, window = key
            sample_count = len(code)
            appear_count = counter.get(key, 0)
            # max() 会优先比较样本数，再比较出现次数，最后窗口编号小者优先。
            return (sample_count, appear_count, -window)

        best_key = max(candidate_keys, key=task_score)
        best_code, best_window = best_key

        self.apply_locked_task(best_code, best_window)
        self.current_action_key = best_key
        self.last_seen_tasks = seen_tasks

        rospy.logwarn(
            "最终锁定任务: code=%s, pos=%d, score=%s",
            best_code,
            best_window + 1,
            str(task_score(best_key))
        )
        return True

    def all_qr_callback(self, msg):
        """
        接收 F1_detect_code0719.py 发布的一帧全部二维码。
        数据格式：
        {"all_data": [{"window":0, "pos":1, "code":"AC", "sample_count":2}, ...]}
        """
        if not self.scan_active:
            return

        try:
            payload = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("解析 /cam_all_qr 失败: %s", str(e))
            return

        all_data = payload.get("all_data", [])

        for item in all_data:
            try:
                code = str(item.get("code", ""))
                window = int(item.get("window", -1))
            except Exception:
                continue

            if code not in ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']:
                continue
            if window < 0 or window > 3:
                continue

            self.scan_candidates.append({"code": code, "window": window})

        if len(all_data) > 0:
            rospy.logwarn("收到全部二维码候选: %s", str(all_data))

    def detect_result(self, msg):
        """
        兼容旧版 /cam_return。注意：这里不再直接覆盖 windows_A/B/C/1234，
        只在 scan_active=True 时作为候选任务收集。
        """
        if not self.scan_active:
            return

        data = list(msg.data)
        if len(data) < 5:
            return

        task = self.task_from_msg_data(data)
        if task is None:
            return

        self.scan_candidates.append(task)
        rospy.logwarn("收到兼容候选任务: %s", str(task))

    def task_from_msg_data(self, data):
        """
        将旧版 /cam_return 数组转换成候选任务。
        data: [是否去C, 是否去A, 是否去B, 样本数, 化验窗口idx, 错误窗口]
        """
        if len(data) < 5:
            return None

        c_flag = int(data[0])
        a_flag = int(data[1])
        b_flag = int(data[2])
        window = int(data[4])

        code = ""
        if a_flag == 1:
            code += "A"
        if b_flag == 1:
            code += "B"
        if c_flag == 1:
            code += "C"

        if code not in ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']:
            return None
        if window < 0 or window > 3:
            return None

        return {"code": code, "window": window}

    def apply_locked_task(self, code, window):
        """
        将最终锁定任务写回原程序使用的变量。
        后面的取样、播报、送样流程不需要大改。
        """
        self.windows_A = 1 if 'A' in code else 0
        self.windows_B = 1 if 'B' in code else 0
        self.windows_C = 1 if 'C' in code else 0
        self.windows_count = len(code)
        self.windows_1234 = window

        self.ram_result = [
            self.windows_C,
            self.windows_A,
            self.windows_B,
            self.windows_count,
            self.windows_1234,
            0
        ]
        rospy.logwarn("锁定后的 self.ram_result: %s", str(self.ram_result))

    def mark_current_task_finished(self):
        """
        小车完成送样后，记录当前任务已经执行过。
        若裁判软件下一圈还没刷新，则不会重复执行该任务。
        """
        if self.current_action_key is not None:
            self.finished_tasks_for_scene.add(self.current_action_key)
            rospy.logwarn("记录已完成任务: %s", str(self.current_action_key))
            self.current_action_key = None

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

  
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")