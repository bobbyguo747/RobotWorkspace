#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
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
        euler_angles = (pi/2,pi/2, pi/2,-pi/2, -pi/2, -pi/2,-pi/2,0,-pi,0)
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
        waypoints.append(Pose(Point(1.395, 2.075, 0), quaternions[0]))      # //C
        waypoints.append(Pose(Point(0.550, 2.468, 0), quaternions[1]))      #//A 0.675,2.393   0.65
        waypoints.append(Pose(Point(1.374, 3.045, 0), quaternions[2]))      # //B x 减小//1.35,2.899 B小一点1.40
        waypoints.append(Pose(Point(-0.929, 0.853, 0), quaternions[3]))     # //4(-0.830, 0.969, 0)
        waypoints.append(Pose(Point(-1.862, 1.298, 0), quaternions[4]))    # //3 （-1.650, 1.232, 0)
        waypoints.append(Pose(Point(-1.004, 1.800, 0), quaternions[5]))    # //2
        waypoints.append(Pose(Point(-1.784, 2.345, 0), quaternions[6]))    # //1
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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.cam_sub = rospy.Subscriber('/cam_return', Int32MultiArray, self.detect_result,queue_size=10)     #订阅检测的药品和窗口数组
        # 默认备用识别结果：即使本轮二维码完全没识别到，也按这个结果继续跑，不进入死循环。
        # 格式：[是否去C, 是否去A, 是否去B, 样本数, 目标窗口编号, 错误区域]
        # 这里保留你原来的默认逻辑：[0,0,1,1,2] = 只去B，最后去3号窗口。
        self.fallback_result = [0, 0, 1, 1, 2, 0] # 11134
        self.ram_result = list(self.fallback_result)
        self.error_count = 0
        self.got_new_detection = False
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

                    # 每轮到达识别区后，先重置识别状态。
                    # 重点：不等待到死。如果6秒内没有新识别结果，就使用 fallback_result 继续执行。
                    self.got_new_detection = False
                    self.apply_detection_result(self.fallback_result, source="默认备用识别结果")
                    # self.clear_costmaps_service()
                    rospy.sleep(6)     # 给6秒钟识别时间；若识别不稳定可改回10，若追求速度可继续降到4

                    if not self.got_new_detection:
                        rospy.logwarn("本轮未收到新的二维码识别结果，使用默认备用结果继续跑: %s", self.ram_result)
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

    def apply_detection_result(self, result, source="识别结果"):
        """
        统一应用识别结果。
        result 格式兼容 5位或6位：
            [是否去C, 是否去A, 是否去B, 样本数, 目标窗口编号]
            [是否去C, 是否去A, 是否去B, 样本数, 目标窗口编号, 错误区域]
        即使结果异常，也不会让程序卡死；会回退到 fallback_result 继续跑。
        """
        try:
            data = list(result)
        except Exception as e:
            rospy.logwarn("%s 无法转换为列表: %s，使用默认备用结果", source, e)
            data = list(self.fallback_result)

        if len(data) < 5:
            rospy.logwarn("%s 长度不足: %s，使用默认备用结果", source, data)
            data = list(self.fallback_result)

        # 兼容旧版5位数组，自动补第6位错误区域
        if len(data) == 5:
            data = data + [0]

        # 基本合法性保护：非法值不停车，直接回退到默认备用结果
        valid = True
        if data[0] not in [0, 1] or data[1] not in [0, 1] or data[2] not in [0, 1]:
            valid = False
        if data[3] not in [1, 2, 3]:
            valid = False
        if data[4] not in [0, 1, 2, 3]:
            valid = False

        if not valid:
            rospy.logwarn("%s 数值非法: %s，使用默认备用结果继续跑", source, data)
            data = list(self.fallback_result)

        self.ram_result = data
        self.windows_C = self.ram_result[0]      # 0=不去，1=去
        self.windows_A = self.ram_result[1]
        self.windows_B = self.ram_result[2]
        self.windows_count = self.ram_result[3]  # 样品数
        self.windows_1234 = self.ram_result[4]   # 3=4口，2=3口，1=2口，0=1口
        self.error_count = self.ram_result[5]

        if self.error_count != 0:
            rospy.logwarn("识别结果提示第 %d 个二维码区域存在异常内容，但仍按主二维码结果继续跑", self.error_count)

        # ================== 终端输出当前采用的识别数组 ==================
        # 这里会在运行 F1_yaofang0719.py 的 Ubuntu 终端中显示导航端实际采用的数组。
        # 如果本轮没识别到二维码，也会显示“默认备用识别结果”的数组，保证你知道小车按什么结果继续跑。
        print("\n================ 导航端当前采用数组 ================")
        print("来源: %s" % source)
        print("当前数组 = %s" % self.ram_result)
        print("数组含义: [是否去C, 是否去A, 是否去B, 样本数, 目标窗口编号, 错误区域编号]")
        print("解析结果: C=%d, A=%d, B=%d, 样本数=%d, 目标窗口编号=%d, 错误区域=%d" % (
            self.windows_C, self.windows_A, self.windows_B, self.windows_count, self.windows_1234, self.error_count
        ))
        print("================================================\n")
        rospy.logwarn("已应用%s: %s", source, self.ram_result)

    def detect_result(self, msg):
        if self.count == 10:   # 只有识别阶段才接收，其他时间不接受识别，以防干扰
            print("\n================ 导航端收到 /cam_return ================")
            print("收到原始数组: %s" % list(msg.data))
            print("=======================================================\n")
            rospy.logwarn("导航端收到 /cam_return 原始数组: %s", list(msg.data))
            self.got_new_detection = True
            self.apply_detection_result(msg.data, source="新二维码识别结果")

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