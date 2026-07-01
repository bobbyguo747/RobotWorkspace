#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
F1_yaofang0719.py

0719 导航主控拆分版：
1. 只负责导航、任务选择、语音播放、发布当前任务状态。
2. 不再连接裁判 TCP，不再 socket 发送，避免通信阻塞主任务。
3. 到达识别板1后等待 /cam_return；识别到合法结果立即执行。
4. 超时仍未识别到时启用默认任务 ABC4。
5. 任务锁定后，后续 /cam_return 不再覆盖本轮路线。

/cam_return 数据格式：
  [C, A, B, sample_count, target_area, error_count]
  target_area: 0->1号区, 1->2号区, 2->3号区, 3->4号区
"""

import json
import os
import time
from math import pi

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']

SCRIPT_DIR = '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts'
AUDIO_DIR = '/home/EPRobot/robot_ws/src/pharmacy_pkg/yuyingwenjian'


class MoveBaseSquare(object):
    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.referee_task_pub = rospy.Publisher(
            '/referee_task', String, queue_size=10, latch=True
        )
        self.cam_sub = rospy.Subscriber(
            '/cam_return', Int32MultiArray, self.detect_result,
            queue_size=20, tcp_nodelay=True
        )

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for move_base action server...')
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy(
            '/move_base/clear_costmaps', Empty
        )

        self.waypoints = self.build_waypoints()

        # ================== 任务默认值与等待机制 ==================
        # 默认 ABC4：C=1, A=1, B=1, count=3, target_area=3(4号区)
        self.DEFAULT_TASK = [1, 1, 1, 3, 3, 0]
        self.CV1_WAIT_TIMEOUT = float(rospy.get_param('~cv1_wait_timeout', 5.0))
        # 超时后先进入“默认待提交”状态，短暂允许迟到的视觉结果覆盖默认值。
        # 这样保留默认 ABC4，但能解决实测中 /cam_return 晚到 1~2 秒的问题。
        self.CV1_DEFAULT_COMMIT_DELAY = float(rospy.get_param('~cv1_default_commit_delay', 3.0))
        self.CV2_WAIT_TIME = float(rospy.get_param('~cv2_wait_time', 2.5))
        # 前往识别板1途中如果已经识别到 /cam_return，先缓存；
        # 到达识别点后优先使用缓存，避免停车后反而看不到二维码导致默认 ABC4。
        self.CV1_PRECACHE_MAX_AGE = float(rospy.get_param('~cv1_precache_max_age', 12.0))

        self.count = 9
        self.windows_C = 1
        self.windows_A = 1
        self.windows_B = 1
        self.windows_count = 3
        self.windows_1234 = 3
        self.error_count = 0
        self.ram_result = list(self.DEFAULT_TASK)

        self.has_valid_task = False
        self.task_locked = False
        self.default_task_used = False
        self.default_pending = False
        self.default_pending_start_time = 0.0
        self.cv1_wait_start_time = 0.0
        self.current_cv1 = ''
        self.current_cv2 = ''

        # ================== CV1 预识别缓存 ==================
        # count == 9 前往识别板1途中，如果视觉节点已经发布有效结果，先缓存。
        # count == 10 到达识别板1后，优先使用缓存，避免错过最佳识别视角。
        self.latest_cv1_task = None
        self.latest_cv1_task_time = 0.0
        self.latest_cv1_task_cv = ''

        self.publish_task('default', 'idle', 0, '主控启动')
        rospy.logwarn('Starting navigation 0719 split version...')
        self.run_loop()

    def build_waypoints(self):
        quaternions = []
        euler_angles = (
            pi / 2, pi, pi / 2, -1.357, -pi / 2,
            -0.684 + pi / 2, -1.238, 0, -pi, 0
        )
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            quaternions.append(Quaternion(*q_angle))

        waypoints = []
        waypoints.append(Pose(Point(1.284, 2.160, 0), quaternions[0]))   # 0 C
        waypoints.append(Pose(Point(0.603, 2.620, 0), quaternions[1]))   # 1 A
        waypoints.append(Pose(Point(1.284, 3.000, 0), quaternions[2]))   # 2 B
        waypoints.append(Pose(Point(-1.075, 0.898, 0), quaternions[3]))  # 3 4号区
        waypoints.append(Pose(Point(-1.731, 1.296, 0), quaternions[4]))  # 4 3号区
        waypoints.append(Pose(Point(-1.080, 1.715, 0), quaternions[5]))  # 5 2号区
        waypoints.append(Pose(Point(-1.734, 2.325, 0), quaternions[6]))  # 6 1号区
        waypoints.append(Pose(Point(0.002, -0.056, 0), quaternions[7]))  # 7 起点
        waypoints.append(Pose(Point(-0.496, 3.921, 0), quaternions[8]))  # 8 识别板2/答题区
        waypoints.append(Pose(Point(0.866, -0.002, 0), quaternions[9]))  # 9 识别板1
        return waypoints

    def publish_task(self, task, phase, task_id=0, info=''):
        data = {
            'task': str(task),
            'phase': str(phase),
            'task_id': int(task_id),
            'info': str(info)
        }
        if self.current_cv1 in VALID_CV:
            data['CV1'] = self.current_cv1
        if self.current_cv2 in VALID_CV:
            data['CV2'] = self.current_cv2
        msg = json.dumps(data, ensure_ascii=True)
        self.referee_task_pub.publish(msg)
        rospy.loginfo('[裁判状态] %s' % msg)

    def build_cv1_from_windows(self):
        text = ''
        if self.windows_A == 1:
            text += 'A'
        if self.windows_B == 1:
            text += 'B'
        if self.windows_C == 1:
            text += 'C'
        if text in VALID_CV:
            return text
        return ''

    def build_cv1_from_task_data(self, data):
        """
        根据 /cam_return 的 [C, A, B, count, target, error] 反推 CV1 字符串。
        注意输出顺序按裁判软件常用显示：A -> B -> C。
        """
        if len(data) < 5:
            return ''
        try:
            c = int(data[0])
            a = int(data[1])
            b = int(data[2])
        except Exception:
            return ''

        text = ''
        if a == 1:
            text += 'A'
        if b == 1:
            text += 'B'
        if c == 1:
            text += 'C'
        if text in VALID_CV:
            return text
        return ''

    def is_valid_task_msg(self, data):
        if len(data) < 5:
            return False
        try:
            c = int(data[0])
            a = int(data[1])
            b = int(data[2])
            sample_count = int(data[3])
            target_area = int(data[4])
        except Exception:
            return False

        if c not in [0, 1] or a not in [0, 1] or b not in [0, 1]:
            return False
        if sample_count not in [1, 2, 3]:
            return False
        if target_area not in [0, 1, 2, 3]:
            return False
        if c + a + b != sample_count:
            return False
        return True

    def apply_task_result(self, data, source='vision'):
        while len(data) < 6:
            data.append(0)

        self.ram_result = list(data)
        self.windows_C = int(data[0])
        self.windows_A = int(data[1])
        self.windows_B = int(data[2])
        self.windows_count = int(data[3])
        self.windows_1234 = int(data[4])
        self.error_count = int(data[5])

        self.has_valid_task = True
        self.default_task_used = (source not in ['vision', 'vision_precache'])
        if source in ['vision', 'vision_precache']:
            self.default_pending = False

        cv1_backup = self.build_cv1_from_windows()
        if cv1_backup in VALID_CV:
            self.current_cv1 = cv1_backup

        rospy.logwarn(
            '[CV1] 任务写入成功 source=%s data=%s CV1=%s target=%d' %
            (source, str(self.ram_result), self.current_cv1,
             self.windows_1234 + 1)
        )
        self.publish_task(
            self.current_cv1 if self.current_cv1 else 'default',
            'cv1_done', 1,
            'CV1任务已确认 source=%s target=%d' %
            (source, self.windows_1234 + 1)
        )

    def detect_result(self, msg):
        """
        接收二维码识别节点发布的 /cam_return。

        关键逻辑：
        1. count == 9 前往识别板1途中，如果识别到了，先缓存；
        2. count == 10 到达识别板1等待阶段，如果识别到了，立即使用；
        3. 任务正式锁定后，不再允许后续二维码覆盖本轮路线。
        """
        data = list(msg.data)

        if not self.is_valid_task_msg(data):
            rospy.logwarn('[CV1] 非法 /cam_return，忽略: %s' % str(data))
            return

        if self.task_locked:
            rospy.loginfo('[CV1] 任务已锁定，忽略后续 /cam_return')
            return

        cv1_text = self.build_cv1_from_task_data(data)

        # count == 9：前往识别板1途中。实测中这个阶段反而更容易看到二维码，先缓存。
        if self.count == 9:
            self.latest_cv1_task = list(data)
            self.latest_cv1_task_time = time.time()
            self.latest_cv1_task_cv = cv1_text
            rospy.logwarn(
                '[CV1] 前往识别板途中缓存 /cam_return: %s CV1=%s target=%d' %
                (str(data), cv1_text, int(data[4]) + 1)
            )
            return

        # count == 10：停车等待识别阶段。收到合法结果就立即使用。
        if self.count == 10:
            self.latest_cv1_task = list(data)
            self.latest_cv1_task_time = time.time()
            self.latest_cv1_task_cv = cv1_text
            rospy.logwarn(
                '[CV1] 等待阶段收到 /cam_return: %s CV1=%s target=%d' %
                (str(data), cv1_text, int(data[4]) + 1)
            )
            self.apply_task_result(list(data), source='vision')
            return

        # 其他阶段不允许 /cam_return 决定主任务，避免识别板2或杂帧污染下一轮。
        return

    def make_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def clear_costmap_safe(self):
        try:
            self.clear_costmaps_service()
        except Exception as e:
            rospy.logwarn('clear_costmaps failed: %s' % e)

    def move(self, goal, timeout=90.0):
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn('Timed out achieving goal')
            return False
        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal succeeded!')
            return True
        rospy.logwarn('Goal failed, state=%s' % str(state))
        return False

    def go_to_waypoint(self, index, task, phase, task_id, info):
        self.publish_task(task, phase, task_id, info)
        goal = self.make_goal(self.waypoints[index])
        ok = self.move(goal)
        if ok:
            self.clear_costmap_safe()
            rospy.sleep(0.3)
        return ok

    def play_audio_file(self, filename):
        path = os.path.join(AUDIO_DIR, filename)
        cmd = 'play ' + path
        rospy.loginfo('[语音] %s' % cmd)
        os.system(cmd)

    def play_pickup_audio(self):
        cv = self.build_cv1_from_windows()
        if cv not in VALID_CV:
            return
        if self.windows_1234 == 3:
            prefix = 'xuejiang'
        elif self.windows_1234 == 2:
            prefix = 'zuzhi'
        elif self.windows_1234 == 1:
            prefix = 'tuoye'
        else:
            prefix = 'jingmaixue'
        self.play_audio_file(prefix + cv + '.wav')

    def play_lab_audio(self):
        if self.windows_1234 == 3:
            prefix = 'jisu'
        elif self.windows_1234 == 2:
            prefix = 'mianyi'
        elif self.windows_1234 == 1:
            prefix = 'tiye'
        else:
            prefix = 'xuechanggui'
        self.play_audio_file(prefix + str(self.windows_count) + '.wav')

    def reset_for_new_round(self):
        self.has_valid_task = False
        self.task_locked = False
        self.default_task_used = False
        self.default_pending = False
        self.default_pending_start_time = 0.0
        self.cv1_wait_start_time = 0.0
        self.current_cv1 = ''
        self.current_cv2 = ''

        # ================== CV1 预识别缓存 ==================
        # count == 9 前往识别板1途中，如果视觉节点已经发布有效结果，先缓存。
        # count == 10 到达识别板1后，优先使用缓存，避免错过最佳识别视角。
        self.latest_cv1_task = None
        self.latest_cv1_task_time = 0.0
        self.latest_cv1_task_cv = ''
        self.ram_result = list(self.DEFAULT_TASK)
        self.windows_C = self.DEFAULT_TASK[0]
        self.windows_A = self.DEFAULT_TASK[1]
        self.windows_B = self.DEFAULT_TASK[2]
        self.windows_count = self.DEFAULT_TASK[3]
        self.windows_1234 = self.DEFAULT_TASK[4]
        self.error_count = 0

    def wait_or_apply_default(self):
        # 优先使用前往识别板1途中缓存到的有效结果。
        # 这能解决“路上已经识别到 ABC1，停车后看不到，最终默认 ABC4”的问题。
        if (not self.has_valid_task) and (self.latest_cv1_task is not None):
            cache_age = time.time() - self.latest_cv1_task_time
            if cache_age <= self.CV1_PRECACHE_MAX_AGE:
                rospy.logwarn(
                    '[CV1] 使用前往途中缓存结果: %s CV1=%s age=%.1f target=%d' %
                    (str(self.latest_cv1_task), self.latest_cv1_task_cv,
                     cache_age, int(self.latest_cv1_task[4]) + 1)
                )
                self.apply_task_result(list(self.latest_cv1_task), source='vision_precache')
            else:
                rospy.logwarn(
                    '[CV1] 缓存结果过期，忽略: age=%.1f / %.1f' %
                    (cache_age, self.CV1_PRECACHE_MAX_AGE)
                )
                self.latest_cv1_task = None
                self.latest_cv1_task_time = 0.0
                self.latest_cv1_task_cv = ''

        if self.has_valid_task:
            # 如果是超时默认值，先短暂保持可覆盖状态，等待迟到的视觉结果。
            # 实测日志里 /cam_return 经常在超时后 1~2 秒才发布；这里可以避免直接锁定 ABC4。
            if self.default_pending:
                pending_time = time.time() - self.default_pending_start_time
                if pending_time < self.CV1_DEFAULT_COMMIT_DELAY:
                    rospy.logwarn_throttle(
                        0.5,
                        '[CV1] 已进入默认ABC4待提交，仍允许视觉覆盖 %.1f / %.1f 秒...' %
                        (pending_time, self.CV1_DEFAULT_COMMIT_DELAY)
                    )
                    rospy.sleep(0.05)
                    return False
                rospy.logerr('[CV1] 默认ABC4待提交结束，正式使用默认任务')
                self.default_pending = False
                return True
            return True

        wait_time = time.time() - self.cv1_wait_start_time
        if wait_time < self.CV1_WAIT_TIMEOUT:
            rospy.logwarn_throttle(
                0.5,
                '[CV1] 等待识别结果 %.1f / %.1f 秒...' %
                (wait_time, self.CV1_WAIT_TIMEOUT)
            )
            rospy.sleep(0.05)
            return False

        rospy.logerr(
            '[CV1] %.1f 秒未收到有效 /cam_return，进入默认ABC4待提交状态' %
            self.CV1_WAIT_TIMEOUT
        )
        self.apply_task_result(list(self.DEFAULT_TASK), source='timeout_default')
        self.default_pending = True
        self.default_pending_start_time = time.time()
        return False

    def execute_pickups(self):
        # 原程序顺序是 C -> A -> B，这里保持不变。
        if self.windows_C == 1:
            if not self.go_to_waypoint(0, 'C', 'pickup_C', 2, '前往/到达取药窗口C'):
                return False
            rospy.loginfo('取到 C 窗口中的样品')
            rospy.sleep(0.5)

        if self.windows_A == 1:
            if not self.go_to_waypoint(1, 'A', 'pickup_A', 2, '前往/到达取药窗口A'):
                return False
            rospy.loginfo('取到 A 窗口中的样品')
            rospy.sleep(0.5)

        if self.windows_B == 1:
            if not self.go_to_waypoint(2, 'B', 'pickup_B', 2, '前往/到达取药窗口B'):
                return False
            rospy.loginfo('取到 B 窗口中的样品')
            rospy.sleep(0.5)

        self.play_pickup_audio()
        return True

    def run_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.count == 9:
                rospy.loginfo('从起点到识别区')
                self.reset_for_new_round()
                rospy.logwarn('[CV1] 已清空上一轮缓存，开始允许前往途中预识别')
                ok = self.go_to_waypoint(
                    9, 'default', 'cv1_goto', 1, '前往识别板1'
                )
                if ok:
                    rospy.logwarn('到达识别板1，开始等待 CV1')
                    self.count = 10
                    self.cv1_wait_start_time = time.time()
                    self.has_valid_task = False
                    self.task_locked = False
                    self.default_pending = False
                    self.default_pending_start_time = 0.0
                    self.publish_task('default', 'cv1_wait', 1, '等待识别板1 /cam_return')
                else:
                    rospy.logwarn('识别板1导航失败，2秒后重试')
                    rospy.sleep(2.0)

            elif self.count == 10:
                if not self.wait_or_apply_default():
                    rate.sleep()
                    continue

                self.task_locked = True
                rospy.logwarn(
                    '[CV1] 最终执行任务: C=%d A=%d B=%d count=%d target=%d source=%s' %
                    (self.windows_C, self.windows_A, self.windows_B,
                     self.windows_count, self.windows_1234 + 1,
                     'default' if self.default_task_used else 'vision')
                )
                self.publish_task(
                    self.current_cv1 if self.current_cv1 else 'default',
                    'pickup_start', 2, '开始取药'
                )

                if self.execute_pickups():
                    self.count = 11
                else:
                    rospy.logwarn('取药阶段导航失败，1秒后重试本阶段')
                    rospy.sleep(1.0)

            elif self.count == 11:
                rospy.loginfo('从配药区到识别板2/答题区')
                ok = self.go_to_waypoint(
                    8, 'default', 'cv2_goto', 3, '前往识别板2/答题区'
                )
                if ok:
                    rospy.logwarn('到达识别板2，停留 %.1f 秒等待 CV2' %
                                  self.CV2_WAIT_TIME)
                    self.count = 12
                    self.publish_task('default', 'cv2_wait', 3, '等待识别板2 CV2')
                    rospy.sleep(self.CV2_WAIT_TIME)
                else:
                    rospy.logwarn('识别板2导航失败，1秒后重试')
                    rospy.sleep(1.0)

            elif self.count == 12:
                lab_task = str(self.windows_1234 + 1)
                lab_index = 6 - self.windows_1234
                ok = self.go_to_waypoint(
                    lab_index, lab_task, 'lab', 4,
                    '前往数字/化验区%s' % lab_task
                )
                if ok:
                    rospy.logwarn('到达数字/化验区 %s，送样完成' % lab_task)
                    self.publish_task(lab_task, 'deliver', 4, '化验区送药完成')
                    self.play_lab_audio()
                    rospy.sleep(1.0)
                    self.count = 9
                else:
                    rospy.logwarn('数字/化验区导航失败，1秒后重试')
                    rospy.sleep(1.0)

            rate.sleep()

    def shutdown(self):
        rospy.loginfo('Stopping the robot...')
        try:
            self.move_base.cancel_goal()
        except Exception:
            pass
        rospy.sleep(0.5)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation test finished.')
