#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
F1_referee_report0719.py

独立裁判软件通信节点（双模式修正版）。
只负责：订阅导航/视觉状态 -> 组包 JSON -> TCP 通信。
不参与导航决策；裁判软件断开、IP 错误、发送失败，都不会影响小车主任务。

为什么做双模式：
1. 如果裁判软件是“服务器”，小车主动连接裁判电脑：client 模式。
2. 如果裁判软件是“客户端”，裁判电脑主动连接小车：server 模式。
3. 默认 both，两种方式同时开。哪种接上就用哪种，不影响导航。

订阅：
  /odom           nav_msgs/Odometry
  /vision_report  std_msgs/String    内容：A/B/C/AB/AC/BC/ABC
  /referee_task   std_msgs/String    内容：JSON，如 {"task":"A", "phase":"pickup_A"}

参数：
  ~server_ip 默认 192.168.5.2     # 裁判电脑IP，小车主动连它
  ~server_port 默认 8888
  ~listen_ip 默认 0.0.0.0         # 小车本地监听，裁判软件主动连小车时用
  ~listen_port 默认 8888
  ~enable_client 默认 True
  ~enable_server 默认 True
  ~report_hz 默认 2.0
  ~connect_timeout 默认 0.2
  ~reconnect_interval 默认 1.0
  ~cv2_fallback_to_cv1 默认 True
"""

import json
import math
import socket
import time

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String

VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']


def get_bool_param(name, default_value):
    value = rospy.get_param(name, default_value)
    if isinstance(value, bool):
        return value
    value = str(value).strip().lower()
    return value in ['1', 'true', 'yes', 'on']


class RefereeReporter0719(object):
    def __init__(self):
        rospy.init_node('f1_referee_report0719', anonymous=False)

        self.server_ip = rospy.get_param('~server_ip', '192.168.5.2')
        self.server_port = int(rospy.get_param('~server_port', 8888))
        self.listen_ip = rospy.get_param('~listen_ip', '0.0.0.0')
        self.listen_port = int(rospy.get_param('~listen_port', self.server_port))
        self.enable_client = get_bool_param('~enable_client', True)
        self.enable_server = get_bool_param('~enable_server', True)
        self.report_hz = float(rospy.get_param('~report_hz', 2.0))
        self.connect_timeout = float(rospy.get_param('~connect_timeout', 0.2))
        self.reconnect_interval = float(rospy.get_param('~reconnect_interval', 1.0))
        self.cv2_fallback_to_cv1 = get_bool_param('~cv2_fallback_to_cv1', True)

        self.client_sock = None       # 小车主动连裁判电脑时使用
        self.listen_sock = None       # 小车作为服务器监听时使用
        self.accepted_clients = []    # 裁判软件主动连小车时的连接
        self.last_connect_try = 0.0
        self.send_count = 0

        self.current_task = 'default'
        self.current_phase = 'idle'
        self.current_info = ''
        self.task_id = 0

        self.cv1 = ''
        self.cv2 = ''
        self.latest_vision = ''
        self.latest_vision_time = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_v = 0.0
        self.angular_w = 0.0
        self.speed_abs = 0.0
        self.has_odom = False

        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/vision_report', String, self.vision_callback, queue_size=10)
        rospy.Subscriber('/referee_task', String, self.task_callback, queue_size=10)

        if self.enable_server:
            self.start_listen_server()

        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('[裁判通信] 独立节点启动')
        rospy.logwarn('[裁判通信] client模式=%s -> 裁判电脑 %s:%d' %
                      (str(self.enable_client), self.server_ip, self.server_port))
        rospy.logwarn('[裁判通信] server模式=%s -> 小车监听 %s:%d' %
                      (str(self.enable_server), self.listen_ip, self.listen_port))

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        try:
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        except Exception:
            yaw = 0.0
        self.odom_yaw = yaw

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.linear_v = vx
        self.angular_w = msg.twist.twist.angular.z
        self.speed_abs = math.sqrt(vx * vx + vy * vy)
        self.has_odom = True

    def task_callback(self, msg):
        text = str(msg.data).strip()
        if not text:
            return
        try:
            data = json.loads(text)
            self.current_task = str(data.get('task', self.current_task))
            self.current_phase = str(data.get('phase', self.current_phase))
            self.current_info = str(data.get('info', ''))
            self.task_id = int(data.get('task_id', self.task_id))

            cv1 = str(data.get('CV1', '')).strip()
            cv2 = str(data.get('CV2', '')).strip()
            if cv1 in VALID_CV:
                self.cv1 = cv1
            if cv2 in VALID_CV:
                self.cv2 = cv2
        except Exception:
            # 兼容直接发 task 字符串
            self.current_task = text

    def vision_callback(self, msg):
        result = str(msg.data).strip()
        if result not in VALID_CV:
            rospy.logwarn('[裁判通信] 收到非法视觉字符串，忽略: %s' % result)
            return

        self.latest_vision = result
        self.latest_vision_time = time.time()

        if self.current_phase in ['cv2_wait', 'board2', 'lab', 'deliver']:
            self.cv2 = result
            rospy.logwarn('[裁判通信] CV2 更新为: %s' % self.cv2)
        else:
            self.cv1 = result
            rospy.logwarn('[裁判通信] CV1 更新为: %s' % self.cv1)

    def start_listen_server(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.listen_ip, self.listen_port))
            sock.listen(3)
            sock.setblocking(False)
            self.listen_sock = sock
            rospy.logwarn('[裁判通信] 已启动本地TCP服务，等待裁判软件连接 %s:%d' %
                          (self.listen_ip, self.listen_port))
        except Exception as e:
            self.listen_sock = None
            rospy.logerr('[裁判通信] 本地监听启动失败: %s' % e)

    def accept_clients(self):
        if self.listen_sock is None:
            return
        while True:
            try:
                client, addr = self.listen_sock.accept()
                client.settimeout(0.2)
                self.accepted_clients.append(client)
                rospy.logwarn('[裁判通信] 裁判软件已连接到小车: %s:%s' %
                              (str(addr[0]), str(addr[1])))
            except socket.error:
                break
            except Exception as e:
                rospy.logwarn_throttle(1.0, '[裁判通信] accept失败: %s' % e)
                break

    def connect_to_referee_server(self):
        if not self.enable_client:
            return False
        if self.client_sock is not None:
            return True

        now = time.time()
        if now - self.last_connect_try < self.reconnect_interval:
            return False
        self.last_connect_try = now

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.connect_timeout)
            sock.connect((self.server_ip, self.server_port))
            sock.settimeout(0.2)
            self.client_sock = sock
            rospy.logwarn('[裁判通信] 小车已主动连接裁判电脑 %s:%d' %
                          (self.server_ip, self.server_port))
            return True
        except Exception as e:
            rospy.logwarn_throttle(2.0, '[裁判通信] 主动连接裁判电脑失败: %s' % e)
            try:
                sock.close()
            except Exception:
                pass
            self.client_sock = None
            return False

    def get_report_cv2(self):
        if self.cv2 in VALID_CV:
            return self.cv2
        if self.cv2_fallback_to_cv1 and self.current_phase in ['cv2_wait', 'board2', 'lab', 'deliver']:
            if self.cv1 in VALID_CV:
                return self.cv1
        return ''

    def build_message(self):
        data = {
            'task': str(self.current_task),
            'task_id': int(self.task_id),
            'phase': str(self.current_phase),
            'speed': round(float(self.speed_abs), 3),
            'odom': [
                round(float(self.odom_x), 3),
                round(float(self.odom_y), 3)
            ],
            'position': {
                'x': round(float(self.odom_x), 3),
                'y': round(float(self.odom_y), 3),
                'yaw': round(float(self.odom_yaw), 3),
                'linear_v': round(float(self.linear_v), 3),
                'angular_w': round(float(self.angular_w), 3)
            },
            'has_odom': bool(self.has_odom),
            'info': self.current_info,
            'stamp': round(float(rospy.Time.now().to_sec()), 3)
        }

        if self.cv1 in VALID_CV:
            data['CV1'] = self.cv1
        cv2_value = self.get_report_cv2()
        if cv2_value in VALID_CV:
            data['CV2'] = cv2_value

        return json.dumps(data, ensure_ascii=True) + '\n'

    def send_to_socket(self, sock, msg, label):
        try:
            sock.sendall(msg.encode('utf-8'))
            return True
        except Exception as e:
            rospy.logwarn('[裁判通信] %s发送失败，断开此连接: %s' % (label, e))
            try:
                sock.close()
            except Exception:
                pass
            return False

    def send_once(self):
        self.accept_clients()
        self.connect_to_referee_server()

        msg = self.build_message()
        sent = False

        if self.client_sock is not None:
            if self.send_to_socket(self.client_sock, msg, 'client模式'):
                sent = True
            else:
                self.client_sock = None

        alive_clients = []
        for client in self.accepted_clients:
            if self.send_to_socket(client, msg, 'server模式'):
                alive_clients.append(client)
                sent = True
        self.accepted_clients = alive_clients

        if sent:
            self.send_count += 1
            rospy.loginfo_throttle(1.0, '[裁判通信] send: %s' % msg.strip())
        else:
            rospy.logwarn_throttle(
                2.0,
                '[裁判通信] 暂无可用连接：若裁判主动连小车，请连 192.168.5.4:%d；若小车主动连裁判，请确认裁判电脑IP=%s 端口=%d' %
                (self.listen_port, self.server_ip, self.server_port)
            )

    def run(self):
        rate = rospy.Rate(self.report_hz)
        while not rospy.is_shutdown():
            self.send_once()
            rate.sleep()

    def shutdown(self):
        rospy.logwarn('[裁判通信] 节点关闭')
        for sock in [self.client_sock, self.listen_sock]:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        for client in self.accepted_clients:
            try:
                client.close()
            except Exception:
                pass
        self.client_sock = None
        self.listen_sock = None
        self.accepted_clients = []


if __name__ == '__main__':
    try:
        RefereeReporter0719().run()
    except rospy.ROSInterruptException:
        pass
