#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import tf
import math
from nav_msgs.msg import Odometry

SERVER_IP = '192.168.12.159'
SERVER_PORT = 8888

MAP_FRAME = 'map'
BASE_FRAME = 'base_link'

REPORT_HZ = 5
RECONNECT_INTERVAL = 2.0


class RefereeReporter(object):
    def __init__(self):
        rospy.init_node('referee_reporter_node', anonymous=True)

        self.linear_v = 0.0
        self.angular_w = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0

        self.has_odom = False
        self.has_pose = False

        self.vision_cv1 = "none"
        self.vision_cv2 = "none"
        self.current_task = "idle"
        self.crash = 0

        self.sock = None

        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        self.tf_listener = tf.TransformListener()

    def odom_callback(self, msg):
        self.linear_v = msg.twist.twist.linear.x
        self.angular_w = msg.twist.twist.angular.z

        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        self.odom_yaw = yaw

        self.has_odom = True

    def update_map_pose(self):
        try:
            self.tf_listener.waitForTransform(
                MAP_FRAME,
                BASE_FRAME,
                rospy.Time(0),
                rospy.Duration(0.2)
            )

            trans, rot = self.tf_listener.lookupTransform(
                MAP_FRAME,
                BASE_FRAME,
                rospy.Time(0)
            )

            self.map_x = trans[0]
            self.map_y = trans[1]

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            self.map_yaw = yaw

            self.has_pose = True
            return True

        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "cannot get TF %s -> %s: %s",
                MAP_FRAME,
                BASE_FRAME,
                str(e)
            )
            self.has_pose = False
            return False

    def connect_to_referee(self):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("connecting referee %s:%d ...", SERVER_IP, SERVER_PORT)

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((SERVER_IP, SERVER_PORT))
                self.sock.settimeout(None)

                rospy.loginfo("connected referee")
                return True

            except Exception as e:
                rospy.logerr("connect failed: %s, retry in %.1f seconds", str(e), RECONNECT_INTERVAL)

                try:
                    self.sock.close()
                except Exception:
                    pass

                self.sock = None
                rospy.sleep(RECONNECT_INTERVAL)

        return False

    def build_message(self):
        speed_abs = abs(self.linear_v)

        data = {
            "task": self.current_task,

            "speed": round(speed_abs, 3),

            "odom": {
                "x": round(self.odom_x, 3),
                "y": round(self.odom_y, 3),
                "yaw": round(self.odom_yaw, 3),
                "linear_v": round(self.linear_v, 3),
                "angular_w": round(self.angular_w, 3)
            },

            "position": {
                "x": round(self.map_x, 3),
                "y": round(self.map_y, 3),
                "yaw": round(self.map_yaw, 3)
            },

            "CV1": self.vision_cv1,
            "CV2": self.vision_cv2,
            "CRASH": self.crash,

            "has_odom": self.has_odom,
            "has_pose": self.has_pose,

            "stamp": round(rospy.Time.now().to_sec(), 3)
        }

        return json.dumps(data) + "\r\n"

    def run(self):
        rate = rospy.Rate(REPORT_HZ)

        while not rospy.is_shutdown():
            if self.sock is None:
                if not self.connect_to_referee():
                    continue

            try:
                self.update_map_pose()

                msg = self.build_message()
                self.sock.sendall(msg.encode('utf-8'))

                rospy.loginfo_throttle(1.0, "send: %s", msg.strip())

                rate.sleep()

            except Exception as e:
                rospy.logerr("send failed: %s, reconnecting", str(e))

                try:
                    self.sock.close()
                except Exception:
                    pass

                self.sock = None
                rospy.sleep(RECONNECT_INTERVAL)


if __name__ == '__main__':
    try:
        node = RefereeReporter()
        node.run()
    except rospy.ROSInterruptException:
        pass
