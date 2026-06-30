#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import cv2, cv_bridge
import rospy
import random
import math
import numpy as np
from enum import Enum
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from pyzbar.pyzbar import decode

os.system('export LANG=zh_CN.UTF-8')  # Linux 终端设置编码

approxPolyDP_epslion = 0.02  # 多边形近似参数，越小越精准
wh_rate = 0.4  # 长宽比系数，越小越接近正方形
min_area = 500
max_area = 20000
min_center_distance = 20  # 中心距离
fps = 0
detect_num = 0

VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']
last_vision_result = ""
last_vision_time = 0.0


def distance(point1, point2):
    """计算距离"""
    return math.sqrt((point1[0] - point2[0]) ** 2 +
                     (point1[1] - point2[1]) ** 2)


def get_locating_point(locating_boxs):
    dingweidian = []
    locating_points = []
    for i in range(len(locating_boxs)):
        M1 = cv2.moments(locating_boxs[i])
        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])
        dingweidian.append((cx1, cy1))
    locating_points = filter_points(dingweidian, 10)
    return locating_points


def publish_vision_result(result):
    """
    发布视觉字符串给主控节点。
    只发布 A/B/C/AB/AC/BC/ABC，过滤乱码；
    同一结果每 1 秒允许重复发一次，保证主控切换到识别板2阶段时还能收到。
    """
    global last_vision_result
    global last_vision_time

    if result not in VALID_CV:
        return

    now = time.time()
    if result != last_vision_result or now - last_vision_time > 1.0:
        pub_vision_str.publish(result)
        rospy.logwarn("发布视觉结果 /vision_report: %s" % result)
        last_vision_result = result
        last_vision_time = now


rospy.init_node('detect_abc', anonymous=True)

# 原有的控制指令发布者
pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)
# 视觉字符串发布者；主控根据 count 判断它是 CV1 还是 CV2
pub_vision_str = rospy.Publisher('/vision_report', String, queue_size=10)

# 打开摄像头
cap = cv2.VideoCapture(
    "http://192.168.5.4:8080/stream?topic=/camera/rgb/image_raw"
)

while not rospy.is_shutdown():

    # ================== 清空缓存，防滞后 ==================
    for _ in range(5):
        cap.grab()

    hx, frame = cap.retrieve()

    # ================== 防崩溃保护 ==================
    if hx is False:
        print('read video error, retrying...')
        rospy.sleep(0.5)
        continue

    height, width = frame.shape[:2]
    angle = 5
    center = (width // 2, height // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    frame = cv2.warpAffine(frame, matrix, (width, height))

    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = True

    edges = cv2.Canny(grayImg, 50, 150, apertureSize=3)
    _, contours, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    contours2 = []
    for i in range(len(contours)):
        M1 = cv2.moments(contours[i])
        if M1['m00'] < min_area or M1['m00'] > max_area:
            continue
        contours2.append(contours[i])

    sibianxing = []
    for c in contours2:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)
        hull = approx
        if len(hull) == 4:
            sibianxing.append(hull)

    zhengfangxing = []
    for hull in sibianxing:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        if abs(w - h) / (w + h) < wh_rate:
            zhengfangxing.append(hull)

    dingweikuang = []
    jieguo = []
    for i in range(len(zhengfangxing) - 1):
        M1 = cv2.moments(zhengfangxing[i])
        if M1['m00'] == 0:
            continue
        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])
        for j in range(i + 1, len(zhengfangxing)):
            M2 = cv2.moments(zhengfangxing[j])
            if M2['m00'] == 0:
                continue
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            res1 = cv2.pointPolygonTest(
                zhengfangxing[i], (cx2, cy2), False
            )
            res2 = cv2.pointPolygonTest(
                zhengfangxing[j], (cx1, cy1), False
            )
            if res1 > 0 and res2 > 0:
                distance_ = distance((cx1, cy1), (cx2, cy2))
                if distance_ < min_center_distance:
                    dingweikuang += [zhengfangxing[i], zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])

    if display:
        dingweikuang_frame = frame.copy()

        # 保留原来的 48 定位框规则，保证和当前识别板透视矫正逻辑一致。
        if (len(dingweikuang) == 48):

            min_x, max_x, min_y, max_y = 1000, 0, 1000, 0

            for i in range(len(jieguo)):
                for j in range(len(jieguo[0])):
                    if jieguo[i][j][0][0] < min_x:
                        min_x = jieguo[i][j][0][0]
                        min_x_index = [i, j]
                    if jieguo[i][j][0][0] > max_x:
                        max_x = jieguo[i][j][0][0]
                        max_x_index = [i, j]
                    if jieguo[i][j][0][1] < min_y:
                        min_y = jieguo[i][j][0][1]
                        min_y_index = [i, j]
                    if jieguo[i][j][0][1] > max_y:
                        max_y = jieguo[i][j][0][1]
                        max_y_index = [i, j]

            x0, y0 = jieguo[min_x_index[0]][min_x_index[1]][0]
            x1, y1 = jieguo[max_x_index[0]][max_x_index[1]][0]
            x2, y2 = jieguo[min_y_index[0]][min_y_index[1]][0]
            x3, y3 = jieguo[max_y_index[0]][max_y_index[1]][0]

            cv2.circle(dingweikuang_frame, (x0, y0), 4,
                       color=(0, 0, 255), thickness=2)
            cv2.circle(dingweikuang_frame, (x1, y1), 4,
                       color=(255, 0, 0), thickness=2)
            cv2.circle(dingweikuang_frame, (x2, y2), 4,
                       color=(255, 255, 255), thickness=2)
            cv2.circle(dingweikuang_frame, (x3, y3), 4,
                       color=(255, 255, 255), thickness=2)

            real_box_centers = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
            if min_x_index[0] < max_x_index[0]:
                goal_box_centers = [
                    (50, 550), (550, 50), (50, 50), (550, 550)
                ]
            else:
                goal_box_centers = [
                    (50, 50), (550, 550), (550, 50), (50, 550)
                ]

            dst_pts = np.array(goal_box_centers, dtype=np.float32)
            src_pts = np.array(real_box_centers, dtype=np.float32)
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped_img = cv2.warpPerspective(
                dingweikuang_frame, M, (600, 600)
            )

            aera1_crop = warped_img[0:300, 0:300]
            cv2.imwrite(
                "/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera1.jpg",
                aera1_crop
            )
            aera2_crop = warped_img[0:300, 300:600]
            cv2.imwrite(
                "/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera2.jpg",
                aera2_crop
            )
            aera3_crop = warped_img[300:600, 0:300]
            cv2.imwrite(
                "/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera3.jpg",
                aera3_crop
            )
            aera4_crop = warped_img[300:600, 300:600]
            cv2.imwrite(
                "/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera4.jpg",
                aera4_crop
            )

            gray_img1 = cv2.cvtColor(aera1_crop, cv2.COLOR_BGR2GRAY)
            gray_img2 = cv2.cvtColor(aera2_crop, cv2.COLOR_BGR2GRAY)
            gray_img3 = cv2.cvtColor(aera3_crop, cv2.COLOR_BGR2GRAY)
            gray_img4 = cv2.cvtColor(aera4_crop, cv2.COLOR_BGR2GRAY)

            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enh_img1 = clahe.apply(gray_img1)
            enh_img2 = clahe.apply(gray_img2)
            enh_img3 = clahe.apply(gray_img3)
            enh_img4 = clahe.apply(gray_img4)

            img0_code = decode(enh_img1)
            img1_code = decode(enh_img2)
            img2_code = decode(enh_img3)
            img3_code = decode(enh_img4)

            img0_data = ""
            img1_data = ""
            img2_data = ""
            img3_data = ""

            for QR in img0_code:
                img0_data = QR.data.decode("utf-8")
            for QR in img1_code:
                img1_data = QR.data.decode("utf-8")
            for QR in img2_code:
                img2_data = QR.data.decode("utf-8")
            for QR in img3_code:
                img3_data = QR.data.decode("utf-8")

            all_data = [img0_data, img1_data, img2_data, img3_data]

            if all_data == ["", "", "", ""]:
                continue

            rospy.logwarn("识别到二维码数据: %s", all_data)

            # 只把合法 A/B/C 组合纳入任务判定，乱码不参与最长长度选择。
            img_len = [
                len(x) if x in VALID_CV else 0 for x in all_data
            ]
            max_img = max(img_len)

            if max_img == 0:
                rospy.logwarn("未扫到符合规则的短字符(A/B/C)，继续捕捉下一帧...")
                continue

            windows_count = img_len.index(max_img)
            img_abc = all_data[windows_count]

            if img_abc not in VALID_CV:
                rospy.logwarn("识别结果不在合法集合内: %s" % img_abc)
                continue

            msg = Int32MultiArray()
            msg.data = [0, 0, 0, 0, 4, 0]
            Normal = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC', '']

            Error_count = 0
            for i in all_data:
                if i not in Normal and i != "":
                    Error_count = all_data.index(i) + 1
                    print("窗口 %d 出现错误信息" % Error_count)

            # 发布合法视觉字符串。主控节点决定它属于 CV1 还是 CV2。
            publish_vision_result(img_abc)

            if max_img == 3:
                msg.data = [1, 1, 1, 3, windows_count, Error_count]
                print("结果为ABC，输出为%s" % msg.data)
                pub_flag.publish(msg)

            elif max_img == 2:
                if img_abc == 'AB':
                    msg.data = [0, 1, 1, 2, windows_count, Error_count]
                    print("结果为AB，输出为%s" % msg.data)
                    pub_flag.publish(msg)
                elif img_abc == 'BC':
                    msg.data = [1, 0, 1, 2, windows_count, Error_count]
                    print("结果为BC，输出为%s" % msg.data)
                    pub_flag.publish(msg)
                elif img_abc == 'AC':
                    msg.data = [1, 1, 0, 2, windows_count, Error_count]
                    print("结果为AC，输出为%s" % msg.data)
                    pub_flag.publish(msg)

            elif max_img == 1:
                if img_abc == 'A':
                    msg.data = [0, 1, 0, 1, windows_count, Error_count]
                    print("结果为A，输出为%s" % msg.data)
                    pub_flag.publish(msg)
                elif img_abc == 'B':
                    msg.data = [0, 0, 1, 1, windows_count, Error_count]
                    print("结果为B，输出为%s" % msg.data)
                    pub_flag.publish(msg)
                elif img_abc == 'C':
                    msg.data = [1, 0, 0, 1, windows_count, Error_count]
                    print("结果为C，输出为%s" % msg.data)
                    pub_flag.publish(msg)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
