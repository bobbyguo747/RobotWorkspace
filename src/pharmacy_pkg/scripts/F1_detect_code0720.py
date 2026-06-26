#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import cv2, cv_bridge
# import cv2
import rospy
import random
import math
import numpy as np
from enum import Enum
# from detectColor import detectColor
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from pyzbar.pyzbar import decode
from collections import deque, Counter

# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg

os.system('export LANG=zh_CN.UTF-8')  # Linux 终端设置编码

approxPolyDP_epslion = 0.02  # 多边形近似参数，越小越精准
wh_rate = 0.4  # 长宽比系数，越小越接近正方形
min_area = 500
max_area = 20000

min_center_distance = 20  # 中心距离

fps = 0

detect_num = 0

# ================== 【新增：识别稳定性参数】 ==================
# 保留原代码结构，只增加稳定确认和合法性过滤。
# 说明：
# 1. 识别端没有识别到时不发布消息，导航端会使用备用结果继续跑，不会卡死。
# 2. 连续多帧结果一致才发布，减少偶发误识别。
# 3. 只允许 A/B/C/AB/AC/BC/ABC 参与判断，非法内容不再当成有效任务。
DEBUG_SAVE_AREAS = False          # 调试时可改 True，保存四个裁剪区域图片；比赛建议 False，减少磁盘写入
VALID_CODES = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']
NORMAL_CODES = VALID_CODES + ['']

STABLE_HISTORY_SIZE = 5           # 最近保存几次有效识别结果
STABLE_MIN_COUNT = 2              # 至少几次一致才发布；追求更稳可改 3，追求更快可改 2
result_history = deque(maxlen=STABLE_HISTORY_SIZE)
last_published_result = None


def build_result_msg(all_data):
    """
    根据四个二维码区域内容生成 /cam_return 消息。
    输出格式：
        [是否去C, 是否去A, 是否去B, 样本数量, 目标窗口编号0~3, 错误窗口编号0~4]
    说明：
        错误窗口编号 Error_count：0 表示无错误；1~4 表示第几个区域有非法内容。
    """
    # 记录非法二维码区域。为了兼容原代码，Error_count 仍然使用 1~4；0 表示没有错误。
    Error_count = 0
    for index, data in enumerate(all_data):
        if data not in NORMAL_CODES and data != "":
            Error_count = index + 1
            rospy.logwarn("窗口 %d 出现非法二维码内容: %s", Error_count, data)
            break

    # 只有合法内容参与长度比较，非法内容长度直接按 0 处理，防止 AD/12 之类误参与判断。
    img_len = []
    for data in all_data:
        if data in VALID_CODES:
            img_len.append(len(data))
        else:
            img_len.append(0)

    max_img = max(img_len)
    if max_img == 0:
        return None

    windows_count = img_len.index(max_img)   # 0~3，对应四个二维码区域
    img_abc = all_data[windows_count]

    msg = Int32MultiArray()

    if img_abc == 'ABC':
        msg.data = [1, 1, 1, 3, windows_count, Error_count]
    elif img_abc == 'AB':
        msg.data = [0, 1, 1, 2, windows_count, Error_count]
    elif img_abc == 'BC':
        msg.data = [1, 0, 1, 2, windows_count, Error_count]
    elif img_abc == 'AC':
        msg.data = [1, 1, 0, 2, windows_count, Error_count]
    elif img_abc == 'A':
        msg.data = [0, 1, 0, 1, windows_count, Error_count]
    elif img_abc == 'B':
        msg.data = [0, 0, 1, 1, windows_count, Error_count]
    elif img_abc == 'C':
        msg.data = [1, 0, 0, 1, windows_count, Error_count]
    else:
        # 理论上不会进入这里，保底不发布
        return None

    return msg


def publish_stable_result(msg):
    """
    多帧稳定确认后再发布。
    如果同一结果已经发布过，就不重复刷屏发布。
    """
    global last_published_result

    result = tuple(msg.data)
    result_history.append(result)

    most_common, count = Counter(result_history).most_common(1)[0]
    if count >= STABLE_MIN_COUNT:
        if most_common != last_published_result:
            stable_msg = Int32MultiArray()
            stable_msg.data = list(most_common)
            # ================== 终端输出识别数组 ==================
            # 这里会在运行 F1_detect_code0719.py 的 Ubuntu 终端中显示最终发布到 /cam_return 的数组
            print("\n================ 二维码识别端输出数组 ================")
            print("/cam_return = %s" % stable_msg.data)
            print("数组含义: [是否去C, 是否去A, 是否去B, 样本数, 目标窗口编号, 错误区域编号]")
            print("====================================================\n")
            rospy.logwarn("二维码识别端发布 /cam_return: %s", stable_msg.data)
            pub_flag.publish(stable_msg)
            last_published_result = most_common
            rospy.logwarn("稳定识别结果发布: %s", stable_msg.data)
        else:
            rospy.loginfo("稳定结果未变化，不重复发布: %s", list(most_common))


def distance(point1, point2):
    """计算距离"""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def get_locating_point(locating_boxs):
    """
    这个函数名为 get_locating_point，它的作用是获取定位框的中心点。传入的参数 locating_boxs 是一个包含若干个定位框的列表，每个定位框是一个由点坐标构成的列表。
    该函数首先定义了一个空列表 dingweidian 和一个空列表 locating_points。然后遍历传入的所有定位框，通过 OpenCV 的 cv2.moments 函数计算每个定位框的中心点坐标，并将其添加到 dingweidian 列表中。
    接着调用了 filter_points 函数对 dingweidian 中的点进行筛选，把距离较近的点合并成一个点，最终将得到的筛选后的点添加到 locating_points 列表中。
    最后返回 locating_points 列表，其中包含了所有定位框的中心点坐标。
    """
    # 定义两个空列表
    dingweidian = []
    locating_points = []
    # 循环定位框列表中的每一个框
    for i in range(len(locating_boxs)):
        # 计算当前框的矩（moments）
        M1 = cv2.moments(locating_boxs[i])
        # 计算当前框的重心坐标
        cx1 = int(M1['m10'] / M1['m00'])
        cy1 = int(M1['m01'] / M1['m00'])
        # 将当前框的重心坐标添加到列表中
        dingweidian.append((cx1, cy1))
    # 将中心距里较近的点合并成一个点
    locating_points = filter_points(dingweidian, 10)
    # 返回合并后的点列表
    return locating_points


rospy.init_node('detect_abc', anonymous=True)

pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)

# 打开摄像头
cap = cv2.VideoCapture("http://192.168.12.1:8080/stream?topic=/camera/rgb/image_raw")
# cap = cv2.VideoCapture("http://192.168.31.66:8080/stream?topic=/usb_cam/image_raw")

# imgA = cv2.imread('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/A.jpg')
# imgB = cv2.imread('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/B.jpg')
# imgC = cv2.imread('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/C.jpg')
# imgD = cv2.imread('/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/NULL.jpg')

# gray_imgA = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
# gray_imgB = cv2.cvtColor(imgB, cv2.COLOR_BGR2GRAY)
# gray_imgC = cv2.cvtColor(imgC, cv2.COLOR_BGR2GRAY)
# gray_imgD = cv2.cvtColor(imgD, cv2.COLOR_BGR2GRAY)

# retA, binary_imgA = cv2.threshold(gray_imgA, 127, 255, cv2.THRESH_BINARY)
# retB, binary_imgB = cv2.threshold(gray_imgB, 127, 255, cv2.THRESH_BINARY)
# retC, binary_imgC = cv2.threshold(gray_imgC, 127, 255, cv2.THRESH_BINARY)
# retD, binary_imgD = cv2.threshold(gray_imgD, 127, 255, cv2.THRESH_BINARY)

# _, contoursA, hierarchyA = cv2.findContours(binary_imgA, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
# _, contoursB, hierarchyB = cv2.findContours(binary_imgB, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
# _, contoursC, hierarchyC = cv2.findContours(binary_imgC, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
# _, contoursD, hierarchyD = cv2.findContours(binary_imgD, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

while not rospy.is_shutdown():

    # ================== 【改进 1：清空缓存，防滞后】 ==================
    # 连续抓取并丢弃5帧，清空 OpenCV 底层队列里的旧图片
    # 保证接下来拿到的是此刻机器人停稳后、摄像头最新鲜的画面
    for _ in range(5):
        cap.grab()  # grab() 只抓取不解码，速度极快
    
    # 真正去解码并获取最新的一帧
    hx, frame = cap.retrieve()

    # ================== 【改进 2：防崩溃保护】 ==================
    # 如果 hx 为 False，说明此时网络波动或摄像头偶尔卡顿
    if hx is False or frame is None:
        # 打印报错，但**不要直接 exit(0) 退出程序**！否则遇到一点小波动节点就死了。
        print('read video error, retrying...')
        rospy.sleep(0.5)  # 稍微等半秒钟，让摄像头缓冲一下
        continue          # 直接进入下一次循环重试

    # ================== 保留你原本的图像旋转逻辑 ==================
    # 获取图像的高度和宽度
    height, width = frame.shape[:2]

    # 定义旋转角度（正值表示逆时针旋转）
    angle = 5

    # 计算旋转中心
    center = (width // 2, height // 2)

    # 定义旋转矩阵
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

    # 执行旋转操作
    frame = cv2.warpAffine(frame, matrix, (width, height))
    # ==============================================================
    
    # 接下来是你原本的图像处理代码...
    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = True
    # ......

    # grayImg or gray

    edges = cv2.Canny(grayImg, 50, 150, apertureSize=3)
    # edges = gray

    _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 筛选条件0 面积
    contours2 = []
    for i in range(len(contours)):
        M1 = cv2.moments(contours[i])
        if M1['m00'] < min_area or M1['m00'] > max_area:
            continue
        contours2.append(contours[i])

    # 筛选条件1 四边形
    sibianxing = []
    for c in contours2:
        # 对轮廓进行多边形逼近
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)
        hull = approx
        if len(hull) == 4:
            sibianxing.append(hull)

    # 筛选条件2 正方形
    # 判断正方形方案1，判断长宽比
    zhengfangxing = []
    for hull in sibianxing:

        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), angle = rect
        abs(w - h)
        if abs(w - h) / (w + h) < wh_rate:  # 边长归一标准差
            zhengfangxing.append(hull)

    # 筛选条件3 正方形的包含关系
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
            if M2['m00'] == 0:  # 两者面积为0则跳过
                continue
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            # 1二者中心互相包含
            res1 = cv2.pointPolygonTest(zhengfangxing[i], (cx2, cy2), False)
            res2 = cv2.pointPolygonTest(zhengfangxing[j], (cx1, cy1), False)
            if res1 > 0 and res2 > 0:  # 如果两个轮廓中心具有包含关系
                # 2计算中心距离
                distance_ = distance((cx1, cy1), (cx2, cy2))
                if distance_ < min_center_distance:  # 中心距里小于5个像素
                    dingweikuang += [zhengfangxing[i], zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])
    if display:
        # cv2.imshow('step1_edges', edges)

        # contours_frame=frame.copy()
        # cv2.drawContours(contours_frame,contours, -1, (0, 0, 255), 1)
        # cv2.putText(contours_frame, "counts:{}".format(len(contours)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step2_all_contours', contours_frame)

        # contours2_frame=frame.copy()
        # cv2.drawContours(contours2_frame,contours2, -1, (0, 0, 255), 1)
        # "FPS: {:.2f}".format(fps)
        # cv2.putText(contours2_frame, "min_area:{},max_area:{},counts:{}".format(min_area,max_area,len(contours2)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step3_Area_Selected_contours', contours2_frame)

        # sibianxing_frame=frame.copy()
        # cv2.drawContours(sibianxing_frame,sibianxing, -1, (0, 0, 255), 1)
        # cv2.putText(sibianxing_frame, "approxPolyDP_epslion:{:.2f},counts:{}".format(approxPolyDP_epslion,len(sibianxing)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step4_quadrilateral_selection', sibianxing_frame)

        # zhengfangxing_frame=frame.copy()
        # cv2.drawContours(zhengfangxing_frame,zhengfangxing, -1, (0, 0, 255), 1)
        # cv2.putText(zhengfangxing_frame, "wh_rate:{:.2f},counts:{}".format(wh_rate,len(zhengfangxing)), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.imshow('step5_Square_selection', zhengfangxing_frame)

        dingweikuang_frame = frame.copy()
        if (40 <= len(dingweikuang) <= 60 and len(jieguo) >= 4):

            min_x = 1000
            max_x = 0
            min_y = 1000
            max_y = 0

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

            x0 = jieguo[min_x_index[0]][min_x_index[1]][0][0]
            y0 = jieguo[min_x_index[0]][min_x_index[1]][0][1]
            x1 = jieguo[max_x_index[0]][max_x_index[1]][0][0]
            y1 = jieguo[max_x_index[0]][max_x_index[1]][0][1]
            x2 = jieguo[min_y_index[0]][min_y_index[1]][0][0]
            y2 = jieguo[min_y_index[0]][min_y_index[1]][0][1]
            x3 = jieguo[max_y_index[0]][max_y_index[1]][0][0]
            y3 = jieguo[max_y_index[0]][max_y_index[1]][0][1]

            cv2.circle(dingweikuang_frame, (x0, y0), 4, color=(0, 0, 255), thickness=2)
            cv2.circle(dingweikuang_frame, (x1, y1), 4, color=(255, 0, 0), thickness=2)
            cv2.circle(dingweikuang_frame, (x2, y2), 4, color=(255, 255, 255), thickness=2)
            cv2.circle(dingweikuang_frame, (x3, y3), 4, color=(255, 255, 255), thickness=2)

            real_box_centers = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
            if min_x_index[0] < max_x_index[0]:
                goal_box_centers = [(50, 550), (550, 50), (50, 50), (550, 550)]
            else:
                goal_box_centers = [(50, 50), (550, 550), (550, 50), (50, 550)]

            dst_pts = np.array(goal_box_centers, dtype=np.float32)
            src_pts = np.array(real_box_centers, dtype=np.float32)
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped_img = cv2.warpPerspective(dingweikuang_frame, M, (600, 600))
            # cv2.imshow('warped_img', warped_img)
            aera1_crop = warped_img[0:300, 0:300] # 50:270, 50:250
            # cv2.imshow('aera1_crop', aera1_crop)
            if DEBUG_SAVE_AREAS:
                cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera1.jpg", aera1_crop)
            aera2_crop = warped_img[0:300, 300:600] # 50:270, 350：550
            # cv2.imshow('aera2_crop', aera2_crop)
            if DEBUG_SAVE_AREAS:
                cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera2.jpg", aera2_crop)
            aera3_crop = warped_img[300:600, 0:300] # 320:540, 50:250
            # cv2.imshow('aera3_crop', aera3_crop)
            if DEBUG_SAVE_AREAS:
                cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera3.jpg", aera3_crop)
            aera4_crop = warped_img[300:600, 300:600] # 320:540, 350:550
            # cv2.imshow('aera4_crop', aera4_crop)
            if DEBUG_SAVE_AREAS:
                cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera4.jpg", aera4_crop)

            # print("Found ABC! Result img has saved in robot!")

# 1. 灰度化
            gray_img1 = cv2.cvtColor(aera1_crop, cv2.COLOR_BGR2GRAY)
            gray_img2 = cv2.cvtColor(aera2_crop, cv2.COLOR_BGR2GRAY)
            gray_img3 = cv2.cvtColor(aera3_crop, cv2.COLOR_BGR2GRAY)
            gray_img4 = cv2.cvtColor(aera4_crop, cv2.COLOR_BGR2GRAY)

            # 2. 图像增强：使用 CLAHE 处理光照不均（消除阴影和反光影响）
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enh_img1 = clahe.apply(gray_img1)
            enh_img2 = clahe.apply(gray_img2)
            enh_img3 = clahe.apply(gray_img3)
            enh_img4 = clahe.apply(gray_img4)

            # 3. 锐化图像（可选，如果摄像头稍微失焦可以用这个，如果本身很清晰可注释掉）
            # kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
            # enh_img1 = cv2.filter2D(enh_img1, -1, kernel)
            # ... 其他几个同理

            # 4. 直接把增强后的灰度图交给 pyzbar 解析！(千万别用固定阈值的二值化)
            img0_code = decode(enh_img1)
            img1_code = decode(enh_img2)
            img2_code = decode(enh_img3)
            img3_code = decode(enh_img4)

# ================= 【改进：合法性过滤 + 多帧稳定发布】 =================
            # 初始化为空字符串，而不是空列表 []
            img0_data = ""
            img1_data = ""
            img2_data = ""
            img3_data = ""

            # 提取二维码内字符串
            for QR in img0_code:
                img0_data = QR.data.decode("utf-8")
            for QR in img1_code:
                img1_data = QR.data.decode("utf-8")
            for QR in img2_code:
                img2_data = QR.data.decode("utf-8")
            for QR in img3_code:
                img3_data = QR.data.decode("utf-8")

            # 将扫出的二维码存入 all_data
            all_data = [img0_data, img1_data, img2_data, img3_data]

            # 如果四个区域全为空，说明本帧没识别到；识别端不发布，导航端会按备用结果继续跑。
            if all_data == ["", "", "", ""]:
                rospy.loginfo("本帧未识别到二维码，继续读取下一帧")
                continue

            rospy.logwarn("识别到二维码数据: %s", all_data)

            msg = build_result_msg(all_data)
            if msg is None:
                rospy.logwarn("本帧没有合法任务二维码，不发布: %s", all_data)
                continue

            rospy.logwarn("本帧候选输出: %s", msg.data)
            publish_stable_result(msg)
            # ================= 替换到这里结束 =================
            # 打印结果
            # rospy.loginfo("win0 %s win1 %s win2 %s win3 %s goto %d .", img0_data, img1_data, img2_data, img3_data, windows_count)

    # 监测键盘输入是否为q，为q则退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按q退出
        break

# 释放摄像头
cap.release()

# 结束所有窗口
cv2.destroyAllWindows()

