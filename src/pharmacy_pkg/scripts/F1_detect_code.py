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

    # 开始用摄像头读数据，返回hx为true则表示读成功，frame为读的图像
    hx, frame = cap.read()

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
# ================== 【新增：答题区板2纯视觉识别】 ==================
        # 原理：寻找瘦长型的数字"8"，汉字长宽比接近1，而"8"的长宽比>1.5
    board2_status = 0 # 0=未识别, 1=空闲(快速通过), 2=忙碌(需等8秒)
    try:
        h_f, w_f = frame.shape[:2]
         # 1. 截取中央区域（假设板子在画面中央）
        roi = frame[int(h_f*0.3):int(h_f*0.7), int(w_f*0.2):int(w_f*0.8)]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # 2. Otsu自动阈值二值化（黑字变白字）
        _, binary_roi = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            
            # 3. 仅截取下半部，只看第二行字，避开第一行的干扰
        h_r = binary_roi.shape[0]
        bottom_half = binary_roi[int(h_r*0.5):, :]
            
            # 4. 横向膨胀：强制左右偏旁的汉字粘连成方块，而独立的"8"不受影响
        kernel_morph = np.ones((4, 12), np.uint8)
        dilated = cv2.dilate(bottom_half, kernel_morph, iterations=1)
            
            # 5. 寻找轮廓块
        contours_text, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        is_busy = False
        valid_char_count = 0
        for c_t in contours_text:
            x_t, y_t, w_t, h_t = cv2.boundingRect(c_t)
            if w_t > 10 and h_t > 10: # 过滤太小的噪点
                valid_char_count += 1
                # 判断长宽比：如果高度超过宽度的 1.4 倍，说明看到了"8"
                if h_t / float(w_t) > 1.4:
                    is_busy = True
            
            # 只有当画面里确实有几个字块时，才认定识别有效
        if valid_char_count >= 3:
            if is_busy:
                board2_status = 2 # 发现"8"，忙碌中
            else:
                board2_status = 1 # 没发现"8"，空闲
    except Exception as e:
        pass

        # 默认发送消息数组：前5位设为-1表示无二维码，第6位存识别板2状态
    msg_out = Int32MultiArray()
    msg_out.data = [-1, -1, -1, -1, -1, board2_status]
        # ==============================================================
    # 显示旋转后的图像
    # cv2.imshow('Rotated Image', frame)
    # 如果hx为Flase表示开启摄像头失败，那么就输出"read vido error"并退出程序
    if hx is False:
        # 打印报错
        print('read video error')
        # 退出程序
        exit(0)

    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    display = True

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
        if (len(dingweikuang) == 48):

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
            aera1_crop = warped_img[65:255, 65:235]
            # cv2.imshow('aera1_crop', aera1_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera1.jpg", aera1_crop)
            aera2_crop = warped_img[65:255, 365:535]
            # cv2.imshow('aera2_crop', aera2_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera2.jpg", aera2_crop)
            aera3_crop = warped_img[335:525, 65:235]
            # cv2.imshow('aera3_crop', aera3_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera3.jpg", aera3_crop)
            aera4_crop = warped_img[335:525, 365:535]
            # cv2.imshow('aera4_crop', aera4_crop)
            cv2.imwrite("/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts/aera4.jpg", aera4_crop)

            # print("Found ABC! Result img has saved in robot!")

            # 灰度化
            gray_img1 = cv2.cvtColor(aera1_crop, cv2.COLOR_BGR2GRAY)
            gray_img2 = cv2.cvtColor(aera2_crop, cv2.COLOR_BGR2GRAY)
            gray_img3 = cv2.cvtColor(aera3_crop, cv2.COLOR_BGR2GRAY)
            gray_img4 = cv2.cvtColor(aera4_crop, cv2.COLOR_BGR2GRAY)

            # 二值化
            ret1, binary_img1 = cv2.threshold(gray_img1, 127, 255, cv2.THRESH_BINARY)
            ret2, binary_img2 = cv2.threshold(gray_img2, 127, 255, cv2.THRESH_BINARY)
            ret3, binary_img3 = cv2.threshold(gray_img3, 127, 255, cv2.THRESH_BINARY)
            ret4, binary_img4 = cv2.threshold(gray_img4, 127, 255, cv2.THRESH_BINARY)

            _, contours0, _ = cv2.findContours(binary_img1, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            _, contours1, _ = cv2.findContours(binary_img2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            _, contours2, _ = cv2.findContours(binary_img3, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            _, contours3, _ = cv2.findContours(binary_img4, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            # 进行二维码识别
            img0_code = decode(binary_img1)
            img1_code = decode(binary_img2)
            img2_code = decode(binary_img3)
            img3_code = decode(binary_img4)

# ================= 替换从这里开始 =================
        img0_data = ""
        img1_data = ""
        img2_data = ""
        img3_data = ""

        for QR in img0_code: img0_data = QR.data.decode("utf-8")
        for QR in img1_code: img1_data = QR.data.decode("utf-8")
        for QR in img2_code: img2_data = QR.data.decode("utf-8")
        for QR in img3_code: img3_data = QR.data.decode("utf-8")

        all_data = [img0_data, img1_data, img2_data, img3_data]
        
        # 将原有的 continue 改为 if 结构，防止中断循环导致无法发布板2的状态
        if all_data != ["", "", "", ""]:
            rospy.logwarn("识别到二维码数据: %s", all_data)
            img_len = [len(x) if len(x) < 4 else 0 for x in all_data]
            max_img = max(img_len)
            
            if max_img != 0:
                windows_count = img_len.index(max_img)
                img_abc = all_data[windows_count]
                
                Normal = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC', '']
                Error_count = 0
                for i in all_data:
                    if i not in Normal and i != "":
                        Error_count = all_data.index(i) + 1
                        print("窗口 %d 出现错误信息" % Error_count)

                # 将最终数据赋给 msg_out，一并携带 board2_status
                if max_img == 3:
                    msg_out.data = [1, 1, 1, 3, windows_count, board2_status]
                elif max_img == 2:
                    if img_abc == 'AB': msg_out.data = [0, 1, 1, 2, windows_count, board2_status]
                    elif img_abc == 'BC': msg_out.data = [1, 0, 1, 2, windows_count, board2_status]
                    elif img_abc == 'AC': msg_out.data = [1, 1, 0, 2, windows_count, board2_status]
                elif max_img == 1:
                    if img_abc == 'A': msg_out.data = [0, 1, 0, 1, windows_count, board2_status]
                    elif img_abc == 'B': msg_out.data = [0, 0, 1, 1, windows_count, board2_status]
                    elif img_abc == 'C': msg_out.data = [1, 0, 0, 1, windows_count, board2_status]
        # ================= 替换到这里结束 =================
        pub_flag.publish(msg_out)
            # 打印结果
            # rospy.loginfo("win0 %s win1 %s win2 %s win3 %s goto %d .", img0_data, img1_data, img2_data, img3_data, windows_count)

    # 监测键盘输入是否为q，为q则退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按q退出
        break

# 释放摄像头
cap.release()

# 结束所有窗口
cv2.destroyAllWindows()

