#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
F1_qr_only_static0719.py

独立二维码识别测试文件：
1. 只读取小车摄像头视频流。
2. 只做二维码识别、字符串解析、控制台输出。
3. 不包含任何小车运动控制、电机驱动、move_base、cmd_vel 代码。

来源说明：
- 摄像头流地址、cv2.VideoCapture、缓存清理、异常重连逻辑：
  提取自 F1_detect_code0719.py main()
- 定位框检测、透视矫正、四区域裁剪、pyzbar decode：
  提取自 F1_detect_code0719.py process_frame()
- A/B/C/AB/AC/BC/ABC 到 [C,A,B,count,target,error] 的映射：
  提取自 F1_detect_code0719.py task_msg_from_all_data()
"""

import math
import os
import time

import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode

os.system('export LANG=zh_CN.UTF-8')  # 来源：F1_detect_code0719.py

try:
    unicode
except NameError:
    unicode = str


# ================== 来源：F1_detect_code0719.py 全局配置 ==================
VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']
NORMAL_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC', '']
VALID_CV_U = [unicode(x) for x in VALID_CV]

SCRIPT_DIR = '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts'

approxPolyDP_epslion = 0.02
wh_rate = 0.4
min_area = 500
max_area = 80000
min_center_distance = 20


# ================== 来源：F1_detect_code0719.py distance() ==================
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 +
                     (point1[1] - point2[1]) ** 2)


# ================== 来源：F1_detect_code0719.py find_contours() ==================
def find_contours(edges):
    result = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        _, contours, hierarchy = result
    else:
        contours, hierarchy = result
    return contours


# ================== 来源：F1_detect_code0719.py normalize_cv_text() ==================
def normalize_cv_text(value):
    if value is None:
        return ''
    try:
        if isinstance(value, unicode):
            text_u = value
        else:
            text_u = value.decode('utf-8', 'ignore')
    except Exception:
        try:
            text_u = unicode(value)
        except Exception:
            return ''

    text_u = text_u.strip().upper()
    text_u = text_u.replace(u' ', u'')
    text_u = text_u.replace(u'\t', u'')
    text_u = text_u.replace(u'\r', u'')
    text_u = text_u.replace(u'\n', u'')
    text_u = text_u.replace(u'\x00', u'')

    if text_u in VALID_CV_U:
        try:
            return text_u.encode('ascii')
        except Exception:
            return ''
    return ''


# ================== 来源：F1_detect_code0719.py safe_decode_text() ==================
def safe_decode_text(raw):
    return normalize_cv_text(raw)


# ================== 来源：F1_detect_code0719.py safe_repr() ==================
def safe_repr(obj):
    try:
        return repr(obj)
    except Exception:
        return '<repr_failed>'


# ================== 来源：F1_detect_code0719.py build_decode_candidates() ==================
def build_decode_candidates(crop):
    candidates = []
    if crop is None or crop.size == 0:
        return candidates

    try:
        candidates.append(crop)

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        candidates.append(gray)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enh = clahe.apply(gray)
        candidates.append(enh)

        _, th = cv2.threshold(enh, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(th)

        big = cv2.resize(enh, None, fx=1.5, fy=1.5)
        candidates.append(big)

        big_th = cv2.resize(th, None, fx=1.5, fy=1.5)
        candidates.append(big_th)
    except Exception as e:
        rospy.logwarn_throttle(1.0, 'qr preprocess failed: %s' % e)

    return candidates


# ================== 来源：F1_detect_code0719.py decode_crop_best() ==================
def decode_crop_best(crop):
    for img in build_decode_candidates(crop):
        try:
            codes = decode(img)
            for qr in codes:
                data = safe_decode_text(qr.data)
                if data in VALID_CV:
                    return data
        except Exception:
            continue
    return ''


# ================== 来源：F1_detect_code0719.py area_index_from_center() ==================
def area_index_from_center(cx, cy, width, height):
    if cy < height / 2:
        return 0 if cx < width / 2 else 1
    return 2 if cx < width / 2 else 3


# ================== 来源：F1_detect_code0719.py decode_whole_frame_by_position() ==================
def decode_whole_frame_by_position(frame):
    h, w = frame.shape[:2]
    all_data = ['', '', '', '']

    for img in build_decode_candidates(frame):
        try:
            codes = decode(img)
        except Exception:
            continue

        for qr in codes:
            data = safe_decode_text(qr.data)
            if data not in VALID_CV:
                continue

            try:
                rect = qr.rect
                scale_x = float(img.shape[1]) / float(w)
                scale_y = float(img.shape[0]) / float(h)
                cx = (rect.left + rect.width / 2.0) / scale_x
                cy = (rect.top + rect.height / 2.0) / scale_y
                idx = area_index_from_center(cx, cy, w, h)
                if len(data) > len(all_data[idx]):
                    all_data[idx] = data
            except Exception:
                continue

        if all_data != ['', '', '', '']:
            rospy.logwarn('整图兜底识别: %s' % safe_repr(all_data))
            return all_data

    return all_data


# ================== 来源：F1_detect_code0719.py decode_by_direct_quadrants() ==================
def decode_by_direct_quadrants(frame):
    h, w = frame.shape[:2]
    crops = [
        frame[0:h // 2, 0:w // 2],
        frame[0:h // 2, w // 2:w],
        frame[h // 2:h, 0:w // 2],
        frame[h // 2:h, w // 2:w]
    ]
    return [decode_crop_best(crop) for crop in crops]


# ================== 来源：F1_detect_code0719.py task_msg_from_all_data() 裁剪版 ==================
def task_data_from_all_data(all_data):
    """
    all_data: [区域1, 区域2, 区域3, 区域4]

    返回：
    [C, A, B, count, target_area, error_count, img_abc]
    """
    if len(all_data) != 4:
        return None

    all_data = [normalize_cv_text(x) for x in all_data]
    rospy.logwarn('识别到二维码数据: %s' % safe_repr(all_data))

    img_len = [len(x) if x in VALID_CV else 0 for x in all_data]
    max_img = max(img_len)
    if max_img == 0:
        rospy.logwarn('未扫到符合规则的短字符(A/B/C)，继续捕捉下一帧...')
        return None

    if img_len.count(max_img) > 1:
        rospy.logwarn('多个区域出现同长度结果，暂不发布: %s' % safe_repr(all_data))
        return None

    windows_count = img_len.index(max_img)
    img_abc = all_data[windows_count]
    if img_abc not in VALID_CV:
        rospy.logwarn('识别结果不在合法集合内: %s' % safe_repr(img_abc))
        return None

    error_count = 0
    for i, item in enumerate(all_data):
        if item not in NORMAL_CV:
            error_count = i + 1
            rospy.logwarn('窗口 %d 出现错误信息: %s' %
                          (error_count, safe_repr(item)))
            break

    if img_abc == 'ABC':
        task_data = [1, 1, 1, 3, windows_count, error_count, img_abc]
    elif img_abc == 'AB':
        task_data = [0, 1, 1, 2, windows_count, error_count, img_abc]
    elif img_abc == 'BC':
        task_data = [1, 0, 1, 2, windows_count, error_count, img_abc]
    elif img_abc == 'AC':
        task_data = [1, 1, 0, 2, windows_count, error_count, img_abc]
    elif img_abc == 'A':
        task_data = [0, 1, 0, 1, windows_count, error_count, img_abc]
    elif img_abc == 'B':
        task_data = [0, 0, 1, 1, windows_count, error_count, img_abc]
    elif img_abc == 'C':
        task_data = [1, 0, 0, 1, windows_count, error_count, img_abc]
    else:
        return None

    rospy.logwarn('结果为%s，输出为%s' %
                  (img_abc, safe_repr(task_data[0:6])))
    return task_data


# ================== 来源：F1_detect_code0719.py process_frame() 裁剪版 ==================
def process_frame(frame, enable_direct_fallback=True):
    height, width = frame.shape[:2]

    # 保留原程序 5 度旋转修正
    angle = 5
    center = (width // 2, height // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    frame = cv2.warpAffine(frame, matrix, (width, height))

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)
    contours = find_contours(edges)

    contours2 = []
    for contour in contours:
        m1 = cv2.moments(contour)
        if m1['m00'] < min_area or m1['m00'] > max_area:
            continue
        contours2.append(contour)

    sibianxing = []
    for c in contours2:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, approxPolyDP_epslion * peri, True)
        if len(approx) == 4:
            sibianxing.append(approx)

    zhengfangxing = []
    for hull in sibianxing:
        rect = cv2.minAreaRect(hull)
        (x, y), (w, h), rect_angle = rect
        if (w + h) == 0:
            continue
        if abs(w - h) / (w + h) < wh_rate:
            zhengfangxing.append(hull)

    dingweikuang = []
    jieguo = []
    for i in range(len(zhengfangxing) - 1):
        m1 = cv2.moments(zhengfangxing[i])
        if m1['m00'] == 0:
            continue
        cx1 = int(m1['m10'] / m1['m00'])
        cy1 = int(m1['m01'] / m1['m00'])

        for j in range(i + 1, len(zhengfangxing)):
            m2 = cv2.moments(zhengfangxing[j])
            if m2['m00'] == 0:
                continue
            cx2 = int(m2['m10'] / m2['m00'])
            cy2 = int(m2['m01'] / m2['m00'])

            res1 = cv2.pointPolygonTest(zhengfangxing[i], (cx2, cy2), False)
            res2 = cv2.pointPolygonTest(zhengfangxing[j], (cx1, cy1), False)

            if res1 > 0 and res2 > 0:
                distance_ = distance((cx1, cy1), (cx2, cy2))
                if distance_ < min_center_distance:
                    dingweikuang += [zhengfangxing[i], zhengfangxing[j]]
                    jieguo.append(zhengfangxing[i])

    if len(dingweikuang) >= 8 and len(jieguo) >= 4:
        dingweikuang_frame = frame.copy()
        min_x, max_x, min_y, max_y = 100000, 0, 100000, 0
        min_x_index, max_x_index = None, None
        min_y_index, max_y_index = None, None

        for i in range(len(jieguo)):
            for j in range(len(jieguo[i])):
                x = jieguo[i][j][0][0]
                y = jieguo[i][j][0][1]

                if x < min_x:
                    min_x = x
                    min_x_index = [i, j]
                if x > max_x:
                    max_x = x
                    max_x_index = [i, j]
                if y < min_y:
                    min_y = y
                    min_y_index = [i, j]
                if y > max_y:
                    max_y = y
                    max_y_index = [i, j]

        if None not in [min_x_index, max_x_index,
                        min_y_index, max_y_index]:
            x0, y0 = jieguo[min_x_index[0]][min_x_index[1]][0]
            x1, y1 = jieguo[max_x_index[0]][max_x_index[1]][0]
            x2, y2 = jieguo[min_y_index[0]][min_y_index[1]][0]
            x3, y3 = jieguo[max_y_index[0]][max_y_index[1]][0]

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
            m = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped_img = cv2.warpPerspective(dingweikuang_frame, m,
                                              (600, 600))

            aera1_crop = warped_img[0:300, 0:300]
            aera2_crop = warped_img[0:300, 300:600]
            aera3_crop = warped_img[300:600, 0:300]
            aera4_crop = warped_img[300:600, 300:600]

            try:
                cv2.imwrite(os.path.join(SCRIPT_DIR, 'aera1.jpg'),
                            aera1_crop)
                cv2.imwrite(os.path.join(SCRIPT_DIR, 'aera2.jpg'),
                            aera2_crop)
                cv2.imwrite(os.path.join(SCRIPT_DIR, 'aera3.jpg'),
                            aera3_crop)
                cv2.imwrite(os.path.join(SCRIPT_DIR, 'aera4.jpg'),
                            aera4_crop)
            except Exception:
                pass

            all_data = [
                decode_crop_best(aera1_crop),
                decode_crop_best(aera2_crop),
                decode_crop_best(aera3_crop),
                decode_crop_best(aera4_crop)
            ]
            if all_data != ['', '', '', '']:
                return task_data_from_all_data(all_data)

    rospy.loginfo_throttle(
        1.0,
        '定位框数量=%d，候选结果数=%d，暂未完成透视识别' %
        (len(dingweikuang), len(jieguo))
    )

    if enable_direct_fallback:
        all_data = decode_whole_frame_by_position(frame)
        if all_data != ['', '', '', '']:
            return task_data_from_all_data(all_data)

        all_data = decode_by_direct_quadrants(frame)
        if all_data != ['', '', '', '']:
            rospy.logwarn('四象限兜底识别: %s' % safe_repr(all_data))
            return task_data_from_all_data(all_data)

    return None


# ================== 输出逻辑：根据 F1_detect_code0719.py 的 task_data 映射结果打印 ==================
def print_business_result(task_data):
    c_flag = int(task_data[0])
    a_flag = int(task_data[1])
    b_flag = int(task_data[2])
    sample_count = int(task_data[3])
    target_area = int(task_data[4])
    error_count = int(task_data[5])
    img_abc = task_data[6]

    print('')
    print('========== 二维码业务指令 ==========')
    print('二维码内容: %s' % img_abc)
    print('是否前往CAB: C=%s A=%s B=%s' %
          ('是' if c_flag == 1 else '否',
           '是' if a_flag == 1 else '否',
           '是' if b_flag == 1 else '否'))
    print('样品数量: %d' % sample_count)
    print('目标窗口编号: %d' % (target_area + 1))
    if error_count > 0:
        print('异常窗口编号: %d' % error_count)
    else:
        print('异常窗口编号: 无')
    print('===================================')


# ================== 来源：F1_detect_code0719.py main() 裁剪版 ==================
def main():
    rospy.init_node('qr_only_static_0719', anonymous=True)

    stream_url = rospy.get_param(
        '~stream_url',
        'http://192.168.5.4:8080/stream?topic=/camera/rgb/image_raw'
    )
    enable_direct_fallback = bool(rospy.get_param('~enable_direct_fallback',
                                                  True))

    rospy.logwarn('打开摄像头流: %s' % stream_url)
    cap = cv2.VideoCapture(stream_url)

    last_print_data = []
    last_wait_time = 0.0

    while not rospy.is_shutdown():
        # 来源：F1_detect_code0719.py main()，清空少量缓存，防滞后
        for _ in range(2):
            cap.grab()

        hx, frame = cap.retrieve()

        # 来源：F1_detect_code0719.py main()，读取失败后释放并重连
        if hx is False or frame is None:
            rospy.logwarn('read video error, retrying...')
            try:
                cap.release()
            except Exception:
                pass
            rospy.sleep(0.5)
            cap = cv2.VideoCapture(stream_url)
            continue

        try:
            task_data = process_frame(frame, enable_direct_fallback)

            if task_data is None:
                now = time.time()
                if now - last_wait_time > 1.0:
                    print('等待状态: 未识别到有效二维码，小车保持静止...')
                    last_wait_time = now
                continue

            current_data = list(task_data[0:6])
            if current_data != last_print_data:
                print_business_result(task_data)
                last_print_data = current_data

        except Exception as e:
            rospy.logerr_throttle(1.0, 'process frame failed: %s' % e)
            rospy.sleep(0.05)

    try:
        cap.release()
    except Exception:
        pass


if __name__ == '__main__':
    main()