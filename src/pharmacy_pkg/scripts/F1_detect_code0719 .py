#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
F1_detect_code0719.py

BUGFIX_V4_FORCE_MULTI_QR_20260701

0719 二维码识别低延迟修正版：
1. 直接订阅 /camera/rgb/image_raw，不走 web_video_server，避免 20~30 秒缓存延迟。
2. 保留 /cam_return 和 /vision_report 输出接口。
3. window_roi 成功时优先使用四个大窗口框排序结果。
4. fallback / quadrant 只作为兜底；兜底至少识别到 2 个合法二维码才参与投票。
5. fallback 横向分界默认 0.62，避免识别板偏在画面一侧时把 1/3 误判成 2/4。
6. 稳定投票后锁定并重复发布，方便主控 count==9/count==10 接收。

发布：
  /cam_return     Int32MultiArray: [C, A, B, count, target_area, error_count]
  /vision_report  String: A/B/C/AB/AC/BC/ABC

/cam_return 数据格式：
  target_area: 0->1号区, 1->2号区, 2->3号区, 3->4号区
"""

import os
import time
import threading
from collections import Counter

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String

os.system('export LANG=zh_CN.UTF-8')

try:
    unicode
except NameError:
    unicode = str


VERSION_TAG = 'BUGFIX_V4_FORCE_MULTI_QR_20260701'

VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']
NORMAL_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC', '']
VALID_CV_U = [unicode(x) for x in VALID_CV]

SCRIPT_DIR = '/home/EPRobot/robot_ws/src/pharmacy_pkg/scripts'

pub_flag = None
pub_vision_str = None

bridge = None
frame_lock = threading.Lock()
latest_frame = None
latest_frame_seq = 0
processed_frame_seq = -1

vote_history = []

stable_data = None
stable_cv = ''
stable_source = ''
stable_start_time = 0.0
last_publish_time = 0.0
last_print_data = None

# 参数默认值，会在 main() 中由 rosparam 覆盖
ROTATE_ANGLE = 5.0
PROCESS_HZ = 10.0
STABLE_VOTE_NEED = 3
STABLE_RATIO = 0.60
VOTE_MAX_AGE = 1.8
PUBLISH_HOLD_SEC = 8.0
REPUBLISH_INTERVAL = 0.2
ALLOW_FALLBACK_VOTE = True
FALLBACK_MIN_NONEMPTY = 2
FALLBACK_X_SPLIT_RATIO = 0.62
FALLBACK_Y_SPLIT_RATIO = 0.50
DEBUG_LOG = True


def safe_repr(obj):
    try:
        return repr(obj)
    except Exception:
        return '<repr_failed>'


def normalize_cv_text(value):
    """把 pyzbar 结果统一为 Python2 下安全的 ASCII str。"""
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


def nonempty_count(all_data):
    count = 0
    for item in all_data:
        if normalize_cv_text(item) in VALID_CV:
            count += 1
    return count


def build_decode_candidates(crop):
    """
    保留原工程中“原图/灰度/CLAHE/二值化/放大”的二维码增强思路。
    """
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

        _, th = cv2.threshold(
            enh, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        candidates.append(th)

        big = cv2.resize(enh, None, fx=1.5, fy=1.5)
        candidates.append(big)

        big_th = cv2.resize(th, None, fx=1.5, fy=1.5)
        candidates.append(big_th)

    except Exception as e:
        rospy.logwarn_throttle(1.0, 'qr preprocess failed: %s' % e)

    return candidates


def decode_crop_best(crop):
    for img in build_decode_candidates(crop):
        try:
            codes = decode(img)
        except Exception:
            continue

        for qr in codes:
            data = normalize_cv_text(qr.data)
            if data in VALID_CV:
                return data

    return ''


def area_index_from_center(cx, cy, width, height):
    """
    fallback 兜底区域划分。
    注意：不再使用画面正中 0.50 作为左右分界，默认改为 0.62。
    这是为了适配当前识别板整体偏在画面左/右一侧的情况，
    避免把真实 1/3 误判成 2/4。
    """
    split_x = float(width) * float(FALLBACK_X_SPLIT_RATIO)
    split_y = float(height) * float(FALLBACK_Y_SPLIT_RATIO)

    if cy < split_y:
        return 0 if cx < split_x else 1
    return 2 if cx < split_x else 3


def rotate_frame(frame):
    if abs(ROTATE_ANGLE) < 0.01:
        return frame

    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    matrix = cv2.getRotationMatrix2D(center, ROTATE_ANGLE, 1.0)
    return cv2.warpAffine(frame, matrix, (w, h))


def decode_whole_frame_by_position(frame):
    """
    整图直接 pyzbar 解码，再按二维码中心位置映射到 1/2/3/4 区。
    该结果不直接发布，必须经过 fallback_min_nonempty 和投票。
    """
    h, w = frame.shape[:2]
    all_data = ['', '', '', '']

    for img in build_decode_candidates(frame):
        try:
            codes = decode(img)
        except Exception:
            continue

        for qr in codes:
            data = normalize_cv_text(qr.data)
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
            return all_data

    return all_data


def decode_by_direct_quadrants(frame):
    """
    最后兜底：整图定位不到时，直接按画面四象限裁剪识别。
    该结果也必须经过 fallback_min_nonempty 和投票。
    """
    h, w = frame.shape[:2]

    sx = int(float(w) * float(FALLBACK_X_SPLIT_RATIO))
    sy = int(float(h) * float(FALLBACK_Y_SPLIT_RATIO))
    sx = max(1, min(w - 1, sx))
    sy = max(1, min(h - 1, sy))

    crops = [
        frame[0:sy, 0:sx],
        frame[0:sy, sx:w],
        frame[sy:h, 0:sx],
        frame[sy:h, sx:w]
    ]

    return [decode_crop_best(crop) for crop in crops]


def find_window_boxes(frame):
    """
    尝试识别四个大窗口黑框。
    V4 改动：RETR_LIST 比 RETR_EXTERNAL 更容易在屏幕/边框干扰下找到内部窗口框。
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    try:
        # 黑色边框在阈值反相图里变成白色。
        _, th = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        kernel = np.ones((5, 5), np.uint8)
        th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)

        result = cv2.findContours(th, cv2.RETR_LIST,
                                  cv2.CHAIN_APPROX_SIMPLE)
        if len(result) == 3:
            _, contours, hierarchy = result
        else:
            contours, hierarchy = result
    except Exception:
        return []

    boxes = []
    min_area = max(2500.0, float(w * h) * 0.012)
    max_area = float(w * h) * 0.35

    for c in contours:
        area = abs(cv2.contourArea(c))
        if area < min_area or area > max_area:
            continue

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.035 * peri, True)
        if len(approx) < 4:
            continue

        x, y, bw, bh = cv2.boundingRect(c)
        if bw <= 0 or bh <= 0:
            continue

        # 过滤贴边的大屏幕外框/画面黑边。
        if x <= 2 or y <= 2 or x + bw >= w - 2 or y + bh >= h - 2:
            continue

        ratio = float(bw) / float(bh)
        if ratio < 0.50 or ratio > 1.90:
            continue

        # 过滤二维码内部小定位块、文字、小噪点。
        if bw < w * 0.13 or bh < h * 0.13:
            continue

        boxes.append((x, y, bw, bh, area))

    # 去重：同一个窗口可能检测到内外两层框，中心接近时保留面积大的。
    boxes = sorted(boxes, key=lambda b: b[4], reverse=True)
    filtered = []
    for b in boxes:
        x, y, bw, bh, area = b
        cx = x + bw / 2.0
        cy = y + bh / 2.0
        duplicate = False

        for fb in filtered:
            fx, fy, fw, fh, farea = fb
            fcx = fx + fw / 2.0
            fcy = fy + fh / 2.0
            if abs(cx - fcx) < min(bw, fw) * 0.45 and \
               abs(cy - fcy) < min(bh, fh) * 0.45:
                duplicate = True
                break

        if not duplicate:
            filtered.append(b)

    if len(filtered) < 4:
        return []

    # 选面积最大的 4 个，再按上/下、左/右排序成 1,2,3,4。
    selected = sorted(filtered[:4], key=lambda b: (b[1] + b[3] / 2.0))
    top = sorted(selected[:2], key=lambda b: b[0] + b[2] / 2.0)
    bottom = sorted(selected[2:4], key=lambda b: b[0] + b[2] / 2.0)
    ordered = top + bottom

    return ordered


def decode_by_window_boxes(frame):
    boxes = find_window_boxes(frame)
    if len(boxes) < 4:
        return ['', '', '', ''], 'no_window_boxes_%d' % len(boxes)

    h, w = frame.shape[:2]
    all_data = []

    for box in boxes:
        x, y, bw, bh, area = box

        # 向内缩一点，避开粗黑框。
        pad_x = int(bw * 0.06)
        pad_y = int(bh * 0.06)
        x1 = max(0, x + pad_x)
        y1 = max(0, y + pad_y)
        x2 = min(w, x + bw - pad_x)
        y2 = min(h, y + bh - pad_y)

        crop = frame[y1:y2, x1:x2]
        all_data.append(decode_crop_best(crop))

    return all_data, 'window_roi'


def task_data_from_all_data(all_data):
    """
    all_data: [区域1, 区域2, 区域3, 区域4]
    返回：
      (data_list, img_abc, target_area) 或 None
    data_list = [C, A, B, sample_count, target_area, error_count]
    """
    if len(all_data) != 4:
        return None

    all_data = [normalize_cv_text(x) for x in all_data]
    img_len = [len(x) if x in VALID_CV else 0 for x in all_data]
    max_img = max(img_len)

    if max_img == 0:
        return None

    # 最长结果必须唯一，防止多个区域同长度时目标区乱跳。
    if img_len.count(max_img) > 1:
        if DEBUG_LOG:
            rospy.logwarn_throttle(
                0.5,
                '多个区域出现同长度结果，暂不计票: %s' % safe_repr(all_data)
            )
        return None

    target_area = img_len.index(max_img)
    img_abc = all_data[target_area]

    if img_abc not in VALID_CV:
        return None

    error_count = 0
    for i, item in enumerate(all_data):
        if item not in NORMAL_CV:
            error_count = i + 1
            break

    if img_abc == 'ABC':
        data_list = [1, 1, 1, 3, target_area, error_count]
    elif img_abc == 'AB':
        data_list = [0, 1, 1, 2, target_area, error_count]
    elif img_abc == 'BC':
        data_list = [1, 0, 1, 2, target_area, error_count]
    elif img_abc == 'AC':
        data_list = [1, 1, 0, 2, target_area, error_count]
    elif img_abc == 'A':
        data_list = [0, 1, 0, 1, target_area, error_count]
    elif img_abc == 'B':
        data_list = [0, 0, 1, 1, target_area, error_count]
    elif img_abc == 'C':
        data_list = [1, 0, 0, 1, target_area, error_count]
    else:
        return None

    return data_list, img_abc, target_area


def add_vote(data_list, img_abc, source, all_data):
    global vote_history

    now = time.time()
    data_tuple = tuple([int(x) for x in data_list])

    vote_history.append((now, data_tuple, img_abc, source, list(all_data)))
    vote_history = [
        item for item in vote_history
        if now - item[0] <= VOTE_MAX_AGE
    ]


def get_voted_result():
    """
    返回稳定投票结果：
      (data_list, img_abc, source, vote_count, total_count, ratio, all_data)
    不稳定时返回 None。
    """
    if len(vote_history) == 0:
        return None

    keys = [item[1] for item in vote_history]
    counter = Counter(keys)
    best_key, best_count = counter.most_common(1)[0]
    total_count = len(keys)
    ratio = float(best_count) / float(total_count)

    if best_count < STABLE_VOTE_NEED or ratio < STABLE_RATIO:
        return None

    best_item = None
    for item in reversed(vote_history):
        if item[1] == best_key:
            best_item = item
            break

    if best_item is None:
        return None

    _, data_tuple, img_abc, source, all_data = best_item
    return list(data_tuple), img_abc, source, best_count, total_count, ratio, all_data


def publish_outputs(data_list, img_abc, source):
    """
    同时发布 /cam_return 和 /vision_report。
    """
    msg = Int32MultiArray()
    msg.data = list(data_list)
    pub_flag.publish(msg)
    pub_vision_str.publish(img_abc)

    rospy.logwarn(
        'publish /cam_return: %s source=%s CV=%s target=%d' %
        (safe_repr(list(data_list)), source, img_abc, int(data_list[4]) + 1)
    )


def print_stable_once(data_list, img_abc, source, vote_count):
    global last_print_data

    if last_print_data == list(data_list):
        return

    last_print_data = list(data_list)
    rospy.logwarn(
        '稳定二维码结果: source=%s vote=%d CV=%s /cam_return=%s target=%d' %
        (source, vote_count, img_abc, safe_repr(list(data_list)),
         int(data_list[4]) + 1)
    )


def handle_stable_candidate(voted_result):
    """
    稳定结果锁定：
    1. 首次稳定后锁定 publish_hold_sec 秒；
    2. 锁定期间即使出现短暂误识别，也继续发布锁定结果；
    3. 锁定过期后允许切换到新的稳定结果。
    """
    global stable_data, stable_cv, stable_source
    global stable_start_time, last_publish_time

    now = time.time()

    if voted_result is not None:
        data_list, img_abc, source, vote_count, total_count, ratio, all_data = voted_result

        if DEBUG_LOG:
            rospy.logwarn_throttle(
                0.2,
                '候选=(%s,%d) all_data=%s source=%s vote=%d/%d ratio=%.2f need=%d,%.2f' %
                (img_abc, int(data_list[4]), safe_repr(all_data), source,
                 vote_count, total_count, ratio, STABLE_VOTE_NEED, STABLE_RATIO)
            )

        can_update = False
        if stable_data is None:
            can_update = True
        elif now - stable_start_time > PUBLISH_HOLD_SEC:
            can_update = True
        elif list(stable_data) == list(data_list):
            can_update = True

        if can_update:
            if stable_data is None or list(stable_data) != list(data_list):
                stable_data = list(data_list)
                stable_cv = img_abc
                stable_source = source
                stable_start_time = now
                print_stable_once(stable_data, stable_cv, stable_source,
                                  vote_count)

    # 锁定窗口内重复发布，方便主控 count==9/count==10 接收。
    if stable_data is not None:
        if now - stable_start_time <= PUBLISH_HOLD_SEC:
            if now - last_publish_time >= REPUBLISH_INTERVAL:
                publish_outputs(stable_data, stable_cv, stable_source)
                last_publish_time = now
        else:
            stable_data = None
            stable_cv = ''
            stable_source = ''
            stable_start_time = 0.0


def process_frame(frame):
    frame = rotate_frame(frame)

    # 优先使用窗口框 ROI；如果能稳定找到四个窗口，单码也可信。
    all_data, source = decode_by_window_boxes(frame)
    result = task_data_from_all_data(all_data)

    if result is not None:
        data_list, img_abc, target_area = result
        add_vote(data_list, img_abc, source, all_data)
        return

    if ALLOW_FALLBACK_VOTE:
        # fallback 只看到单个码时不计票，避免 target 2/4 乱跳。
        all_data = decode_whole_frame_by_position(frame)
        if nonempty_count(all_data) >= FALLBACK_MIN_NONEMPTY:
            result = task_data_from_all_data(all_data)
            if result is not None:
                data_list, img_abc, target_area = result
                add_vote(data_list, img_abc, 'fallback_vote', all_data)
                return
        elif nonempty_count(all_data) > 0:
            rospy.logwarn_throttle(
                0.5,
                'fallback 单码结果暂不计票: %s' % safe_repr(all_data)
            )

        all_data = decode_by_direct_quadrants(frame)
        if nonempty_count(all_data) >= FALLBACK_MIN_NONEMPTY:
            result = task_data_from_all_data(all_data)
            if result is not None:
                data_list, img_abc, target_area = result
                add_vote(data_list, img_abc, 'quadrant_vote', all_data)
                return
        elif nonempty_count(all_data) > 0:
            rospy.logwarn_throttle(
                0.5,
                'quadrant 单码结果暂不计票: %s' % safe_repr(all_data)
            )

    if DEBUG_LOG:
        rospy.loginfo_throttle(
            1.0,
            '等待稳定识别: window=%s source=%s fallback_min=%d split=%.2f' %
            (safe_repr(all_data), source, FALLBACK_MIN_NONEMPTY,
             FALLBACK_X_SPLIT_RATIO)
        )


def image_callback(msg):
    global latest_frame, latest_frame_seq

    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logwarn_throttle(1.0, 'cv_bridge convert failed: %s' % e)
        return

    with frame_lock:
        latest_frame = frame
        latest_frame_seq += 1


def main():
    global pub_flag, pub_vision_str, bridge
    global ROTATE_ANGLE, PROCESS_HZ, STABLE_VOTE_NEED, STABLE_RATIO
    global VOTE_MAX_AGE, PUBLISH_HOLD_SEC, REPUBLISH_INTERVAL
    global ALLOW_FALLBACK_VOTE, FALLBACK_MIN_NONEMPTY
    global FALLBACK_X_SPLIT_RATIO, FALLBACK_Y_SPLIT_RATIO
    global DEBUG_LOG, processed_frame_seq

    rospy.init_node('detect_abc', anonymous=True)

    pub_flag = rospy.Publisher('/cam_return', Int32MultiArray, queue_size=10)
    pub_vision_str = rospy.Publisher('/vision_report', String, queue_size=10)

    bridge = CvBridge()

    image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')

    # 兼容旧 launch 中的 stream_url 参数；低延迟版不再使用它。
    old_stream_url = rospy.get_param('~stream_url', '')
    if old_stream_url:
        rospy.logwarn('低延迟版忽略 stream_url，改为订阅图像话题: %s' % image_topic)

    ROTATE_ANGLE = float(rospy.get_param('~rotate_angle', 5.0))
    PROCESS_HZ = float(rospy.get_param('~process_hz', 10.0))
    STABLE_VOTE_NEED = int(rospy.get_param('~stable_vote_need', 3))
    STABLE_RATIO = float(rospy.get_param('~stable_ratio', 0.60))
    VOTE_MAX_AGE = float(rospy.get_param('~vote_max_age', 1.8))
    PUBLISH_HOLD_SEC = float(rospy.get_param('~publish_hold_sec', 8.0))
    REPUBLISH_INTERVAL = float(rospy.get_param('~republish_interval', 0.2))
    ALLOW_FALLBACK_VOTE = bool(rospy.get_param('~allow_fallback_vote', True))
    FALLBACK_MIN_NONEMPTY = int(rospy.get_param('~fallback_min_nonempty', 2))
    FALLBACK_X_SPLIT_RATIO = float(rospy.get_param('~fallback_x_split_ratio', 0.62))
    FALLBACK_Y_SPLIT_RATIO = float(rospy.get_param('~fallback_y_split_ratio', 0.50))
    DEBUG_LOG = bool(rospy.get_param('~debug_log', True))

    rospy.logwarn('当前识别代码版本: %s' % VERSION_TAG)
    rospy.logwarn('0719低延迟二维码识别启动，订阅: %s' % image_topic)
    rospy.logwarn(
        '投票参数: need=%d ratio=%.2f max_age=%.1f hold=%.1f interval=%.1f fallback_min=%d split=%.2f' %
        (STABLE_VOTE_NEED, STABLE_RATIO, VOTE_MAX_AGE,
         PUBLISH_HOLD_SEC, REPUBLISH_INTERVAL,
         FALLBACK_MIN_NONEMPTY, FALLBACK_X_SPLIT_RATIO)
    )

    rospy.Subscriber(
        image_topic, Image, image_callback,
        queue_size=1, buff_size=2 ** 24, tcp_nodelay=True
    )

    rate = rospy.Rate(PROCESS_HZ)
    last_no_frame_log = 0.0

    while not rospy.is_shutdown():
        frame_to_process = None
        seq_to_process = None

        with frame_lock:
            if latest_frame is not None and latest_frame_seq != processed_frame_seq:
                frame_to_process = latest_frame.copy()
                seq_to_process = latest_frame_seq

        if frame_to_process is None:
            now = time.time()
            if now - last_no_frame_log > 1.0:
                rospy.logwarn('等待 /camera/rgb/image_raw 图像帧...')
                last_no_frame_log = now

            # 即使暂时没有新图像，也继续发布锁定结果，防止主控错过。
            handle_stable_candidate(None)
            rate.sleep()
            continue

        processed_frame_seq = seq_to_process

        try:
            process_frame(frame_to_process)
            voted_result = get_voted_result()
            handle_stable_candidate(voted_result)
        except Exception as e:
            rospy.logerr_throttle(1.0, 'process frame failed: %s' % e)

        rate.sleep()


if __name__ == '__main__':
    main()
