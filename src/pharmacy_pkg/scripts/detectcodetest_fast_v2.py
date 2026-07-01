#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
"""
detectcodetest_fast_v2.py

低延迟二维码识别测试版 v2：
1. 直接订阅 /camera/rgb/image_raw，不走 web_video_server 的 HTTP/MJPEG 流。
2. 只读取最新一帧，queue_size=1，避免 20~30 秒画面缓存延迟。
3. 优先检测识别板上的 4 个大黑框，用大框位置确定 1/2/3/4 区。
4. 每个大框内单独识别二维码，避免整图四象限误判。
5. 增加短窗口投票：连续多帧一致才输出稳定结果。
7. v2允许整图兜底结果参与投票，解决“有稳定兜底但一直不输出”的问题。
6. 不包含任何小车运动控制、电机、/cmd_vel、move_base 代码。
"""

import time
import math
import threading
from collections import deque, Counter

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

try:
    unicode
except NameError:
    unicode = str

VALID_CV = ['A', 'B', 'C', 'AB', 'AC', 'BC', 'ABC']
VALID_CV_U = [unicode(x) for x in VALID_CV]


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


def safe_repr(obj):
    try:
        return repr(obj)
    except Exception:
        return '<repr_failed>'


def task_data_from_text(img_abc, target_area):
    """返回 [C, A, B, sample_count, target_area, error_count, img_abc]。"""
    img_abc = normalize_cv_text(img_abc)
    if img_abc == 'ABC':
        return [1, 1, 1, 3, target_area, 0, img_abc]
    if img_abc == 'AB':
        return [0, 1, 1, 2, target_area, 0, img_abc]
    if img_abc == 'BC':
        return [1, 0, 1, 2, target_area, 0, img_abc]
    if img_abc == 'AC':
        return [1, 1, 0, 2, target_area, 0, img_abc]
    if img_abc == 'A':
        return [0, 1, 0, 1, target_area, 0, img_abc]
    if img_abc == 'B':
        return [0, 0, 1, 1, target_area, 0, img_abc]
    if img_abc == 'C':
        return [1, 0, 0, 1, target_area, 0, img_abc]
    return None


def print_business_result(task_data, source, vote_count):
    c_flag = int(task_data[0])
    a_flag = int(task_data[1])
    b_flag = int(task_data[2])
    sample_count = int(task_data[3])
    target_area = int(task_data[4])
    error_count = int(task_data[5])
    img_abc = task_data[6]

    print('')
    print('========== 稳定二维码业务指令 ==========')
    print('识别来源: %s，投票次数: %d' % (source, vote_count))
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
    print('对应 /cam_return: %s' % safe_repr(task_data[0:6]))
    print('======================================')


def build_decode_candidates(crop):
    candidates = []
    if crop is None or crop.size == 0:
        return candidates

    try:
        candidates.append(crop)

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        candidates.append(gray)

        # 先做轻量二值化，少用复杂增强，减少延迟。
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        _, th = cv2.threshold(blur, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates.append(th)

        # 二维码偏小时，放大后更容易读到。
        big = cv2.resize(gray, None, fx=1.6, fy=1.6)
        candidates.append(big)

        big_th = cv2.resize(th, None, fx=1.6, fy=1.6)
        candidates.append(big_th)
    except Exception:
        pass

    return candidates


def decode_crop_best(crop):
    """返回该区域内最长的合法短字符结果。"""
    best = ''
    for img in build_decode_candidates(crop):
        try:
            codes = decode(img)
        except Exception:
            continue
        for qr in codes:
            data = normalize_cv_text(qr.data)
            if data in VALID_CV and len(data) > len(best):
                best = data
    return best


def contour_boxes_from_mask(mask, frame_w, frame_h):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 3:
        _, contours, _ = result
    else:
        contours, _ = result

    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 2500:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w < 70 or h < 70:
            continue
        if w > frame_w * 0.55 or h > frame_h * 0.65:
            continue
        ratio = float(w) / float(h + 1e-6)
        if ratio < 0.60 or ratio > 1.55:
            continue

        # 黑框是空心框，轮廓面积/外接矩形面积不会太离谱。
        rect_area = float(w * h)
        fill = area / rect_area if rect_area > 0 else 0
        if fill < 0.35:
            continue

        boxes.append((x, y, w, h, area))

    # 去重：同一大框可能被检出多次，中心太近只留面积大的。
    boxes = sorted(boxes, key=lambda b: b[4], reverse=True)
    kept = []
    for b in boxes:
        x, y, w, h, area = b
        cx = x + w / 2.0
        cy = y + h / 2.0
        duplicate = False
        for kb in kept:
            kx, ky, kw, kh, ka = kb
            kcx = kx + kw / 2.0
            kcy = ky + kh / 2.0
            if math.hypot(cx - kcx, cy - kcy) < min(w, h, kw, kh) * 0.45:
                duplicate = True
                break
        if not duplicate:
            kept.append(b)

    return kept


def sort_four_windows(boxes):
    """把 4 个大黑框排序为 [1,2,3,4]。"""
    if len(boxes) < 4:
        return None

    # 取面积最大的 4 个黑框。
    boxes = sorted(boxes, key=lambda b: b[4], reverse=True)[:4]
    boxes = sorted(boxes, key=lambda b: (b[1] + b[3] / 2.0))
    top = sorted(boxes[:2], key=lambda b: (b[0] + b[2] / 2.0))
    bottom = sorted(boxes[2:4], key=lambda b: (b[0] + b[2] / 2.0))
    return top + bottom


def detect_board_windows(frame):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # OTSU 取黑色区域。黑框和二维码都会变白。
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blur, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 横纵闭运算，尽量把大黑框连成稳定外轮廓。
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    boxes = contour_boxes_from_mask(mask, w, h)
    return sort_four_windows(boxes), boxes


def detect_by_windows(frame):
    """基于 4 个大黑框识别，返回 (task_data, all_data, source)。"""
    ordered, boxes = detect_board_windows(frame)
    if ordered is None:
        return None, ['', '', '', ''], 'no_window_boxes_%d' % len(boxes)

    all_data = []
    h, w = frame.shape[:2]
    for idx, b in enumerate(ordered):
        x, y, bw, bh, area = b
        margin = int(max(8, min(bw, bh) * 0.06))
        x0 = max(0, x - margin)
        y0 = max(0, y - margin)
        x1 = min(w, x + bw + margin)
        y1 = min(h, y + bh + margin)
        crop = frame[y0:y1, x0:x1]
        all_data.append(decode_crop_best(crop))

    # 选择最长合法二维码，和原工程规则保持一致。
    lengths = [len(x) if x in VALID_CV else 0 for x in all_data]
    max_len = max(lengths)
    if max_len == 0:
        return None, all_data, 'windows_no_qr'

    # 同长度冲突时暂不输出，避免 AB1/AB3、ABC1/ABC3 跳变。
    if lengths.count(max_len) > 1:
        return None, all_data, 'windows_tie'

    target_area = lengths.index(max_len)
    img_abc = all_data[target_area]
    task_data = task_data_from_text(img_abc, target_area)
    return task_data, all_data, 'window_roi'


def fallback_whole_frame(frame):
    """仅用于观察，不建议作为最终任务依据。"""
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
                if cy < h / 2.0:
                    idx = 0 if cx < w / 2.0 else 1
                else:
                    idx = 2 if cx < w / 2.0 else 3
                if len(data) > len(all_data[idx]):
                    all_data[idx] = data
            except Exception:
                pass
        if all_data != ['', '', '', '']:
            break

    lengths = [len(x) if x in VALID_CV else 0 for x in all_data]
    max_len = max(lengths)
    if max_len == 0 or lengths.count(max_len) > 1:
        return None, all_data, 'fallback_unstable'
    idx = lengths.index(max_len)
    return task_data_from_text(all_data[idx], idx), all_data, 'fallback_observe_only'


class FastQRTester(object):
    def __init__(self):
        rospy.init_node('qr_fast_static_test_0719', anonymous=True)

        if CvBridge is None:
            raise RuntimeError('cv_bridge 不可用。请先确认 ros-melodic-cv-bridge 已安装。')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = 0.0
        self.last_processed_stamp = 0.0

        self.vote_len = int(rospy.get_param('~vote_len', 6))
        self.vote_need = int(rospy.get_param('~vote_need', 3))
        self.process_hz = float(rospy.get_param('~process_hz', 10.0))
        self.enable_fallback_print = bool(rospy.get_param('~enable_fallback_print', True))
        # v2: 允许整图兜底结果参与投票。此前兜底只观察不输出，
        #     你这次日志里一直出现稳定的 ['ABC', '', 'AB', '']，所以需要放开。
        self.allow_fallback_vote = bool(rospy.get_param('~allow_fallback_vote', True))
        self.vote_max_age = float(rospy.get_param('~vote_max_age', 1.8))
        self.vote_ratio = float(rospy.get_param('~vote_ratio', 0.60))

        self.votes = deque(maxlen=self.vote_len)
        self.last_output_key = None
        self.last_wait_print = 0.0

        topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.sub = rospy.Subscriber(
            topic, Image, self.image_callback,
            queue_size=1, buff_size=2 ** 24, tcp_nodelay=True
        )
        rospy.logwarn('快速识别测试启动，直接订阅图像话题: %s' % topic)
        rospy.logwarn('投票参数: 最近%d帧中同一结果达到%d次且占比>=%.2f即输出' %
                      (self.vote_len, self.vote_need, self.vote_ratio))
        rospy.logwarn('兜底投票: allow_fallback_vote=%s, vote_max_age=%.1fs' %
                      (str(self.allow_fallback_vote), self.vote_max_age))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(1.0, 'cv_bridge convert failed: %s' % e)
            return
        with self.lock:
            self.latest_frame = frame
            try:
                self.latest_stamp = msg.header.stamp.to_sec()
            except Exception:
                self.latest_stamp = time.time()

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None, 0.0
            return self.latest_frame.copy(), self.latest_stamp

    def add_vote_and_maybe_output(self, task_data, all_data, source):
        key = (task_data[6], int(task_data[4]))
        now = time.time()
        self.votes.append((key, task_data, source, now))

        # 只统计最近 vote_max_age 秒内的结果，避免几十秒前的旧票污染当前识别。
        fresh = [v for v in self.votes if now - v[3] <= self.vote_max_age]
        self.votes = deque(fresh, maxlen=self.vote_len)

        counter = Counter([v[0] for v in self.votes])
        if not counter:
            return False
        best_key, best_count = counter.most_common(1)[0]
        total = len(self.votes)
        ratio = float(best_count) / float(total) if total > 0 else 0.0

        rospy.logwarn_throttle(
            0.35,
            '候选=%s all_data=%s source=%s vote=%d/%d ratio=%.2f need=%d,%.2f' %
            (str(key), safe_repr(all_data), source, best_count, total,
             ratio, self.vote_need, self.vote_ratio)
        )

        if (best_key == key and best_count >= self.vote_need and
                ratio >= self.vote_ratio):
            if self.last_output_key != best_key:
                print_business_result(task_data, source, best_count)
                self.last_output_key = best_key
            return True
        return False

    def run(self):
        rate = rospy.Rate(self.process_hz)
        while not rospy.is_shutdown():
            frame, stamp = self.get_latest_frame()
            if frame is None:
                rospy.loginfo_throttle(1.0, '等待图像帧，小车保持静止...')
                rate.sleep()
                continue

            # 同一帧不重复处理。
            if stamp == self.last_processed_stamp:
                rate.sleep()
                continue
            self.last_processed_stamp = stamp

            task_data, all_data, source = detect_by_windows(frame)

            if task_data is not None:
                self.add_vote_and_maybe_output(task_data, all_data, source)
                rate.sleep()
                continue

            if self.enable_fallback_print:
                fb_task, fb_all, fb_source = fallback_whole_frame(frame)
                if fb_task is not None:
                    if self.allow_fallback_vote:
                        # v2关键改动：兜底结果不再只观察。
                        # 只有经过 add_vote_and_maybe_output 的短时投票稳定后才真正输出。
                        self.add_vote_and_maybe_output(
                            fb_task, fb_all, 'fallback_vote'
                        )
                    else:
                        rospy.logwarn_throttle(
                            0.8,
                            '低可信兜底观察结果=%s all_data=%s，暂不作为稳定输出' %
                            (safe_repr(fb_task[0:6]), safe_repr(fb_all))
                        )
                else:
                    rospy.loginfo_throttle(
                        1.0,
                        '等待稳定识别: window=%s source=%s fallback=%s' %
                        (safe_repr(all_data), source, safe_repr(fb_all))
                    )
            else:
                now = time.time()
                if now - self.last_wait_print > 1.0:
                    print('等待状态: 未识别到稳定二维码，小车保持静止...')
                    self.last_wait_print = now

            rate.sleep()


if __name__ == '__main__':
    try:
        FastQRTester().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print('程序异常: %s' % e)
