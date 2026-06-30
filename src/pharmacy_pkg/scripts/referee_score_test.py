#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CRAIC 智慧药房裁判软件通信加分逻辑测试脚本

用途：
1. 验证裁判软件是否解析 speed / CV1 / task / odom / CV2 字段
2. 每个阶段持续发送 JSON + 换行，方便观察裁判软件右侧统计项是否加分
3. 不发送 CRASH 字段，避免误触发碰撞扣分

运行位置：
小车终端 / scripts 目录或任意能连到 192.168.5.2:8888 的终端

运行方式：
python referee_score_test.py

建议：
先打开裁判软件，点击“重置”，再点击“开始比赛”，然后运行本脚本。
"""

from __future__ import print_function

import json
import socket
import time
import sys


# ===================== 基本配置 =====================

REFEREE_IP = "192.168.5.2"
REFEREE_PORT = 8888

# 每个测试阶段持续发送多少秒
PHASE_SECONDS = 8

# 发送周期，0.5 秒一次比较接近真实小车上报
SEND_INTERVAL = 0.5

# 根据你当前 F1_yaofang0719.py 里的 waypoint 整理
POINTS = {
    "C": (1.284, 2.160),
    "A": (0.603, 2.620),
    "B": (1.284, 3.000),
    "4": (-1.075, 0.898),
    "3": (-1.731, 1.296),
    "2": (-1.080, 1.715),
    "1": (-1.734, 2.325),
    "START": (0.002, -0.056),
    "CV2_AREA": (-0.496, 3.921),
    "CV1_AREA": (0.866, -0.002),
}

VALID_CV = ["A", "B", "C", "AB", "AC", "BC", "ABC"]


# ===================== 兼容 Python2 / Python3 输入 =====================

try:
    input_func = raw_input
except NameError:
    input_func = input


# ===================== 通信函数 =====================

def connect_referee():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    print("[*] 正在连接裁判软件 {}:{} ...".format(REFEREE_IP, REFEREE_PORT))
    sock.connect((REFEREE_IP, REFEREE_PORT))
    sock.settimeout(1.0)
    print("[+] TCP 连接成功")
    return sock


def send_json_line(sock, data):
    """
    裁判软件目前已验证的方式：
    每次发送一个 JSON 对象，末尾加换行符 \n
    """
    msg = json.dumps(data, ensure_ascii=True) + "\n"
    sock.sendall(msg.encode("utf-8"))
    print("[->] {}".format(msg.strip()))


def build_packet(task="default", speed=0.20, odom=None, cv1=None, cv2=None):
    """
    注意：
    1. odom 使用 [x, y] 数组，不使用 {"x":..., "y":...}
    2. 不默认发送 CRASH 字段
    3. CV1 / CV2 只有在需要测试时才放进去
    """
    if odom is None:
        odom = POINTS["START"]

    packet = {
        "task": str(task),
        "speed": float(speed),
        "odom": [round(float(odom[0]), 3), round(float(odom[1]), 3)]
    }

    if cv1 in VALID_CV:
        packet["CV1"] = cv1

    if cv2 in VALID_CV or cv2 == "WAIT":
        packet["CV2"] = cv2

    return packet


def run_phase(sock, title, packet_func, seconds=PHASE_SECONDS):
    print("\n==============================")
    print("测试阶段：{}".format(title))
    print("==============================")
    print("持续发送 {} 秒，每 {} 秒一包。".format(seconds, SEND_INTERVAL))
    print("请观察裁判软件右侧对应统计项是否变化。")
    print("")

    start = time.time()
    count = 0

    while time.time() - start < seconds:
        packet = packet_func(count)
        send_json_line(sock, packet)
        count += 1

        # 读取裁判软件可能返回的信息；没有返回也正常
        try:
            data = sock.recv(1024)
            if data:
                try:
                    print("[<-] {}".format(data.decode("utf-8", "ignore")))
                except TypeError:
                    print("[<-] {}".format(data.decode("utf-8")))
        except socket.timeout:
            pass
        except Exception:
            pass

        time.sleep(SEND_INTERVAL)

    print("[阶段结束] {}".format(title))


# ===================== 测试流程 =====================

def wait_user(message):
    print("")
    input_func(message + "  按回车继续...")


def test_speed(sock):
    speeds = [0.18, 0.20, 0.25, 0.30]
    def packet_func(i):
        return build_packet(
            task="default",
            speed=speeds[i % len(speeds)],
            odom=POINTS["START"]
        )
    run_phase(sock, "1. speed 字段测试：当前速度应循环变化", packet_func)


def test_cv1(sock):
    cvs = ["A", "B", "C", "AB", "AC", "BC", "ABC"]
    def packet_func(i):
        return build_packet(
            task="default",
            speed=0.20,
            odom=POINTS["CV1_AREA"],
            cv1=cvs[i % len(cvs)]
        )
    run_phase(sock, "2. CV1 字段测试：观察 CV1有效信息/识别板正确", packet_func)


def test_task_and_odom(sock):
    task_order = ["A", "B", "C", "1", "2", "3", "4"]

    for task in task_order:
        wait_user("准备测试 task='{}' + odom={}。可以在裁判软件上点击对应点位/任务".format(
            task, POINTS[task]
        ))

        def packet_func(i, task=task):
            return build_packet(
                task=task,
                speed=0.20,
                odom=POINTS[task]
            )

        run_phase(
            sock,
            "3. task + odom 测试：当前任务={}，坐标={}".format(task, POINTS[task]),
            packet_func,
            seconds=6
        )


def test_cv2(sock):
    print("")
    print("CV2 交叉核对通常需要裁判软件当前任务/点击动作配合。")
    print("本阶段会依次发送 task=1/2/3/4 与 CV2=A/B/C/AB/AC/BC/ABC。")
    print("如果裁判软件没有进入对应环节，不加分也可能是正常的。")
    wait_user("确认开始 CV2 测试")

    pairs = [
        ("1", "A"),
        ("2", "B"),
        ("3", "C"),
        ("4", "ABC"),
        ("1", "AB"),
        ("2", "AC"),
        ("3", "BC"),
    ]

    def packet_func(i):
        task, cv2 = pairs[i % len(pairs)]
        return build_packet(
            task=task,
            speed=0.18,
            odom=POINTS[task],
            cv2=cv2
        )

    run_phase(sock, "4. CV2 字段测试：观察 CV2交叉核对", packet_func, seconds=12)


def test_combined_realistic(sock):
    print("")
    print("综合流程会模拟真实小车一圈：CV1识别 -> A/B/C -> CV2区 -> 数字区。")
    wait_user("确认开始综合流程测试")

    sequence = [
        ("到达识别板1，发送 CV1=ABC", build_packet("default", 0.18, POINTS["CV1_AREA"], cv1="ABC")),
        ("执行任务 A", build_packet("A", 0.20, POINTS["A"], cv1="ABC")),
        ("执行任务 B", build_packet("B", 0.20, POINTS["B"], cv1="ABC")),
        ("执行任务 C", build_packet("C", 0.20, POINTS["C"], cv1="ABC")),
        ("到达识别板2区域，CV2=WAIT", build_packet("default", 0.18, POINTS["CV2_AREA"], cv1="ABC", cv2="WAIT")),
        ("送往数字区 1，CV2=A", build_packet("1", 0.18, POINTS["1"], cv1="ABC", cv2="A")),
    ]

    for title, packet in sequence:
        print("\n--- {} ---".format(title))
        end_time = time.time() + 4
        while time.time() < end_time:
            send_json_line(sock, packet)
            time.sleep(SEND_INTERVAL)


def main():
    print("CRAIC 智慧药房裁判软件加分逻辑测试脚本")
    print("目标地址：{}:{}".format(REFEREE_IP, REFEREE_PORT))
    print("")
    print("使用前建议：")
    print("1. 打开裁判软件")
    print("2. 选择/确认监听 IP 为 192.168.5.2")
    print("3. 点击“重置”")
    print("4. 点击“开始比赛”")
    print("5. 再运行本脚本")
    print("")

    try:
        sock = connect_referee()
    except Exception as e:
        print("[-] 连接失败：{}".format(e))
        print("请检查：裁判软件是否打开、端口是否 8888、防火墙是否拦截、IP 是否为 192.168.5.2")
        sys.exit(1)

    try:
        wait_user("连接成功。请确认裁判软件已经点击“开始比赛”")

        while True:
            print("")
            print("请选择测试项目：")
            print("1 - 测试 speed 当前速度")
            print("2 - 测试 CV1 视觉识别结果")
            print("3 - 测试 task + odom 任务与坐标")
            print("4 - 测试 CV2 交叉核对")
            print("5 - 综合模拟一圈流程")
            print("0 - 退出")
            choice = input_func("输入编号：").strip()

            if choice == "1":
                test_speed(sock)
            elif choice == "2":
                test_cv1(sock)
            elif choice == "3":
                test_task_and_odom(sock)
            elif choice == "4":
                test_cv2(sock)
            elif choice == "5":
                test_combined_realistic(sock)
            elif choice == "0":
                break
            else:
                print("无效输入，请重新选择。")

    except KeyboardInterrupt:
        print("\n[!] 用户中断")
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print("[*] 连接已关闭")


if __name__ == "__main__":
    main()
