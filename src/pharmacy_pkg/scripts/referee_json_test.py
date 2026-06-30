#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Minimal protocol test for 2026 CRAIC smart pharmacy referee software.
Run this on the robot/VM after the referee software has started listening.
"""
import json
import socket
import time

REFEREE_IP = '192.168.5.2'
REFEREE_PORT = 8888


def send_json_line(sock, payload):
    # The referee parses one JSON object per line. Do not remove '\n'.
    msg = json.dumps(payload, ensure_ascii=True) + '\n'
    sock.sendall(msg.encode('utf-8'))
    print('[->]', msg.strip())


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    sock.connect((REFEREE_IP, REFEREE_PORT))
    print('[+] connected to referee %s:%d' % (REFEREE_IP, REFEREE_PORT))

    # Do NOT include CRASH unless a collision really happened.
    samples = [
        {'task': 'A', 'speed': 0.20, 'odom': [0.603, 2.620], 'CV1': 'ABC'},
        {'task': 'B', 'speed': 0.25, 'odom': [1.284, 3.000], 'CV1': 'ABC'},
        {'task': '1', 'speed': 0.18, 'odom': [-1.734, 2.325], 'CV2': 'A'},
    ]

    for i in range(30):
        send_json_line(sock, samples[i % len(samples)])
        time.sleep(0.5)

    sock.close()


if __name__ == '__main__':
    main()
