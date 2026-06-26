#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time

SERVER_IP = '192.168.12.159'
SERVER_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("正在连接裁判软件 %s:%d ..." % (SERVER_IP, SERVER_PORT))
sock.connect((SERVER_IP, SERVER_PORT))
print("连接成功")

count = 0

while True:
    msg = "hello referee %d\r\n" % count
    sock.sendall(msg.encode('utf-8'))
    print("已发送:", msg.strip())
    count += 1
    time.sleep(1)
