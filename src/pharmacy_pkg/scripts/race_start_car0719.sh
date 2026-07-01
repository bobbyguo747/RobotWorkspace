#!/bin/bash
# 小车端一键启动：初始化 + 相机 + web视频 + 识别 + 裁判通信 + 主控
set -e

source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 启动前检查三个 Python 节点是否真的在 scripts 目录且可执行，
# 避免 roslaunch 报 Cannot locate node。
for f in F1_detect_code0719.py F1_yaofang0719.py F1_referee_report0719.py; do
  if [ ! -f "$SCRIPT_DIR/$f" ]; then
    echo "[ERROR] 缺少 $SCRIPT_DIR/$f"
    echo "请先把新版文件复制到 ~/robot_ws/src/pharmacy_pkg/scripts/"
    exit 1
  fi
  chmod +x "$SCRIPT_DIR/$f"
done

rospack profile >/dev/null 2>&1 || true


# 默认双模式通信：
# 1) 小车主动连接裁判电脑 192.168.5.2:8888
# 2) 小车同时监听 0.0.0.0:8888，允许裁判软件主动连接小车 192.168.5.4:8888
roslaunch pharmacy_pkg f1_race_all0719.launch \
  referee_ip:=192.168.5.2 \
  referee_port:=8888 \
  listen_ip:=0.0.0.0 \
  listen_port:=8888 \
  enable_client:=true \
  enable_server:=true \
  cv1_wait_timeout:=5.0 \
  cv1_default_commit_delay:=3.0 \
  cv2_wait_time:=2.5
