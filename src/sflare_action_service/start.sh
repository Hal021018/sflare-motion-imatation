#!/bin/bash
# 启动「游戏版」固定动作服务副本
#
# 与机器上原有的 "EMI Action Service" 完全独立：
#   - 独立目录：/home/aidlux/RobotGK/sflare_action_service
#   - 复用同一个 SDK 锁 → 两个服务不会同时占用机械臂
#   - 按项目要求使用 /usr/local/lib/PallasSDK 的 V2.0.5355
#
# 用法：bash start.sh
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/aidlux/Haier_robot_ws/install/setup.bash

export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PALLAS_ARM_MODE=single
export CYCLONEDDS_URI=file:///home/aidlux/Haier_robot_ws/install/share/robot_bringup/param/cyclonedds.xml

# Python 包和动态库必须成套，禁止继承外部新版动态库配置。
SDK_ROOT="/usr/local/lib"
if [ ! -f "$SDK_ROOT/PallasSDK/libPallasSDK.so" ]; then
  echo "缺少完整 SDK: $SDK_ROOT/PallasSDK" >&2
  exit 1
fi
export PYTHONPATH="$SDK_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PALLAS_SDK_LIB="$SDK_ROOT/PallasSDK/libPallasSDK.so"

cd "$(dirname "$0")" || exit 1

exec /usr/bin/flock -n /home/aidlux/.pallas_arm_sdk.lock python3 dual_arm_fist_stretch.py
