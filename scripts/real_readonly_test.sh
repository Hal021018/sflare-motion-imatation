#!/bin/bash
# 一次性只读真机测试：停 EMI -> 游戏真机模式 -> self_check（可选 release_limit）-> 恢复 EMI
# 安全边界：不发送任何关节运动指令；release_limit 仅在自检显示无急停、无超限时才发（会返回 NOT_REQUIRED）
# 验证恢复函数：ONLY_RESTORE=1 bash real_readonly_test.sh
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/aidlux/Haier_robot_ws/install/setup.bash
source /home/aidlux/sflare_game_ros/install/setup.bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PALLAS_ARM_MODE=single
export CYCLONEDDS_URI=file:///home/aidlux/Haier_robot_ws/install/share/robot_bringup/param/cyclonedds.xml

EMI_DIR="/home/aidlux/RobotGK/EMI Action Service"
EMI_PATTERN='^python3 dual_arm_fist_stretch.py'
LOCK_FILE="/home/aidlux/.pallas_arm_sdk.lock"
GAME_PATTERN='use_fake_sdk:=false'
GAME_LOG=/tmp/game_real_test.log

emi_pids() { pgrep -f "$EMI_PATTERN" || true; }

restore_emi() {
  if [ -n "$(emi_pids)" ]; then
    echo "EMI 已在运行，跳过启动"
    return 0
  fi
  # 锁被其它服务（如 pallas_dual_arm_motion 旧生产服务）占用时，不抢锁、不报错
  local holder
  holder=$(fuser "$LOCK_FILE" 2>/dev/null | tr -s '[:space:]' ' ' | sed 's/^ //;s/ $//')
  if [ -n "$holder" ]; then
    echo "SDK 锁被 pid ${holder} 占用（非 EMI），跳过 EMI 启动；未做所有权切换"
    return 0
  fi
  # trap 执行时不再依赖外层环境，显式重建（曾因缺 aid_robot_msgs 启动失败）
  source /opt/ros/humble/setup.bash
  source /home/aidlux/Haier_robot_ws/install/setup.bash
  export ROS_DOMAIN_ID=15 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PALLAS_ARM_MODE=single
  export CYCLONEDDS_URI=file:///home/aidlux/Haier_robot_ws/install/share/robot_bringup/param/cyclonedds.xml
  cd "$EMI_DIR" || { echo "EMI 目录不存在"; return 1; }
  local attempt i
  for attempt in 1 2; do
    setsid nohup /usr/bin/flock -n /home/aidlux/.pallas_arm_sdk.lock \
      python3 dual_arm_fist_stretch.py > /tmp/emi_action.log 2>&1 < /dev/null &
    disown 2>/dev/null || true
    for i in $(seq 1 10); do
      sleep 1
      if [ -n "$(emi_pids)" ]; then echo "EMI 已启动（第 ${attempt} 次尝试）"; return 0; fi
    done
    echo "EMI 第 ${attempt} 次启动失败："
    tail -5 /tmp/emi_action.log 2>/dev/null
  done
  return 1
}

trap restore_emi EXIT

if [ "${ONLY_RESTORE:-0}" = "1" ]; then
  echo "ONLY_RESTORE=1：只验证恢复函数（幂等，不动机械臂）"
  exit 0
fi

echo "=== 1) 停 EMI ==="
PIDS=$(emi_pids)
[ -n "$PIDS" ] && { kill -TERM $PIDS; sleep 3; }
if [ -n "$(emi_pids)" ]; then echo "EMI 未停，放弃测试"; exit 1; fi
echo "EMI 已停"

echo "=== 2) 启动游戏节点（use_fake_sdk=false） ==="
cd /home/aidlux/sflare_game_ros || exit 1
setsid nohup ros2 run sflare_game_controller game_node --ros-args \
  -p use_fake_sdk:=false \
  -p controller_ip:=192.168.3.100 \
  -p move_speed_percent:=20.0 \
  -p move_hold_sec:=2.0 \
  > "$GAME_LOG" 2>&1 < /dev/null &
disown 2>/dev/null || true
sleep 8

echo "--- 节点日志 ---"
cat "$GAME_LOG"
echo "--- 节点进程 ---"
pgrep -af "$GAME_PATTERN" || echo "节点未起"

echo "=== 3) self_check（只读） ==="
SC=$(timeout 40 ros2 service call /sflare_os/game/command \
  sflare_game_interfaces/srv/GameCommand "{request_type: 'self_check'}" 2>&1)
echo "$SC"

EMERGENCY_OK=0
OVERTRAVEL_OK=0
echo "$SC" | grep -qE '"emergency_stop_status":[[:space:]]*"0"' && EMERGENCY_OK=1
echo "$SC" | grep -qE '"overtravel_status":[[:space:]]*"0"' && OVERTRAVEL_OK=1

if [ "$EMERGENCY_OK" = 1 ] && [ "$OVERTRAVEL_OK" = 1 ]; then
  echo "=== 4) release_limit（自检安全；无超限应返回 NOT_REQUIRED，不 Jog） ==="
  timeout 40 ros2 service call /sflare_os/game/command \
    sflare_game_interfaces/srv/GameCommand "{request_type: 'release_limit'}" 2>&1
else
  echo "=== 4) 跳过 release_limit：自检显示急停或超限异常（或调用本身失败） ==="
fi

echo "=== 5) 停止游戏节点 ==="
pkill -INT -f "$GAME_PATTERN" 2>/dev/null
sleep 6
pgrep -af "$GAME_PATTERN" || echo "游戏节点已退出"

echo "=== 6) 恢复前确认无残留 ==="
pgrep -af 'sflare_game_controller/game_node' || echo "无残留游戏节点"
