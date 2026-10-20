#!/bin/bash
# 只做 Controller.connect() 对比测试：游戏自带 SDK vs 已验证 SDK
# 安全边界：不 add_robot、不上电、不运动；连接后立即 disconnect
# 恢复函数验证：ONLY_RESTORE=1 bash sdk_probe.sh
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
GOOD_LIB="/home/aidlux/RobotGK/PallasSDK/libPallasSDK.so"

emi_pids() { pgrep -f "$EMI_PATTERN" || true; }

restore_emi() {
  if [ -n "$(emi_pids)" ]; then
    echo "EMI 已在运行，跳过启动"
    return 0
  fi
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
if [ -n "$(emi_pids)" ]; then echo "EMI 未停，放弃"; exit 1; fi
echo "EMI 已停"

probe() {
  local label="$1"; shift
  echo "=== $label ==="
  env "$@" python3 - <<'PY' 2>&1 | tail -8
import sflare_game_controller.PallasSDK as S
from sflare_game_controller.PallasSDK import _lib as L
print("PallasSDK:", S.__file__)
print("loaded lib:", getattr(L, "_lib", None))
c = S.Controller()
try:
    print("connect rc =", c.connect("192.168.3.100", 2094), "| is_connected =", c.is_connected())
except Exception as exc:
    print("connect raised:", type(exc).__name__, exc)
finally:
    try:
        print("disconnect rc =", c.disconnect())
    except Exception as exc:
        print("disconnect raised:", type(exc).__name__, exc)
PY
}

probe "2) 游戏自带 SDK" PALLAS_SDK_LIB=
echo
probe "3) 已验证 SDK（与 EMI 同款）" PALLAS_SDK_LIB="$GOOD_LIB"
