# 动作模仿 (Motion Imitation)

机器人示范六个固定动作 → 用户遥操模仿 → 按 B 键采集归位前真实关节角 → 评分。

## 当前状态（2026-10-20）

| 项 | 状态 |
|---|---|
| 六个动作执行层 | 已从 EMI `servo_j` 切换为 **`move_j`**（`PALLAS_GAME_MOTION_MODE` 可回退） |
| 5 秒展示 | 六个动作 JSON 的展示 `wait` 已标记 `phase:"still"`，仅在此时发布 `still` |
| B 键采样 | 游戏端状态机完成；客户端需按 `API.md` 改造，**尚未接入** |
| 部署 | 已上传至 **172.168.102.134** 并 colcon 构建两包；未启动节点 |
| 真机验收 | `like` 完整通过；`handshake` 需把 `timeout_margin_sec` 调到 40 才通过（26.9s）；`handover` 回零起始遇 `/e_stop=1` 中断，剩余 3 动作未测 |
| 现场遗留 | 机械臂当前非零姿态；**须先问清急停原因**再继续 |

> 详细进度、真机记录、踩坑与下一步见 **[HANDOVER.md](HANDOVER.md)**。
> 功能说明见 [docs/14-项目功能说明.md](docs/14-项目功能说明.md)，接口见 [API.md](API.md)，
> 全部文档索引见 [docs/README.md](docs/README.md)。

## 目录结构

```
motion-imitation/
├── src/
│   ├── sflare_game_controller/   游戏节点（Topic 命令、评分、B 键状态机）
│   ├── sflare_game_interfaces/   ROS 消息/服务定义
│   └── sflare_action_service/    动作服务副本（六动作、move_j、recover_limit）
├── docs/                         00–15 设计、部署、测试与排查记录
├── scripts/                      真机预检、部署、动作测试脚本
├── API.md                        当前对外 Topic 接口（唯一有效版本）
├── HANDOVER.md                   交接文档（进度、阻塞、下一步）
└── README.md
```

不入库：`reference/`（原始交付快照）、`*.so`（厂商库）、`*.tar.gz`（备份包）、
`data/`、`traces/`、`logs/`、`diagnostics/*.csv`（原始轨迹数据）。

## 快速开始

```bash
# 环境（真机）
source /opt/ros/humble/setup.bash
source /home/aidlux/Haier_robot_ws/install/setup.bash
source /home/aidlux/RobotGK/sflare_game_ros/install/setup.bash
export ROS_DOMAIN_ID=15 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/aidlux/Haier_robot_ws/install/share/robot_bringup/param/cyclonedds.xml
export PYTHONPATH=/usr/local/lib:$PYTHONPATH          # 必须 V2.0.5355
export PALLAS_SDK_LIB=/usr/local/lib/PallasSDK/libPallasSDK.so

# 动作服务
cd /home/aidlux/RobotGK/sflare_action_service && bash start.sh

# 游戏节点（真机）
ros2 run sflare_game_controller game_node --ros-args -p use_fake_sdk:=false

# 只读自检
ros2 topic pub --once /sflare_os/game/imitate/cmd std_msgs/msg/String \
  '{data: "{\"cmd\":\"arm_check\",\"args\":{}}"}'
```

## 测试

```bash
# 动作服务（离线）
cd src/sflare_action_service
PYTHONPATH=/usr/local/lib python3 -m unittest \
  test_offline test_single_actions test_action_session test_recovery \
  test_hand_clearance test_game_limit_recovery test_movej -q

# 游戏侧（本机无 ROS 时部分跳过）
cd src/sflare_game_controller
python -m pytest -q
```

最近记录：动作服务 139 项（3 跳过）；游戏 84 通过 + 32 跳过（本机无 ROS）。

## Git 交接

仓库尚未推送到远端。首次上传：

```bash
git add -A
git commit -m "feat: 动作模仿 1.0（六动作 move_j + B 键采样 + spatial_joint_v2 评分）"
git remote add origin <仓库地址>
git push -u origin master
```

- 密码/令牌不写入仓库；机器人登录与 SDK 位置见 `HANDOVER.md` 第 3 节。
- `reference/` 与 `*.so` 被忽略；接手方需从机器人 `/usr/local/lib/PallasSDK` 取配套动态库。
