# 动作模仿 · 交接文档

更新：2026-10-20
本文只描述**当前状态**；历史过程见 `docs/` 中标注「历史文档」的文件。

---

## 0. 一句话现状

六个示范动作 + 5 秒展示 + B 键采样 + `spatial_joint_v2` 评分，**代码完成**；
已部署 **172.168.102.134**（两包 colcon 构建成功，未启动）；
真机部分验收：`like` 通过，`handshake` 需 40s 余量才通过，`handover` 回零时遇急停中断，其余 3 个动作未测。

---

## 1. 功能

1. 客户端查询动作列表（`acts`），选择动作发起示范（`arm_demo`）
2. 动作服务执行六个动作之一：运动 → **目标姿态停留 5 秒** → 回零 → 下电
3. 客户端建立遥操会话；用户按 **B 键** → 客户端暂停运动输出 → 游戏采集归位前真实编码器角度
4. 游戏按 `spatial_joint_v2` 评分（位置 70 + 动作专属关节 30）并锁存
5. 游戏发放归位许可 → 客户端才调用原 `right_init` → 回传 `reset_done`

**没有 30 秒自动采样**，只由 B 键触发。

---

## 2. 架构与执行链路

```
上位客户端 ── /sflare_os/game/imitate/cmd (String JSON) ──▶ 游戏节点
           ◀─ /sflare_os/game/imitate/result (事件流) ─────┘
           ◀─ /sflare_os/game/imitate/teleop_control ──────┘
           ── /sflare_os/game/imitate/teleop_event ───────▶

游戏节点 ── 动作命令（内部动作服务协议）──▶ 动作服务
动作服务 ── PallasSDK move_j ──▶ 右臂 192.168.3.100:2094（robot_id=1）
```

- 六动作与右臂 `gozero` 默认走 **`move_j`**；`PALLAS_GAME_MOTION_MODE=emi` 可回退旧 EMI 执行层
- 评分角度取**真实编码器反馈**，不使用占位关节 Topic
- 原 `fish_driver_node` / `right_init` 不改，B 键逻辑在客户端侧截住归位调用

---

## 3. 仓库结构

```
motion-imitation/
├── README.md                入口与快速开始
├── API.md                   ★ 当前对外 Topic 接口（唯一有效）
├── HANDOVER.md              本文
├── src/
│   ├── sflare_game_controller/   游戏节点：命令、阶段、B 键状态机、评分
│   ├── sflare_game_interfaces/   ROS 消息/服务
│   └── sflare_action_service/    动作服务：六动作、move_j、手势、限位恢复
├── docs/
│   ├── README.md            ★ 文档索引（当前 / 历史）
│   ├── 04/07/09/10/11/12/13/14/15   当前有效
│   └── 00/01/02/03/05/06/08         历史存档（顶部有标注）
└── scripts/                 真机预检、部署、动作测试脚本
```

**不入库**：`reference/`、`*.so`、`*.tar.gz`、`*.zip`、`data/`、`traces/`、`logs/`、
`diagnostics/*.csv`、`__pycache__/`、`.pytest_cache/`、`build/`、`install/`。

---

## 4. 机器与部署

| 项 | 172.168.102.134（当前） | 172.168.102.48（上一测试机） |
|---|---|---|
| 角色 | 本次部署目标 | 真机动作结果来源 |
| 代码 | `/home/aidlux/RobotGK/sflare_game_ros`、`sflare_action_service` | 同路径旧版本 |
| 状态 | colcon 两包构建成功；**未启动节点** | 测试节点已停；臂处非零姿态 |
| SDK | 待核对（部署时未加载） | V2.0.5355 + PythonSDK V2.0.0，固件 COMM-E V2.4.0034 |

**所有机器通用**：

```bash
export ROS_DOMAIN_ID=15
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/aidlux/Haier_robot_ws/install/share/robot_bringup/param/cyclonedds.xml
export PYTHONPATH=/usr/local/lib:$PYTHONPATH
export PALLAS_SDK_LIB=/usr/local/lib/PallasSDK/libPallasSDK.so
```

- SDK 必须 **Python 包与 `libPallasSDK.so` 成套**（V2.0.5355），不要自动混用机器上的其他版本
- 动作服务与旧生产服务共用锁 `/home/aidlux/.pallas_arm_sdk.lock`，**同一时刻只能有一个**
- 切换：`sudo systemctl stop pallas_dual_arm_motion` → `cd .../sflare_action_service && bash start.sh`
- 恢复：`pkill -INT -f dual_arm_fist_stretch.py` → `sudo systemctl start pallas_dual_arm_motion`

---

## 5. 接口

完整字段、示例、时序与幂等要求见 **[API.md](API.md)**。

| 方向 | Topic |
|---|---|
| 客户端 → 游戏 | `/sflare_os/game/imitate/cmd` |
| 游戏 → 客户端 | `/sflare_os/game/imitate/result` |
| 客户端 → 游戏（B 键） | `/sflare_os/game/imitate/teleop_event` |
| 游戏 → 客户端（B 键） | `/sflare_os/game/imitate/teleop_control` |

命令：`acts` / `arm_check` / `arm_demo{act_name}`。
结果：`type = acts | arm_check | arm_demo(start/moving/still/reset/end) | arm_power(on/off) | score | error`。

> 旧的 `/sflare_os/game/command` Service 接口与 `moveaction` 命令**已废弃**，不要按旧文档调用。

---

## 6. 真机结果与已知问题

### 6.1 结果（2026-10-20，172.168.102.48）

| 动作 | 结果 |
|---|---|
| `like` 点赞 | **完整通过**：start/moving/still/reset/end 与 on/off 齐全 |
| `handshake` 握手 | 默认配置约 23s 触发完成超时；`timeout_margin_sec=40` 后 **26.861s 通过** |
| `handover` 递物 | 上升与展示完成，回零起始收到 `/e_stop=1` 中断，**不算通过** |
| `diagonal_point` / `raise_head` / `doubt` | 因急停停止序列，**未测** |

急停后只读复查：`check_error=[]`，机械臂停在非零姿态
`[32.974, -0.088, 0.252, 28.561, 83.139, -0.066, 0.022]°`，已下电断连，未发 `recover`。

### 6.2 已知问题

| 优先级 | 问题 | 说明 |
|---|---|---|
| **P0** | 现场急停原因未确认 | 臂未回零；**先问清原因再继续**，不要自动恢复或直接发动作 |
| **P1** | `handshake` 默认超时不足 | 动作估时 + 10s 余量不够（收尾伺服平息耗时）；测试用启动参数 40 是临时的，未改代码默认值 |
| **P2** | B 键客户端未适配 | 游戏端状态机就绪；客户端需按 `API.md` 截住 `right_init`，否则归位可能抢在采样前 |
| **P3** | `.134` 未启动验收 | 只完成了上传与构建 |
| **P4** | 评分阈值未标定 | `spatial_joint_v2` 的阈值是工程初值 |

---

## 7. 待办（按顺序）

1. **确认急停原因**；现场复位后先把机械臂安全回零（走原 `right_init` 或人工确认的流程）
2. 在 `.48` 复测剩余 3 个动作（`diagonal_point` / `raise_head` / `doubt`）
3. 把 `timeout_margin_sec` 的默认值按实测修正（或按动作配置），不要依赖启动参数
4. 客户端按 `API.md` 接入 B 键；完成一次真实「示范 → 遥操 → B → 评分 → 归位」整轮
5. `.134` 启动、自检、单动作、全动作验收
6. 标定评分阈值；确认 `handover` 回零速度与游戏总超时

---

## 8. 快速开始

```bash
# 动作服务
cd /home/aidlux/RobotGK/sflare_action_service
export PALLAS_GAME_MOTION_MODE=movej
bash start.sh

# 游戏节点
ros2 run sflare_game_controller game_node --ros-args \
  -p use_fake_sdk:=false -p timeout_margin_sec:=40.0

# 只读自检
ros2 topic pub --once /sflare_os/game/imitate/cmd std_msgs/msg/String \
  '{data: "{\"cmd\":\"arm_check\",\"args\":{}}"}'
```

详细操作与手动评分辅助程序见 **[docs/15-测试指令说明.md](docs/15-测试指令说明.md)**。

### 测试

```bash
# 动作服务
cd src/sflare_action_service
PYTHONPATH=/usr/local/lib python3 -m unittest \
  test_offline test_single_actions test_action_session test_recovery \
  test_hand_clearance test_game_limit_recovery test_movej -q

# 游戏侧
cd src/sflare_game_controller
python -m pytest -q
```

最近记录：动作服务 139 项（3 跳过）；游戏 84 通过 + 32 跳过（本机无 ROS）。

---

## 9. 历史索引（为什么是 move_j）

早期版本用 EMI `servo_j` 流式执行，2026-10-19 在 `.48` 上出现 **`servo_j` 返回 0 但机械臂不动、`move_j` 正常**，排查结论是控制器 EMI 引擎状态问题（非 SDK、非代码）。
2026-10-20 改为 `move_j` 后真机动作恢复。完整排查过程与错误假设见：

- `docs/03-游戏侧改造.md`（EMI 时期游戏侧）
- `docs/09-SDK5355真机动作链路测试.md`（真机结果）
- `docs/11-movej执行层切换.md`（切换设计与回退）

**不要**再从历史文档里照抄 `/sflare_os/game/command` 或 `moveaction` 的调用方式。

---

## 10. 交接注意

1. 仓库不含口令；机器人账号与口令由项目负责人单独提供
2. 厂商 `.so` 不入库，从目标机 `/usr/local/lib/PallasSDK` 取配套动态库
3. 任何运动前先 `arm_check`；急停优先于一切 Topic 命令
4. 报告测试结果时区分「离线测试通过」与「真机验收通过」
