# 动作模仿 Topic 接口（2026-10-20）



## 通信

| 方向 | Topic | ROS消息类型 |
|---|---|---|
| 上位机 → 本项目 | `/sflare_os/game/imitate/cmd` | `std_msgs/msg/String` |
| 本项目 → 上位机 | `/sflare_os/game/imitate/result` | `std_msgs/msg/String` |

String.data 是 JSON 文本。真机domain=15，可靠通信；结果为事件流，不重放历史事件。
上位机先订阅 result，再发命令。不排队，忙时返回 error。ROS参数服务不属于业务接口。

## 请求

```json
{"cmd":"acts","args":{}}
```

| cmd | args | 作用 |
|---|---|---|
| acts | `{}` | 获取当前实际动作列表 |
| arm_check | `{}` | 只读自检，不运动 |
| arm_demo | `{"act_name":"点赞"}` | 指定动作示范，可能上电、运动并回零下电 |

act_name 接受 acts 返回的显示名，也接受内部英文名。不支持示例中的敬礼。
当前六动作：指向(diagonal_point)、握手(handshake)、前伸递物(handover)、
点赞(like)、举手过头(raise_head)、疑问(doubt)。以实际 acts 返回为准。
`right_init` 不是本项目新增的 Topic 命令，B键仍走原有服务。

## 结果

统一格式：

```json
{"type":"arm_check","data":{"msg":""}}
```

| type | data | 含义 |
|---|---|---|
| acts | `{"list":["指向","握手","前伸递物","点赞","举手过头","疑问"]}` | 实际动作列表 |
| arm_check | `{"msg":""}` | 空msg为本次自检正常；非空为急停、超限或读取/服务异常说明 |
| arm_demo | `{"state":"start"}` | 请求已接受，开始准备，不代表已经运动 |
| arm_demo | `{"state":"moving"}` | 实际进入动作运动段 |
| arm_demo | `{"state":"still"}` | 目标姿态到位后进入5秒展示段，仅显式标记phase=still的等待发布；准备等待不发布 |
| arm_demo | `{"state":"reset"}` | 实际开始收势/回零轨迹 |
| arm_demo | `{"state":"end"}` | 动作服务确认本轮成功结束；失败不发送end |
| arm_power | `{"state":"on"}` | 执行器已读取并确认本次动作臂上电 |
| arm_power | `{"state":"off"}` | 执行器已读取并确认本次动作臂下电；不是通过idle推测 |
| score | 见下例 | 有效B键握手完成后发布的归位前采样评分 |
| error | `{"cmd":"arm_demo","code":"ACTION_TIMEOUT","msg":"..."}` | 非法命令、忙、未知动作、动作失败等，扩展错误事件 |

动作可能包含多次moving；当前六个游戏动作各有一次still，对应目标姿态的5秒展示。不能把运动阶段硬编码为固定延时。
arm_power是本项目执行器的硬件确认事件，不是对外部驱动操作的全局电源监听。
未拿到确认时不伪造on/off；下电后不能仅根据服务返回code200判断机械到位。

### score

```json
{
  "type":"score",
  "data":{
    "score":95,
    "j1":{"target":103.0,"actual":101.2},
    "j2":{"target":0.0,"actual":1.3},
    "j3":{"target":0.0,"actual":0.2},
    "j4":{"target":0.0,"actual":0.8},
    "j5":{"target":0.0,"actual":-0.4},
    "j6":{"target":0.0,"actual":0.1},
    "j7":{"target":0.0,"actual":0.0}
  }
}
```

示例数值仅说明格式。角度单位均为度，七轴不含j0；target是上次成功示范的
动作标准，actual必须是在B键归位运动**之前**采集的编码器角度。
总分使用 spatial_joint_v2（位置70分+动作专属方向归一化关节30分），
阈值为未标定的可调工程初值。详细公式见 `docs/05-当前接口文档.md` 第4节；
该旧文档的Service调用章节已被本文件取代。

## 发布示例

只读动作查询：

```bash
ros2 topic pub --once /sflare_os/game/imitate/cmd std_msgs/msg/String \
  '{data: "{\"cmd\":\"acts\",\"args\":{}}"}'
ros2 topic echo /sflare_os/game/imitate/result std_msgs/msg/String
```

先 source ROS、机器人工作区和游戏install/setup.bash，设置domain=15与机器CycloneDDS配置。
游戏真机节点启动仍使用完整旧SDK：

```bash
export PYTHONPATH=/usr/local/lib:$PYTHONPATH
export PALLAS_SDK_LIB=/usr/local/lib/PallasSDK/libPallasSDK.so
ros2 run sflare_game_controller game_node --ros-args -p use_fake_sdk:=false
```

## B键客户端接口

| 方向（相对游戏节点） | Topic | 类型 |
|---|---|---|
| 客户端 → 游戏 | `/sflare_os/game/imitate/teleop_event` | `std_msgs/msg/String` |
| 游戏 → 客户端 | `/sflare_os/game/imitate/teleop_control` | `std_msgs/msg/String` |

String.data为JSON字符串。先订阅teleop_control和result，再进行示范与采样。

```text
成功示范 → arm_round → 实际开始遥操后started
按B → button → pause_for_sample → 停止遥操输出后paused
→ 读取真实角度、锁存并发布score → reset_after_sample
→ 客户端调用原right_init → 确认真实归位后reset_done
```

### 输出到teleop_control

以下R/S/Q分别表示本轮编号、遥操会话编号和采样请求编号；示例须替换为实际值。

```json
{"command":"arm_round","round_id":"R","capture_trigger":"button"}
{"command":"pause_for_sample","round_id":"R","session_id":"S","request_id":"Q","reason":"button"}
{"command":"reset_after_sample","round_id":"R","session_id":"S","request_id":"Q","reason":"button","sampled":true,"allow_reset":true}
{"command":"stop_without_reset","round_id":"R","session_id":"S","request_id":"Q","reason":"button"}
```

arm_round登记本轮，不是运动指令。R和Q由游戏节点生成，S由客户端为实际遥操生成。
stop_without_reset在无有效票据时可能仅含command和round_id。

### 输入到teleop_event

```json
{"event":"started","round_id":"R","session_id":"S"}
{"event":"button","round_id":"R","session_id":"S"}
{"event":"paused","round_id":"R","session_id":"S","request_id":"Q"}
{"event":"reset_done","round_id":"R","session_id":"S","request_id":"Q","success":true}
{"event":"cancelled","round_id":"R","session_id":"S"}
{"event":"emergency_stop","round_id":"R","session_id":"S"}
```

paused必须表示运动输出已暂停且在途输出已处理，不是仅收到暂停消息。
游戏节点在paused后读取真实反馈，采样时刻不是物理B键按下瞬间。
reset_done失败时success=false；只有确认真实下电才允许额外填写power_confirmed="off"。

客户端只在匹配R/S/Q且sampled=true、allow_reset=true的reset_after_sample到达后，
调用一次原 `/arm_control_act`（`aid_robot_msgs/srv/ArmControl`）：

```json
{"op":"call_service","service":"/arm_control_act","args":{"request":{"reqid":0,"act":"right_init","data":""}},"id":"b-reset-Q"}
```

服务response只用于该调用的结果配对，不能单凭code200推断真实到位或下电。
禁止同时发button和right_init；禁止把score当作独立归位许可。

### 时序、幂等与错误

- 暂停确认5秒、采样15秒、归位确认45秒，是故障保护；没有30秒自动采样。
- 重复B仅处理一次；旧轮次、旧会话、旧请求不得触发采样或归位。
- 客户端须对归位许可去重，断线/取消/急停后不得执行缓存许可或自动重试归位。
- 无标准、错误消息、忙和采样失败通过result中的error报告；失败不放行归位。
- cancelled/emergency_stop当前会输出CAPTURE_FAILED，并通知stop_without_reset，不执行自动归位。

完整客户端接入细节与rosbridge封装见 [B键客户端接口](docs/12-客户端B键接入接口.md)。
