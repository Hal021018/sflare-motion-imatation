# 游戏六动作切换 move_j（2026-10-20）

## 范围与当前状态

仅修改本地 motion-imitation 项目。未部署至测试机，未上电、未发运动命令。
默认切换 like、handshake、handover、diagonal_point、raise_head、doubt 及右臂 gozero。
其余动作、双臂流程、正式动作服务不改。旧 SDK V2.0.5355 不变。

切换前完整 src、API.md、HANDOVER.md 备份：
`C:/Users/blond/Desktop/motion-imitation_backups/before_movej_20261020_153528`。

## 执行链路

原 Topic → 原会话/急停门控 → motion_backend.py → MoveJCoordinator → SDK move_j。
保留入口文件名 dual_arm_fist_stretch.py 与既有节点/Topic 名称，名称中的 EMI 不再代表六动作的实际运动方式。
不调用 EMI 启停、逐帧发送或插补；仅复用原模块的 SDK 加载、灵巧手与下电清理工具。

六个 JSON 与备份逐文件校验一致：坐标、点序、手势、5秒展示不变。
后续still语义修正仅给六个5秒展示wait增加 `phase:"still"` 元数据；去除此字段后与备份完整一致。
手势仍使用现有拇指先让位的安全张开流程。Topic、评分及仅B键采样规则不改；真实B键驱动握手仍未接通。

## 原源码复用与时序边界

参数来源：桌面 action service/01_源码/当前版本/dual_arm_motion_actions.py。
未导入该大型脚本，未复制其整个业务框架。

- set_speed=100；move_j 调用超时5000ms；到位等待35秒、轮询100ms。
- 到位条件：各轴误差≤1°且速度≤0.05rad/s，连续3次。
- like/handshake 使用原 NORMAL_PROFILE；doubt 使用 DANCE_PROFILE；其余使用 SMOOTH_PROFILE。
- Profile 依次为原SDK的11个字段，参数与原源码一致，离线测试进行AST对照。

重要：保留关键帧不代表运动时间不变。JSON 的 EMI 段时长不作为 move_j 时长；实际加减速和运行时间由原 Profile 与控制器决定。
因此原先4.8秒等回零段时间不能保证沿用，真机验收需重点检查回零速度及游戏总超时，不能宣称已解决顿挫或机械问题。

保留取消/急停门控；异常先停止、确认下电后断连。下电失败不释放连接、不允许自动复用。
启动发现已经上电则拒绝接管，不切断其他控制者的电源。未新增自动恢复运动。
独立限位恢复流程保持既有实现，本轮不调整。

## 回退与离线验证

默认环境变量 PALLAS_GAME_MOTION_MODE=movej；显式设置为 emi 可回退旧执行层。变量非法时拒绝执行。
配置通过服务进程启动环境传入，无需改变公开 Topic 消息。

新增 test_movej.py：覆盖六动作顺序与手势、Profile、回零优先级、取消、到位超时、无效反馈、越界、上电接管拒绝及下电失败。
假SDK禁止 EMI 调用；测试不连接硬件。原EMI会话回归显式选择 emi 模式，避免混淆两种执行器。
本机无ROS的用例会跳过，离线通过不能等同真机验收。

本轮结果：动作服务 unittest 共139项，3项跳过，其余通过；末次反馈读取/上电确认调整后新增后端12项再次通过。
游戏侧 pytest 84项通过、32项跳过；修改文件 compileall 通过。
