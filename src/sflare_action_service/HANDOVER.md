# EMI Action Service 交接

更新时间：2026-10-16（北京时间）

## 最新变更：2026-10-19 灵巧手碰撞余量

用户确认升级固件后剩余动作测试通过；新机器高度正常，旧机器零位问题不在本工程修改。本批仅针对灵巧手碰撞：safe_fist和hold_stretch_grip的拇指弯曲10→7（0～10标度），四指10及拇指旋转5.5不变；共用open先以SerialCtrl 0x4C单独发送拇指弯曲0和旋转6，先行0.5秒，再以原0x50全手张开。初始化未知手势时也不先重发四指握紧。

0.5秒是待现场确认的先行时间，不是反馈确认到位；若仍摩擦应停测，不能宣称已保证避碰。运行时通过原tick保持上一帧目标，不暂停EMI发包；初始化/清理阶段的等待可被取消。取消或单指发送失败时不继续四指张开。动作坐标、零点、底盘及运动参数不改。

协议依据：桌面 roh_lites_firmware-main/protocol/OHandSerialProtocol_CN.md，HAND_CMD_SET_FINGER_POS=0x4C。新增test_hand_clearance.py，完整回归命令增加该模块。底盘停止失败测试改为在线性线程启动后计数，避免新增手势保持帧使原测试提前取消。

修改前本地备份：EMI Action Service_backups/20261019_before_hand_clearance。真机拇指余量与0.5秒先行时间待用户现场确认；急停测试读取真实/e_stop，不注入伪造信号。

## 当前交付：剩余功能模块本地迁入完成（2026-10-16）

用户本轮要求全部迁入；随后明确机器暂不可SSH，只做本地备份、不部署。当前机器仍是旧版本，不要声称远端已更新或本批已真机验收。

- limit_recovery.py：从原版单独提取五个恢复方法及原常量；候选轴判定、报警Axis解析、闭环自适应Jog核心AST与原版一致。保留正负限位、真实越界逐轴恢复、近限位3°、15速度、0.15秒起始脉冲、0.1～0.5秒自适应、90秒/600次限制、行程+1°、方向错误和无进展保护、清警例外-5047/-5059。
- emi_coordinator.py：在发送线程退出、底盘停止、下电确认、诊断完成后接入原恢复流程；仅/e_stop明确为0、无取消及其他故障时授权，先确认退出EMI，恢复后再次下电确认再断连。不修改正常运动插补、发包参数或周期。
- controller_guard.py：SDK非零返回值检查；外部控制释放探测、探测成功后先去使能再断连，拒绝连接不碰对方句柄；探测无robot、无上电、无profile、无运动。
- ROS入口：恢复结果状态；网络致命故障在安全清理后退出，未确认清理仍退出且禁止复用；只有本会话急停+-5029共同证据能解释单独-20039。物理故障不被提前recover掩盖。
- 连接监测沿用原版5秒节奏并独占工作线程会话，不开第二个SDK所有者；期间动作/回零不排队，stop可取消监测，实体急停仍优先。
- 结合10月10日新SDK实测修正-20035：TCP不可达为connection_failed，可达但SDK拒绝为可能handle_busy；只有SDK真正连接并安全释放后才ready，不将该码一概宣称手柄占用。
- 接入适配：恢复后还原当前已验收EMI profile，而非原move_j profile；清警前再次核对物理/电源/网络故障；SDK设置调用检查返回值。原错误码正则排除负小数，避免把越界日志里的负关节角/限位值当成SDK错误码，误阻断合法恢复。
- 自动脱限不续做原动作、不自动回零。动作JSON、action_pipeline.py、action_program.py、chassis_control.py均保持原样。

验收清单：FULL_ACCEPTANCE.md。新回归模块：test_recovery.py；完整命令：

```bash
python3 -m compileall -q .
python3 -m unittest test_offline test_single_actions test_action_session test_recovery -q
```

本轮本地101项测试通过；包含原核心AST比对、模拟恢复收尾与监测/急停竞态。没有真实ROS集成或Jog真机验收。仅为测试工程的功能迁入完成，不代表与原move_j服务的动态时序完全等价，不包含正式systemd服务安装/发布。手动启动进程退出后不会自行重启，已有管理器的重启策略由部署方负责。

修改前备份：C:/Users/blond/Desktop/EMI Action Service_backups/20261016_before_remaining_modules/EMI Action Service。修改后交付备份见同父目录20261016_remaining_modules_verified；备份内DEPLOYMENT_MANIFEST.json记录文件哈希。此前历史记录如下，仅代表对应日期，不再代表当前功能缺失情况。

## 功能模块迁入：第二阶段第一批（故障分类）

新增 action_faults.py，迁入原版物理报警、强电未就绪、网络故障、无效目标及限位故障分类；由当前工作线程已有的异常和清理后 check_error 结果分类，不新增SDK查询。故障优先于普通stop/动作完成，阻止故障被ready掩盖；物理报警仍锁定并沿用已有recover确认机制。/e_stop明确为0时过滤历史-5029，未知或按下时按物理报警处理。

新SDK实机日志证明-20035也表示TCP连接失败（未接左臂、eth0 DOWN、No route to host），因此不再仅凭此码声称手柄占用，发布connection_failed；未引入占用轮询。该项是基于实测对原版错误码解释的修正。

本批仅迁入分类和已有会话门禁，不迁入自动Jog限位恢复、网络故障自动重启、外部占用监测，不改变recover行为。限位故障发布limit_recovery_failed表示未恢复，不触发任何恢复运动；fatal_network当前仅为状态，沿用本工程安全清理策略。复杂恢复留待下一批独立验证。

离线回归包含新增4项测试，共62项；动作JSON、插补、编排、EMI发送及底盘模块未修改。本地修改前备份：C:/Users/blond/Desktop/EMI Action Service_backups/20261010_164843_before_fault_classification/EMI Action Service。

部署目标：172.168.102.76:/home/aidlux/RobotGK/EMI Action Service。仅更新本批入口、故障模块、测试和本文档；不覆盖SDK、配置、动作、日志或诊断文件，不自动重启节点，不运行真机动作。旧进程保持旧模块，更新将在下次启动时生效。

## 功能模块迁入：第一阶段

用户于2026-10-09反馈新迁移动作真机验收完成，结论为“基本没有问题”。这是用户验收记录，不是本轮重新进行真机测试。

本轮完成原版急停、停止、回零优先级及会话管理迁入：
- 新增 action_session.py，沿用原版会话字段，SDK仍只归动作工作线程所有。
- stop 优先于 gozero；重复 gozero 合并；正常动作不排队。
- 右臂动作被回零抢占时只回右臂；独立 gozero 仍按主机单/双臂能力执行。
- 回零必须在旧EMI会话下电、断连确认后执行；沿用现有check_error诊断结果作原版故障门禁，不增加SDK查询。诊断不完整或原版阻断故障存在时不自动回零。
- 急停立即锁定、取消会话并发送ROS底盘零速通知；释放后不自动运动。清理期间提前收到recover先登记，确认安全清理且/e_stop=0后才确认恢复；再次急停撤销该确认。
- recover仅确认服务锁定，不连接控制器、不清警、不上电、不运动。
- 状态提交和会话释放在同一锁内完成；旧工作线程不能覆盖新会话状态；急停清理期间不误报普通busy或提前ready。
- 安全清理未确认仍退出进程，禁止回零或确认恢复。

验证：58项离线测试全部通过（原37项＋21项会话测试）；含真实Python线程清理竞态、FakeSDK先下电断连再建立回零连接；compileall通过。19个动作JSON以及action_pipeline.py、action_program.py、chassis_control.py的SHA256与本轮修改前完全一致。emi_coordinator.py仅增加已有诊断结果的存储供优先级回零门禁使用，运动参数/插值/发包逻辑未改。

本批只修改本地测试工程，未部署、未进行真机运动或真实ROS集成测试。下一阶段才迁入完整故障分类、外部控制器占用监测；复杂故障恢复与限位恢复仍未迁入。

当前完整离线回归命令：
python3 -m unittest test_offline test_single_actions test_action_session -q

以下为此前动作迁移记录，保留历史测试参数和边界：

本轮完成桌面测试工程的全部16个单臂动作迁移和离线回归。**本轮未部署远端、未替换正式服务、未执行真机动作。** 此前仅 wave 在新 SDK/固件环境已完成真机验证；其他动作及底盘同步仍需逐个验收。

## 范围和解耦

- actions/*.json：16个单臂动作，以及原有 fist/stretch/stretch_ready 三个双臂动作。
- dual_arm_fist_stretch.py：ROS入口、原动作别名、取消和工作线程，不放点位表。
- action_pipeline.py：文件读取、校验、原五次 Hermite 插值与轨迹规划。
- action_program.py：单臂动作 JSON 流程解释、提前缓存轨迹、手势/停留/反馈到位、里程计条件循环。
- chassis_control.py：从原版单独抽取的底盘 ROS接口和控制方法，不包含机械臂 SDK 运动代码。
- emi_coordinator.py：统一 EMI 连接、控制器限位预检、发送、反馈、下电断连。
- test_offline.py / test_single_actions.py：纯离线测试。不会连接机器人。
- diagnostics/migrate_single_actions.py：一次性提取工具，只打印补丁；不要反复运行覆盖人工调参。

权威参考：
C:/Users/blond/Desktop/action service/01_源码/当前版本/dual_arm_motion_actions.py

没有复制三千多行的正式服务，没有引入 move_j、校零、机械臂限位 Jog 恢复或外部句柄释放监测。仍不能称为完整正式服务替代品。

## 动作和底盘

单臂动作：like、handshake、forward_wave、dance、sway、heart、forward、turn_grip、doubt、raise_head、shakehand、slight_raise、diagonal_point、chin_rest、handover、wave。

六个底盘联动动作：forward_wave、dance、sway、heart、forward、turn_grip。保留原 /odom、/cmd_vel_remote_ctrl、/mode_set 接口；保留 yaw 控制参数、反馈稳定到位、超时、无进展、命令角度预算、漂移重获控制，以及直行0.2m/s、0.3m（原版按1.5秒计时，不新增里程闭环）。

turn_grip 保留至多100次、以底盘到位提前结束的循环；dance 保留至多34次旋转循环及50次收尾等待。dance 全圈18秒、0.6rad/s、540度命令预算；最终方向修正8秒、0.4rad/s、90度修正范围，预算公式不变。

stop/急停/异常时停止所属底盘线程并回idle，未确认线程退出则禁止复用会话。底盘故障触发的取消不再误报为正常ready。
dance 纯编排失败保留停止底盘后健康臂回零的处理；不对SDK/跟踪故障或取消/急停执行自动回零，绝不清除共享取消事件。与原版“底盘故障时清取消后回零”相比，这是保守边界，不可声称完全等价。

## 播放和测试参数

wave 保持已验证文件及原插值路径不变：20ms、原六个点位、末点11.4秒，6.6→11.4回零段为4.8秒。SHA256：
f63b157c6b4c247c07acf2a8dc1cac038daeb95b6c1776b298775cf4ba37ebb8

其余15个单臂动作采用 program 流程：
- keyframes 保存原版点位、末尾显式零点；time 是点位库索引时间，不是实际播放时间轴。
- program 的 move.frame 引用点位，duration 定义 EMI 段时长；wait 定义停留，hand 定义原手势，yaw/linear 指令定义底盘联动。
- 全部可选运动边在上电前采用相同五次平滑启停核预插补，逐点经形状/有限值校验，再按控制器 get_limit 动态限位预检。运行时仅选缓存帧，不计算样条。
- 周期统一20ms；首段5秒；普通段1.8秒；握手段0.4秒；疑问段1.2秒；回零4.8秒；超过85度/秒的步长仍会细分。
- 保留原手势内容、触发顺序、展示停留和回零前张手。普通 move 进入下一步前保留反馈到位确认；dance 原不等待到位的中间 dispatch 对应 settle=false，但仍发送完整 EMI 段。

原 move_j 没有固定段时长。**上述时长是新测试参数，完整EMI段不等同于原dance的0.3秒重发目标节奏；运动时间及底盘同步尚未真机验收。** 也不能把 wave 的 pass 反向插值与新动作的 stop-to-stop 段说成完全相同的曲线。

不强制 J4>=35°，不锁零J6/J7，不修改原坐标迎合限位。原疑问/托腮J4为116/119度；控制器若不允许，启动前拒绝动作，不能偷偷截断或放宽控制器限位。

## 验证结果

2026-09-30：共37项离线测试通过；compileall通过。
覆盖原21项回归、新动作完整性、16个动作坐标与原版AST比对、15个底盘方法AST一致性、全部缓存轨迹步长/端点/范围、原手势参数、原别名、里程计循环边界、取消停底盘、底盘故障分类、退出未确认禁止复用、限位拒绝、dance纯编排失败回零，以及15个新动作完整FakeSDK会话。

执行：
python3 -m unittest test_offline test_single_actions -q

本机缺少真实ROS运行环境，未做ROS在线集成测试。FakeSDK中为协议测试使用合成限位，不代表真机能接受所有原点位。

## 真机验收前

- 用户曾观察到下电后J4位置明显变化，并怀疑机械传动问题：先确认机械/制动安全，不自动连续运行。
- 确认最新SDK、固件、robot ID、/e_stop以及实际关节限位。
- 本测试节点与正式节点使用同一动作topic，不能同时占用控制器或响应同一指令。
- 先逐个纯手臂动作检查点位、手势、停留、回零，再在安全底盘区域验收六个联动动作的同步和超时。
- wave 的已有发包/原始反馈 CSV 和统计机制保留，其他动作沿用同一诊断记录。
