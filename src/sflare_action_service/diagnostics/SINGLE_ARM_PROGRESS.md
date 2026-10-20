# 单臂 EMI 迁移进度（2026-09-30 20:00续做批次）

## 当前进度：2026-10-16，剩余功能本地迁入完成

已迁入自动Jog脱限、连接释放监测、网络故障退出及对应会话门禁。保留原恢复算法/常量，不改动作JSON、插补、编排、底盘控制及正常EMI参数。完整说明与差异见HANDOVER.md顶部，验收操作见FULL_ACCEPTANCE.md。

本地101项离线测试通过。用户最新要求：只做本地备份，不SSH部署、不启动真机运动；172.168.102.76未更新。原测试工程不因本轮迁入自动替换正式服务。修改前后备份在桌面EMI Action Service_backups目录。

下文全部是历史进度，勿据历史“未迁入”重复生成模块或覆盖已验收动作。

## 最新进度：2026-10-09，功能模块第一阶段完成

用户报告16个新迁移动作已完成真机验收，基本没有问题。本轮迁入原版急停/停止/回零优先级和会话状态管理；新增action_session.py、test_action_session.py，改动ROS入口，emi_coordinator.py仅缓存已有诊断结果。保留所有已验收JSON、插值/轨迹模块、底盘控制方法及EMI运动参数。

58项离线测试通过，含21项会话/优先级测试和FakeSDK连接顺序验证；compileall通过。19个JSON及action_pipeline/action_program/chassis_control哈希与修改前一致。没有远端部署，没有真机动作，没有正式服务替换。

详见HANDOVER.md顶部第一阶段记录。下一阶段：原版完整故障分类、状态上报、外部句柄释放监测；再迁入原有故障恢复/限位恢复与退出/启动集成。不要重复迁入已完成会话管理，不自动开展真机或正式部署。此前小时自动续做仍暂停，未因本轮重新启动。

下文为历史动作迁移结果：

## 当前结果

本地代码迁移与离线验证完成。16个单臂动作全部存在，原别名已接入；六个底盘联动保留原接口/方法及里程计循环。37项离线测试通过，compileall通过。wave.json SHA256仍为f63b157c6b4c247c07acf2a8dc1cac038daeb95b6c1776b298775cf4ba37ebb8，未改已验证插值路径。

新增/更新：actions/*.json（15个新迁移动作，不改wave）、action_program.py、chassis_control.py、action_pipeline.py、emi_coordinator.py、dual_arm_fist_stretch.py、test_offline.py、test_single_actions.py、HANDOVER.md；一次性源提取工具在diagnostics/migrate_single_actions.py。

本轮没有远端部署，没有正式服务改动，没有真机运动。详细参数、行为差异和验收顺序见HANDOVER.md。

## 明确差异/限制

- 新动作采用stop-to-stop五次EMI段；不是把wave的pass反向曲线生搬给所有动作。
- 原move_j没有固定时长；新首段5秒、常规1.8秒、握手0.4秒、疑问1.2秒、回零4.8秒是测试参数。dance中间原dispatch被完整EMI段替代，0.3秒dwell保留，整体节奏需验收。
- 原版底盘15个方法AST一致；底盘故障引发取消按失败报告，不当正常stop。
- dance纯编排失败时停止底盘后仅在无取消、反馈无故障时回零；不清除共享取消/急停，不对底盘故障设置的cancel强行恢复。这一保守边界已写入交接。
- 控制器实际限位若不允许原J4=116/119等，启动前拒绝，不改坐标、不加全局J4保护。
- 尚无真实ROS集成测试；不能称正式服务全功能等价（不包含原复杂恢复模块）。
- 用户曾报告下电后J4下落/怀疑脱齿，未来真机验收需先机械安全确认，不自动测。

## 后续

此批迁移与离线验证已完成，关闭小时续做，避免继续做未授权的真机/正式部署。等待用户后续部署或验收指令。若继续调参，读取HANDOVER和实际代码，不重复运行提取脚本覆盖人工参数。

项目：C:/Users/blond/Desktop/EMI Action Service
原版：C:/Users/blond/Desktop/action service/01_源码/当前版本/dual_arm_motion_actions.py
Python：C:/Users/blond/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe
远端测试目录历史：172.168.102.145 /home/aidlux/RobotGK/EMI Action Service；本批未接入。
