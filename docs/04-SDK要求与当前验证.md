# SDK 要求与验证状态

2026-10-19：用户明确要求使用旧版 **PallasSDK V2.0.5355**。

- 测试机：172.168.102.48。
- 完整 SDK：`/usr/local/lib/PallasSDK`，Python 包与动态库成套使用。
- 动作服务 `start.sh` 已固定该路径，并覆盖继承的 `PALLAS_SDK_LIB`。
- 游戏节点启动环境须设置 `PYTHONPATH=/usr/local/lib:$PYTHONPATH` 和
  `PALLAS_SDK_LIB=/usr/local/lib/PallasSDK/libPallasSDK.so`。
- 此前 V2.1.1391 的小幅 EMI 探针及点赞运动结果，不代表旧 SDK 已验收。
- 旧版此前存在周期性 Invalid robot ID 与 EMI 退出问题，仍需定位，不能宣布修复。
- 不修改动作关键帧、插值算法或固件来掩盖 SDK 问题。
