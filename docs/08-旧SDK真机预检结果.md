# V2.0.5355 真机链路测试预检（2026-10-20）

> **历史记录**：固件升级前的预检结果；后续结果见 [09-SDK5355真机动作链路测试](09-SDK5355真机动作链路测试.md)。

测试机172.168.102.48，控制卡192.168.3.100:2094，robot1。
本轮没有执行上电、EMI启动或运动，没有切换SDK、修改固件或清除报警。

## 实测

- Python包：/usr/local/lib/PallasSDK/__init__.py。
- 实际动态库：/usr/local/lib/PallasSDK/libPallasSDK.so（进程maps确认）。
- get_sdk_version：PallasSDK_V2.0.5355; PythonSDK_V2.0.0。
- 固件：COMM-E_V2.4.0046，QRTOS_V2.4.78，Interpreter_V2.4.35。
- 控制器状态：[0,0,0,0,0,1]，第0项急停反馈0。
- 机械臂状态：预检前、绑定后、退出前均[0,1,0,0,0]，强电第0项0。
- robot.get_id()=1，get_axis_number()=7。
- attach()返回0，但get_attached_id()返回0。
- attach前后check_error()均为[-5314 Error - Invalid robot ID]。
- detach()=0，disconnect()=0。

## 结论

机器人读取通道可用，但绑定读回不符合当前Python SDK文档所述“成功时等于机器人ID”。
attach返回0不能单独证明绑定成功。现有require_attached仅检查返回码、忽略绑定读回；
本次不沿用“恒为0所以正常”的未经证实假设继续上电。

本次完整动作链路在绑定预检处停止，未发送arm_demo，未验证真实运动。
诊断列表可能包含历史错误，因此仅凭此列表不能断定-5314是本次attach新产生的；
也不能仅据此断言固件不兼容或EMI引擎卡死。

后续须核实此旧SDK/当前固件组合下绑定ID查询语义与新生错误来源，
确认有效控制句柄及可可靠退出后，再继续点赞单动作→其余动作的真机验证。
