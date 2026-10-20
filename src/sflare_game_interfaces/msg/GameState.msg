# 定位数据存活状态："0" 无数据，"1" 最近超时时间内收到过数据。
string positioning_status
# 急停状态："0" 未急停，"1" 急停，"-1" 未知或读取失败。
string emergency_stop_status
# 关节限位状态："0" 正常，"1" 超限，"-1" 未知或读取失败。
string overtravel_status
# 当前流程状态，例如 idle、self_checking、moving 或 error。
string current_state
