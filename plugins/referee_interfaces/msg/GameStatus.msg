# 比赛状态数据 (裁判系统串口协议 V1.7.0 0x0001)

# constants for game progress
# 未开始比赛 0
# 准备阶段 1
# 十五秒裁判系统自检阶段 2
# 五秒倒计时 3
# 比赛中 4
# 比赛结算中 5

uint8 game_progress
int32 stage_remain_time             # 当前阶段剩余时间，单位：秒