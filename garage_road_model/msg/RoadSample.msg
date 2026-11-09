# 长边上的一个采样点 header.frame_id  map，长度单位米，角度单位弧度
float64 x           # 采样点 x
float64 y           # 采样点 y
float64 yaw         # 切线朝向，沿本侧边从 start_cap 指向 end_cap
float64 s           # 从本侧边起点累计的弧长，严格递增
bool corner         # 分段拼接处的拐角；该处没有唯一朝内法线   其实是折角上，进来的切线和出去的切线各转 90°，得到两个朝内方向
