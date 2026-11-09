# 通道的一条长边。side_a、side_b 都从 start_cap 走向 end_cap。
geometry_msgs/Polygon raw  # 拟合前的原始折线
RoadSample[] samples       # 按弧长重采样后的曲线；拟合失败时为空
float64 length             # 侧边弧长，等于最后一个采样的 s
