# 通道的一条长边。side_a、side_b 都从 start_cap 走向 end_cap。
geometry_msgs/Polygon raw  # 切出来的原始长边
RoadSample[] samples       # 沿原始长边按弧长采样；失败时为空
float64 length             # 侧边弧长，等于最后一个采样的 s
