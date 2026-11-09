# 一条通道多边形采样出的道路模型。
uint32 source_index              # 对应输入多边形数组的下标
geometry_msgs/Polygon start_cap  # 起点短边；两侧弧长 s 从这里起算
geometry_msgs/Polygon end_cap    # 终点短边
SideBoundary side_a              # 长边 A，方向 start_cap -> end_cap。采样为空表示还不能用
SideBoundary side_b              # 长边 B，已反向，同样 start_cap -> end_cap。采样为空表示还不能用
