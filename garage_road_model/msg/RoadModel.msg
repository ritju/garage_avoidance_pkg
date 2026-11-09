# 一条通道多边形拟合出的道路模型。
uint32 source_index              # 对应输入多边形数组的下标
geometry_msgs/Polygon boundary   # 校验、简化后的通道边界顶点
geometry_msgs/Polygon start_cap  # 起点短边；两侧弧长 s 从这里起算
geometry_msgs/Polygon end_cap    # 终点短边
SideBoundary side_a              # 长边 A，方向 start_cap -> end_cap
SideBoundary side_b              # 长边 B，已反向，同样 start_cap -> end_cap
bool valid                       # 拟合成功
string stage                     # 当前阶段；成功为「完成」，开始拟合前为「建模」
string reason                    # 失败原因；成功时为空
