# /garage_road_models 一次发布的全部通道。
std_msgs/Header header  # stamp 为发布时间，frame_id 为道路坐标 frame
uint64 revision         # 递增版本号；更新的通道到来时，旧版本结果会被丢掉
RoadModel[] roads       # 每个输入多边形一条；失败的也保留，valid 为 false
