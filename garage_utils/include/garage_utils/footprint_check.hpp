#ifndef GARAGE_UTILS__FOOTPRINT_CHECK_HPP_
#define GARAGE_UTILS__FOOTPRINT_CHECK_HPP_

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace garage_utils_pkg
{
// 对一次请求持有的地图快照检查整车及连接段。
class FootprintCheck
{
public:
  FootprintCheck(const nav2_msgs::msg::Costmap & map,
    const geometry_msgs::msg::Polygon & footprint);
  bool motionFree(const geometry_msgs::msg::Pose & from,
    const geometry_msgs::msg::Pose & to, std::string & reason) const;
private:
  const nav2_msgs::msg::Costmap & map_;
  geometry_msgs::msg::Polygon footprint_;
  double radius_{0.0};
};
}
#endif
