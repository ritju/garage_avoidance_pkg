#include "garage_road_model/skeleton_extractor.hpp"

#include "garage_road_model/polygon_processor.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <utility>

namespace garage_road_model
{
namespace
{

using Pixel = std::pair<int, int>;

struct Chain
{
  std::vector<Pixel> nodes;
  double length{0.0};

  Pixel front() const {return nodes.front();}
  Pixel back() const {return nodes.back();}
};

const int kNeighborRow[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int kNeighborCol[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

bool pixelLess(const Pixel & left, const Pixel & right)
{
  return left < right;
}

bool isSkeleton(const cv::Mat & skeleton, int row, int col)
{
  return row >= 0 && col >= 0 && row < skeleton.rows && col < skeleton.cols &&
         skeleton.at<uint8_t>(row, col) != 0;
}

std::vector<Pixel> neighborsOf(const cv::Mat & skeleton, const Pixel & pixel)
{
  std::vector<Pixel> result;
  for (int index = 0; index < 8; ++index) {
    const int row = pixel.first + kNeighborRow[index];
    const int col = pixel.second + kNeighborCol[index];
    if (isSkeleton(skeleton, row, col)) {
      result.emplace_back(row, col);
    }
  }
  return result;
}

std::vector<Pixel> graphNeighbors(const cv::Mat & skeleton, const Pixel & pixel)
{
  auto raw = neighborsOf(skeleton, pixel);
  std::vector<Pixel> kept;
  for (const auto & neighbor : raw) {
    const int drow = neighbor.first - pixel.first;
    const int dcol = neighbor.second - pixel.second;
    if (std::abs(drow) + std::abs(dcol) == 1) {
      kept.push_back(neighbor);
      continue;
    }
    const Pixel horizontal{pixel.first, neighbor.second};
    const Pixel vertical{neighbor.first, pixel.second};
    const bool shortcut = isSkeleton(skeleton, horizontal.first, horizontal.second) ||
      isSkeleton(skeleton, vertical.first, vertical.second);
    if (!shortcut) {
      kept.push_back(neighbor);
    }
  }
  return kept;
}

double pixelDistance(const Pixel & a, const Pixel & b, double resolution)
{
  const double drow = static_cast<double>(a.first - b.first);
  const double dcol = static_cast<double>(a.second - b.second);
  return std::hypot(drow, dcol) * resolution;
}

double chainLength(const std::vector<Pixel> & nodes, double resolution)
{
  double length = 0.0;
  for (size_t i = 1; i < nodes.size(); ++i) {
    length += pixelDistance(nodes[i - 1], nodes[i], resolution);
  }
  return length;
}

std::map<Pixel, std::vector<Pixel>> buildAdjacency(const cv::Mat & skeleton)
{
  std::map<Pixel, std::vector<Pixel>> adjacency;
  for (int row = 0; row < skeleton.rows; ++row) {
    for (int col = 0; col < skeleton.cols; ++col) {
      if (!isSkeleton(skeleton, row, col)) {
        continue;
      }
      Pixel pixel{row, col};
      adjacency[pixel] = graphNeighbors(skeleton, pixel);
    }
  }
  return adjacency;
}

std::vector<Chain> extractChains(
  const std::map<Pixel, std::vector<Pixel>> & adjacency,
  double resolution)
{
  std::map<Pixel, int> degree;
  for (const auto & item : adjacency) {
    degree[item.first] = static_cast<int>(item.second.size());
  }

  std::map<std::pair<Pixel, Pixel>, bool> used;
  auto mark = [&](const Pixel & a, const Pixel & b) {
      Pixel left = a;
      Pixel right = b;
      if (pixelLess(right, left)) {
        std::swap(left, right);
      }
      used[{left, right}] = true;
    };
  auto isUsed = [&](const Pixel & a, const Pixel & b) {
      Pixel left = a;
      Pixel right = b;
      if (pixelLess(right, left)) {
        std::swap(left, right);
      }
      return used.count({left, right}) != 0;
    };

  std::vector<Chain> chains;
  auto walk = [&](const Pixel & start, const Pixel & next) {
      std::vector<Pixel> nodes{start, next};
      mark(start, next);
      Pixel previous = start;
      Pixel current = next;
      for (int guard = 0; guard < 1000000; ++guard) {
        if (degree[current] != 2 || current == start) {
          break;
        }
        Pixel upcoming{-1, -1};
        bool found = false;
        for (const auto & neighbor : adjacency.at(current)) {
          if (neighbor != previous) {
            upcoming = neighbor;
            found = true;
            break;
          }
        }
        if (!found || isUsed(current, upcoming)) {
          break;
        }
        nodes.push_back(upcoming);
        mark(current, upcoming);
        previous = current;
        current = upcoming;
      }
      Chain chain;
      chain.nodes = std::move(nodes);
      chain.length = chainLength(chain.nodes, resolution);
      chains.push_back(std::move(chain));
    };

  for (const auto & item : adjacency) {
    if (degree[item.first] == 2) {
      continue;
    }
    for (const auto & neighbor : item.second) {
      if (!isUsed(item.first, neighbor)) {
        walk(item.first, neighbor);
      }
    }
  }

  for (const auto & item : adjacency) {
    if (degree[item.first] != 2) {
      continue;
    }
    for (const auto & neighbor : item.second) {
      if (!isUsed(item.first, neighbor)) {
        walk(item.first, neighbor);
        break;
      }
    }
  }
  return chains;
}

void dropParallelBubbles(std::vector<Chain> & chains)
{
  std::map<std::pair<Pixel, Pixel>, std::vector<int>> groups;
  for (int index = 0; index < static_cast<int>(chains.size()); ++index) {
    Pixel a = chains[index].front();
    Pixel b = chains[index].back();
    if (pixelLess(b, a)) {
      std::swap(a, b);
    }
    groups[{a, b}].push_back(index);
  }
  std::vector<char> drop(chains.size(), 0);
  for (const auto & group : groups) {
    if (group.second.size() < 2) {
      continue;
    }
    int keep = group.second.front();
    for (int index : group.second) {
      if (chains[index].length > chains[keep].length) {
        keep = index;
      }
    }
    for (int index : group.second) {
      if (index != keep) {
        drop[index] = 1;
      }
    }
  }
  std::vector<Chain> kept;
  for (size_t index = 0; index < chains.size(); ++index) {
    if (!drop[index]) {
      kept.push_back(std::move(chains[index]));
    }
  }
  chains.swap(kept);
}

std::map<Pixel, int> endpointDegree(const std::vector<Chain> & chains)
{
  std::map<Pixel, int> degree;
  for (const auto & chain : chains) {
    if (chain.nodes.size() < 2) {
      continue;
    }
    if (chain.front() == chain.back()) {
      degree[chain.front()] += 2;
    } else {
      degree[chain.front()] += 1;
      degree[chain.back()] += 1;
    }
  }
  return degree;
}

void contractShortBridges(std::vector<Chain> & chains, double resolution, double max_length)
{
  const double limit = std::max(max_length, 4.0 * resolution);
  bool changed = true;
  int guard = 0;
  while (changed && guard < 32) {
    changed = false;
    ++guard;
    auto degree = endpointDegree(chains);
    for (size_t index = 0; index < chains.size(); ++index) {
      const auto & chain = chains[index];
      if (chain.front() == chain.back()) {
        continue;
      }
      if (chain.length > limit) {
        continue;
      }
      if (degree[chain.front()] < 3 || degree[chain.back()] < 3) {
        continue;
      }
      const Pixel keep = pixelLess(chain.front(), chain.back()) ? chain.front() : chain.back();
      const Pixel drop = keep == chain.front() ? chain.back() : chain.front();
      chains.erase(chains.begin() + static_cast<std::ptrdiff_t>(index));
      for (auto & other : chains) {
        for (auto & node : other.nodes) {
          if (node == drop) {
            node = keep;
          }
        }
        other.length = chainLength(other.nodes, resolution);
      }
      changed = true;
      break;
    }
  }

  std::vector<Chain> cleaned;
  for (auto & chain : chains) {
    std::vector<Pixel> nodes;
    for (const auto & node : chain.nodes) {
      if (nodes.empty() || nodes.back() != node) {
        nodes.push_back(node);
      }
    }
    if (nodes.size() < 2) {
      continue;
    }
    chain.nodes = std::move(nodes);
    chain.length = chainLength(chain.nodes, resolution);
    cleaned.push_back(std::move(chain));
  }
  chains.swap(cleaned);
}

void mergeDegreeTwo(std::vector<Chain> & chains, double resolution)
{
  bool changed = true;
  int guard = 0;
  while (changed && guard < 64) {
    changed = false;
    ++guard;
    auto degree = endpointDegree(chains);
    for (const auto & item : degree) {
      if (item.second != 2) {
        continue;
      }
      std::vector<int> hits;
      for (int index = 0; index < static_cast<int>(chains.size()); ++index) {
        if (chains[index].front() == item.first || chains[index].back() == item.first) {
          hits.push_back(index);
        }
      }
      if (hits.size() != 2) {
        continue;
      }
      Chain left = chains[hits[0]];
      Chain right = chains[hits[1]];
      if (left.back() != item.first) {
        std::reverse(left.nodes.begin(), left.nodes.end());
      }
      if (right.front() != item.first) {
        std::reverse(right.nodes.begin(), right.nodes.end());
      }
      if (left.back() != item.first || right.front() != item.first) {
        continue;
      }
      left.nodes.insert(left.nodes.end(), right.nodes.begin() + 1, right.nodes.end());
      left.length = chainLength(left.nodes, resolution);
      const int first = std::min(hits[0], hits[1]);
      const int second = std::max(hits[0], hits[1]);
      chains.erase(chains.begin() + second);
      chains.erase(chains.begin() + first);
      chains.push_back(std::move(left));
      changed = true;
      break;
    }
  }
}

double medianHalfWidth(
  const std::vector<Pixel> & pixels,
  const cv::Mat & distance,
  double resolution)
{
  std::vector<double> widths;
  widths.reserve(pixels.size());
  for (const auto & pixel : pixels) {
    widths.push_back(distance.at<float>(pixel.first, pixel.second) * resolution);
  }
  if (widths.empty()) {
    return resolution;
  }
  std::nth_element(widths.begin(), widths.begin() + widths.size() / 2, widths.end());
  return std::max(widths[widths.size() / 2], resolution);
}

void pruneSpurs(
  std::vector<Chain> & chains,
  const cv::Mat & distance,
  double resolution,
  const ModelConfig & config)
{
  std::vector<Pixel> all;
  for (const auto & chain : chains) {
    all.insert(all.end(), chain.nodes.begin(), chain.nodes.end());
  }
  const double half_width = medianHalfWidth(all, distance, resolution);
  const double length_limit = std::max(config.spur_min_length, config.spur_width_factor * half_width);
  const double end_radius = config.real_end_radius_ratio * half_width;

  bool changed = true;
  int guard = 0;
  while (changed && guard < 32) {
    changed = false;
    ++guard;
    auto degree = endpointDegree(chains);
    for (size_t index = 0; index < chains.size(); ++index) {
      const auto & chain = chains[index];
      if (chain.front() == chain.back()) {
        continue;
      }
      const bool front_leaf = degree[chain.front()] <= 1;
      const bool back_leaf = degree[chain.back()] <= 1;
      if (front_leaf == back_leaf) {
        continue;
      }
      const Pixel leaf = front_leaf ? chain.front() : chain.back();
      const double radius = distance.at<float>(leaf.first, leaf.second) * resolution;
      if (chain.length <= length_limit && radius < end_radius) {
        chains.erase(chains.begin() + static_cast<std::ptrdiff_t>(index));
        changed = true;
        break;
      }
    }
  }
}

std::vector<Chain> selectTrunkChains(
  std::vector<Chain> chains,
  const cv::Mat & distance,
  double resolution,
  const ModelConfig & config)
{
  dropParallelBubbles(chains);
  mergeDegreeTwo(chains, resolution);
  contractShortBridges(chains, resolution, config.bridge_max_length);
  mergeDegreeTwo(chains, resolution);
  dropParallelBubbles(chains);
  pruneSpurs(chains, distance, resolution, config);
  mergeDegreeTwo(chains, resolution);
  dropParallelBubbles(chains);
  if (chains.size() != 1) {
    throw ModelError(
      "主干选择", "主干存在歧义，剩余 " + std::to_string(chains.size()) + " 段");
  }
  if (chains.front().front() == chains.front().back()) {
    throw ModelError("主干选择", "主干存在歧义，骨架成环");
  }
  return chains;
}

Pixel climbToRidge(Pixel pixel, const cv::Mat & distance)
{
  for (int step = 0; step < 40; ++step) {
    Pixel best = pixel;
    float best_distance = distance.at<float>(pixel.first, pixel.second);
    for (int index = 0; index < 8; ++index) {
      const int row = pixel.first + kNeighborRow[index];
      const int col = pixel.second + kNeighborCol[index];
      if (row < 0 || col < 0 || row >= distance.rows || col >= distance.cols) {
        continue;
      }
      const float value = distance.at<float>(row, col);
      if (value > best_distance + 0.05F) {
        best_distance = value;
        best = Pixel{row, col};
      }
    }
    if (best == pixel) {
      break;
    }
    pixel = best;
  }
  return pixel;
}

void orientChain(Chain & chain, const GridGeometry & grid)
{
  const Eigen::Vector2d start = grid.pixelToWorld(chain.front().first, chain.front().second);
  const Eigen::Vector2d end = grid.pixelToWorld(chain.back().first, chain.back().second);
  if (std::tie(end.x(), end.y()) < std::tie(start.x(), start.y())) {
    std::reverse(chain.nodes.begin(), chain.nodes.end());
  }
}

std::vector<Eigen::Vector2d> extendTowardExit(
  const std::vector<Eigen::Vector2d> & trunk,
  bool at_start,
  const std::vector<Eigen::Vector2d> & polygon,
  const ModelConfig & config)
{
  if (trunk.size() < 2) {
    throw ModelError("端部延伸", "端部切向退化");
  }
  const size_t origin_index = at_start ? 0 : trunk.size() - 1;
  size_t cursor = origin_index;
  double looked = 0.0;
  while (looked < 1.0) {
    if (at_start) {
      if (cursor + 1 >= trunk.size()) {
        break;
      }
      looked += (trunk[cursor + 1] - trunk[cursor]).norm();
      ++cursor;
    } else {
      if (cursor == 0) {
        break;
      }
      looked += (trunk[cursor] - trunk[cursor - 1]).norm();
      --cursor;
    }
  }
  const Eigen::Vector2d origin = trunk[origin_index];
  Eigen::Vector2d direction = origin - trunk[cursor];
  if (direction.norm() < 1e-6) {
    throw ModelError("端部延伸", "端部切向退化");
  }
  direction.normalize();
  std::vector<Eigen::Vector2d> extra;
  Eigen::Vector2d point = origin;
  bool reached = false;
  for (int step = 0; step < 800; ++step) {
    const Eigen::Vector2d next = point + config.end_extend_step * direction;
    if (!pointInsidePolygon(next, polygon, 0.0)) {
      Eigen::Vector2d inside = point;
      Eigen::Vector2d outside = next;
      for (int refine = 0; refine < 12; ++refine) {
        const Eigen::Vector2d middle = 0.5 * (inside + outside);
        if (pointInsidePolygon(middle, polygon, 0.0)) {
          inside = middle;
        } else {
          outside = middle;
        }
      }
      if ((inside - point).norm() > 1e-4) {
        extra.push_back(inside);
      }
      reached = true;
      break;
    }
    point = next;
    extra.push_back(point);
  }
  if (!reached) {
    throw ModelError("端部延伸", "端部覆盖不足");
  }
  return extra;
}

}  // namespace

TrunkExtraction extractTrunk(
  const cv::Mat & mask,
  const GridGeometry & grid,
  const std::vector<Eigen::Vector2d> & polygon,
  const ModelConfig & config)
{
  cv::Mat skeleton;
  cv::ximgproc::thinning(mask, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
  cv::Mat distance;
  cv::distanceTransform(mask, distance, cv::DIST_L2, 3);

  cv::Mat components;
  const int component_count = cv::connectedComponents(skeleton, components, 8, CV_32S) - 1;
  if (component_count <= 0) {
    throw ModelError("骨架提取", "骨架为空");
  }
  if (component_count > 1) {
    throw ModelError("骨架提取", "骨架不连通");
  }

  auto adjacency = buildAdjacency(skeleton);
  auto chains = extractChains(adjacency, grid.resolution);
  if (chains.empty()) {
    throw ModelError("骨架提取", "骨架为空");
  }
  chains = selectTrunkChains(std::move(chains), distance, grid.resolution, config);
  for (auto & pixel : chains.front().nodes) {
    pixel = climbToRidge(pixel, distance);
  }
  {
    std::vector<Pixel> centered;
    for (const auto & pixel : chains.front().nodes) {
      if (centered.empty() || centered.back() != pixel) {
        centered.push_back(pixel);
      }
    }
    chains.front().nodes = std::move(centered);
  }
  orientChain(chains.front(), grid);

  TrunkExtraction result;
  result.skeleton.reserve(adjacency.size());
  for (const auto & item : adjacency) {
    result.skeleton.push_back(grid.pixelToWorld(item.first.first, item.first.second));
  }
  result.trunk.reserve(chains.front().nodes.size());
  for (const auto & pixel : chains.front().nodes) {
    result.trunk.push_back(grid.pixelToWorld(pixel.first, pixel.second));
  }
  if (result.trunk.size() < 2) {
    throw ModelError("主干选择", "主干点过少");
  }

  const auto head = extendTowardExit(result.trunk, true, polygon, config);
  const auto tail = extendTowardExit(result.trunk, false, polygon, config);
  std::vector<Eigen::Vector2d> extended;
  extended.reserve(head.size() + result.trunk.size() + tail.size());
  extended.insert(extended.end(), head.rbegin(), head.rend());
  extended.insert(extended.end(), result.trunk.begin(), result.trunk.end());
  extended.insert(extended.end(), tail.begin(), tail.end());
  result.trunk = resamplePolyline(extended, config.resample_spacing);
  if (result.trunk.size() < 4) {
    throw ModelError("主干选择", "主干点过少");
  }
  return result;
}

}  // namespace garage_road_model
