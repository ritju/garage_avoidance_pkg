#ifndef GARAGE_UTILS__TYPES_HPP_
#define GARAGE_UTILS__TYPES_HPP_

#include <vector>
#include <cstdio>

namespace garage_utils_pkg
{
        #define Point std::pair<double, double>

        struct EnhancedPoint
        {
                int index;
                int edge_index;
                std::vector<int> adjacent_vec;
                bool visited;
                Point coord;
        };

        enum class NeighborType
        {
                POINT_POINT,
                POINT_LINE
        };

        // 顶点结构体（包含极角信息）
        struct Vertex 
        {
                double x, y;
                double angle; // 相对于矩形中心的极角
        };

        struct Edge
        {
                int index;
                int pt1_index;
                int pt2_index;
                int rect_index;
                Point pt1;
                Point pt2;
                double dis;
                std::vector<int> adjacent_vec;
                bool has_child = false;
                bool is_child = false;
                int child1_index;
                int child2_index;
                int sibling_index;
        };

        enum class SearchMethod
        {
                DepthFirstSearch
        };



        // 矩形结构体（增强版）
        struct EnhancedRect 
        {
        std::vector<Vertex> vertices;                             // 顶点集合（需排序后使用）
        std::vector<std::pair<size_t, size_t>> edges;             // 边集合（存储顶点索引）
        std::vector<size_t> adjacent_rects;                       // 相邻矩形索引列表
        size_t adjacent_rect_selected;                            // 最终选择的下个矩形索引值， -1代表最后一个
        bool visited = false;                                     // 遍历标记
        size_t index;                                             // 矩形索引值，用于调试矩形排序结果
        std::vector<std::pair<double, double>> middle_long_line;  // 矩形两条长边决定的遍历过程要覆盖的路径
        };
} // end of namespace

#endif

