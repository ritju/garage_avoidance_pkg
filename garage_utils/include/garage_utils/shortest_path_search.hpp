#ifndef GARAGE_UTILS__SHORTEST_PATH_SEARCH_HPP_
#define GARAGE_UTILS__SHORTEST_PATH_SEARCH_HPP_

#include "garage_utils/types.hpp"

namespace garage_utils_pkg
{
        
        class ShortestPathSearch
        {
        public:
                explicit ShortestPathSearch(int number);
                ~ShortestPathSearch();

                void add_edge(Edge edge);
                void add_all_points(std::vector<Point> points);
                int get_end_point();
                void dfs();
                std::vector<int> generate_path(std::vector<Point> points, std::vector<std::vector<double>> dis_);
        
        private:
                int number;
                std::vector<Point> points_;
                std::vector<std::vector<double>> dis_;
        };
} // end of namespace

#endif





