#pragma once

#ifndef _SOLUTION_HPP
#define _SOLUTION_HPP
#include <vector>

#include "node.hpp"

// 使用CW（Clarke and Wright）算法构造初始解
Solution cw(const std::vector<Node>& nodes, const std::vector<Node>& station);

// 最近邻算法构造初始解
Solution knn(const std::vector<Node>& nodes, const std::vector<Node>& station);

// 最近插入法构造初始解
Solution NInsertion(const std::vector<Node>& nodes, const std::vector<Node>& station);

// 扫描法构造初始解
Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station);
#endif  // _SOLUTION_HPP