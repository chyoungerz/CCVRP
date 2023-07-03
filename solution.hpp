#pragma once

#ifndef _SOLUTION_HPP
#define _SOLUTION_HPP
#include <vector>

#include "node.hpp"

// 使用CW（Clarke and Wright）算法构造初始解
Solution cw(const std::vector<Node>& nodes, const std::vector<Node>& station);

// 基于贪婪策略的最近邻算法构造初始解
Solution greedynear(std::vector<const Node*>& nodes, const uint32_t depot_num, const uint32_t maxload);

// 基于贪婪策略的客户分配构造初始解
Solution nassign(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const int maxload);

// 扫描法构造初始解
Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station);
#endif  // _SOLUTION_HPP