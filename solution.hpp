#pragma once

#ifndef _SOLUTION_HPP
#define _SOLUTION_HPP
#include <vector>

#include "node.hpp"

// 使用CW（Clarke and Wright）算法构造初始解
Solution cw(std::vector<const Node*>& nodes, const u32 depot_num, const u32 maxload, const u32 routes);

// 基于贪婪策略的客户分配构造初始解
Solution greedynear(std::vector<const Node*>& nodes, const u32 depot_num, const u32 maxload);

// 基于贪婪策略的客户分配构造初始解
Solution nassign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes);

// 扫描法构造初始解
Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station);
#endif  // _SOLUTION_HPP