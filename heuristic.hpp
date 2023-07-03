#pragma once

#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class SA {
  public:
	Solution bestSol;  // 最好解
	Solution sol;      // 当前解
	void init(std::vector<const Node*>& nodes, const uint32_t depot_num, uint32_t maxload);
	void run();
};

#endif