#pragma once

#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class SA {
  public:
	Solution bestSol;  // 最好解
	Solution sol;      // 当前解
	void init(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const char cmd);
	void run();
};

#endif