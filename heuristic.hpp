#pragma once

#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class SA {
  public:
	Solution bestSol;
	void init(const std::vector<Node*>& nodes, const std::vector<Node*>& station);
};

#endif