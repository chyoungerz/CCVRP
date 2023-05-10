#pragma once

#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"
#include <eigen3/Eigen/Core>
//#include "threadpool.h"

//ILS算法
namespace ILS {
	// 迭代局部搜索，
	void init(std::vector<Node>& nodes, const unsigned int num, const unsigned int epoch);
}

////ALNS算法
namespace ALNS {
	void init(std::vector<Vehicle>& vehicles, const unsigned int num, const unsigned int epoch);
}

#endif