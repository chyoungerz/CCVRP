#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <string>
#include <vector>

#include "node.hpp"

namespace LS {

	/// @brief 两点优化
	/// @param solution 解
	/// @param stop 达到最大停止次数
	/// @return true，false
	bool twoPoint(Solution& solution, uint32_t stop);
}  // namespace LS

#endif