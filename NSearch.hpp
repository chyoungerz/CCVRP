#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <string>
#include <vector>

#include "node.hpp"

namespace ALNS {
	void repair(Solution& solution);
	void destory(Solution& solution);
}  // namespace ALNS

#endif