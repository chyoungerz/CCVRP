#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <map>
#include <string>
#include <vector>

#include "node.hpp"

namespace ALNS {

	/// @brief 修复算子
	/// @param solution 解
	/// @param rest 需要插入的节点（pair<厂站序号， 节点>）
	void repair(Solution& solution, std::vector<std::pair<uint32_t, const Node*>>& rest);

	/// @brief 破坏算子(最差优先)
	/// @param solution 解
	/// @param p 破坏的比例
	/// @param rest 破坏下来的节点（pair<厂站序号， 节点>）
	void destory_wst(Solution& solution, const float p, std::vector<std::pair<uint32_t, const Node*>>& rest);

	/// @brief 破坏算子(随机）
	/// @param solution 解
	/// @param p 破坏的比例
	/// @param rest 破坏下来的节点（pair<厂站序号， 节点>）
	void destory_rnd(Solution& solution, const float p, std::vector<std::pair<uint32_t, const Node*>>& rest);
}  // namespace ALNS

namespace LNS {
	void run(Solution& solution, const std::vector<const Node*>& nodes, uint32_t epoch);

}  // namespace LNS
#endif