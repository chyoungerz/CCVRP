#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <vector>

#include "node.hpp"

namespace ALNS {

	/// @brief 修复算子
	/// @param solution 解
	/// @param rest 需要插入的节点（pair<厂站序号， 节点>）
	void repair(Solution& solution, std::vector<std::pair<u32, Node*>>& rest);

	/// @brief 破坏算子(最差优先)
	/// @param solution 解
	/// @param p 破坏的比例
	/// @param rest 破坏下来的节点（pair<厂站序号， 节点>）
	void destory_wst(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest);

	/// @brief 破坏算子(随机）
	/// @param solution 解
	/// @param p 破坏的比例
	/// @param rest 破坏下来的节点（pair<厂站序号， 节点>）
	void destory_rnd(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest);
}  // namespace ALNS

namespace LNS {
	void run(Solution& solution, const std::vector<Node*>& nodes, u32 epoch);

}  // namespace LNS

namespace LS {
	void one(Solution& solution, bool& flag);
	void two(Solution& solution, bool& flag);
	void three(Solution& solution, bool& flag);
	void arc(Solution& solution, bool& flag);
	void twoOpt(Solution& solution, bool& flag);
}  // namespace LS

namespace VNS {
	/// @brief 计算节点的邻域
	void neighbor(std::vector<Node*>& node, u32 size);

	/// @brief 点重定位操作
	void relocate(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 2-opt操作
	void twoopt(Solution& solution, u32& num, bool& flag);

	/// @brief 两点交换操作
	void exchange(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 点边交换操作
	void arcnode(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 边交换操作
	void arcswap(Solution& solution, u32& num, bool& flag);

	/// @brief oropt 2 操作
	void oropt2(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief oropt 3 操作
	void oropt3(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief oropt 4 操作
	void oropt4(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 交叉扰动优化操作
	void cross(Solution& solution, u32& num, bool& flag);
};  // namespace VNS
#endif