#pragma once

#ifndef _OPERATOR_HPP
#define _OPERATOR_HPP

#include "node.hpp"

/// @brief 节点操作
namespace OP {
	//-------------------- node opt --------------------//

	/// @brief 插入节点后
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insertb(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength);

	/// @brief 插入节点前
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insertf(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength);

	/// @brief 插入节点,非开头末尾
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insert(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength);

	/// @brief 末尾删除
	/// @param vehicle 待删除的路线（车辆）
	/// @return true -> 删除的节点指针，false -> nullptr
	const Node* removeback(Vehicle& vehicle);

	/// @brief 开头删除
	/// @param vehicle 待删除的路线（车辆）
	/// @return true -> 删除的节点指针，false -> nullptr
	const Node* removefront(Vehicle& vehicle);

	/// @brief 删除节点, 非开头和末尾
	/// @param vehicle 待删除的路线（车辆）
	/// @param pos 位置 [1, size-1)
	/// @param diflength 差值
	/// @return 删除的节点指针
	const Node* erase(Vehicle& vehicle, const uint32_t pos, const double diflength);

	/// @brief 删除无用节点，配合remove使用
	/// @param vehicle 待去除的路线（车辆）
	void erase(Vehicle& vehicle);

	/// @brief 去除节点(将位置替换为首节点), 非开头和末尾
	/// @param vehicle 待去除的路线（车辆）
	/// @param pos 位置 [1, size-1)
	/// @param diflength 差值
	/// @return 删除的节点指针
	const Node* remove(Vehicle& vehicle, const uint32_t pos, const double diflength);

	/// @brief 两点交换算子(使用邻域)
	/// @param sol 解
	void twoNSwap(Solution& sol);

	/// @brief 一点移动算子(使用邻域)
	/// @param sol 解
	void oneNMove(Solution& sol);

	/// @brief 2-opt
	/// @param sol
	void twoOpt(Solution& sol);

	/// @brief 交换不同路线（车辆）两节点, 开头和末尾为厂站
	/// @param vehicle_a 待交换的路线（车辆）A
	/// @param vehicle_b 待交换的路线（车辆）B
	/// @param pos_a 路线（车辆）A 的位置 [1, size-1)
	/// @param pos_b 路线（车辆）B 的位置 [1, size-1)
	/// @param diflength pair<路线A差值, 路线B差值>
	void twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, const std::pair<double, double> diflength);

	//-------------------- string opt --------------------//

	/// @brief 反转路线（车辆）
	/// @param vehicle 待反转的路线（车辆）
	/// @param from_pos 开始位置 [1, size-1)
	/// @param to_pos 结束位置 [1, size-1)
	/// @return true，false
	bool reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos);

	/// @brief 交换两路线（车辆）
	/// @param vehicle_a 待交换的路线（车辆）A
	/// @param vehicle_b 待交换的路线（车辆）B
	/// @param from_a_pos 路线（车辆）A 开始位置 [1, size-1)
	/// @param to_a_pos 路线（车辆）A 结束位置 [1, size-1)
	/// @param from_b_pos 路线（车辆）B 开始位置 [1, size-1)
	/// @param to_b_pos 路线（车辆）B 结束位置 [1, size-1)
	/// @return true，false
	bool twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos);

}  // namespace OP

/// @brief 计算评估操作的差值
namespace COST {

	/// @brief 计算前插操作的差值
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入的节点
	/// @param pos 位置（之后）[1, size-1)
	/// @return 差值
	double insertf(std::vector<const Node*>& route, const Node* node, const uint32_t pos);

	/// @brief 计算后插操作的差值
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入的节点
	/// @param pos 位置（之后）[1, size-1)
	/// @return 差值
	double insertb(std::vector<const Node*>& route, const Node* node, const uint32_t pos);

	/// @brief 计算删除操作的差值
	/// @param vehicle 待删除的路线（车辆）
	/// @param pos 位置（之后）[1, size-1)
	/// @return 差值
	double erase(std::vector<const Node*>& route, const uint32_t pos);

	/// @brief 计算交换同一路线（车辆）节点的差值, 开头和末尾为厂站
	/// @param vehicle 待交换的路线（车辆）
	/// @param pos_i 位置 i [1, size-1)
	/// @param pos_j 位置 j [1, size-1)
	/// @return 差值
	double swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j);

	/// @brief 计算交换不同路线（车辆）两节点的差值, 开头和末尾为厂站
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a A的位置 [1, size-1)
	/// @param pos_b B的位置 [1, size-1)
	/// @return pair<差值A, 差值B>
	std::pair<double, double> twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b);

	/// @brief 计算反转路线（车辆）的差值（未实现）
	/// @param vehicle
	/// @param from_pos
	/// @param to_pos
	/// @return 差值
	double reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos);

	/// @brief 计算交换两路线（车辆）的差值（未实现）
	/// @param vehicle_a
	/// @param vehicle_b
	/// @param from_a_pos
	/// @param to_a_pos
	/// @param from_b_pos
	/// @param to_b_pos
	/// @return pair<差值A, 差值B>
	std::pair<double, double> twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos);
}  // namespace COST

namespace CHK {

	/// @brief 查找
	/// @param route
	/// @param seq(序号或指针)
	/// @return 位置
	uint32_t find(std::vector<const Node*>& route, const uint32_t seq);
	uint32_t find(std::vector<const Node*>& route, const Node* seq);

	/// @brief 计算移动发生在同一路线（车辆）的差值, 开头和末尾为厂站，默认后插
	/// @param vehicle 待移动的路线（车辆）
	/// @param h 起始位置
	/// @param d 目标位置
	/// @param out_d 差值
	/// @return 是否可行
	bool move(Vehicle& vehicle, const uint32_t h, const uint32_t d, double& out_d);

	/// @brief 计算移动发生在不同路线（车辆）的差值, 开头和末尾为厂站,默认后插
	/// @param vehicle_h 待移动的起始路线（车辆）
	/// @param vehicle_d 待移动的目标路线（车辆）
	/// @param h 起始位置
	/// @param d 目标位置
	/// @param out_da 起始路线（车辆）差值
	/// @param out_db 目标路线（车辆）差值
	/// @return 是否可行
	bool move(Vehicle& vehicle_h, Vehicle& vehicle_d, const uint32_t h, const uint32_t d, double& out_da, double& out_db);

	/// @brief 计算交换同一路线（车辆）节点的差值, 开头和末尾为厂站
	/// @param vehicle 待交换的路线（车辆）
	/// @param pos_i 位置 i [1, size-1)
	/// @param pos_j 位置 j [1, size-1)
	/// @param out_d 路径差值
	/// @return 是否可行
	bool swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j, double& out_d);

	/// @brief 计算交换不同路线（车辆）两节点的差值, 开头和末尾为厂站
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a A的位置 [1, size-1)
	/// @param pos_b B的位置 [1, size-1)
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行
	bool twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, double& out_da, double& out_db);

	/// @brief 2-opt-cross
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a A的位置 [1, size-1)
	/// @param pos_b B的位置 [1, size-1)
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行, 可行时已交换好
	bool twoCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, double& out_da, double& out_db);

}  // namespace CHK
#endif