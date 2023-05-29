#pragma once

#ifndef _OPERATION_HPP
#define _OPERATION_HPP

#include "node.hpp"

//-------------------- node opt --------------------//
namespace OP {

	/// @brief 末尾插入
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @return true，false
	bool insertback(Vehicle& vehicle, const Node* node);

	/// @brief 开头插入
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @return true，false
	bool insertfront(Vehicle& vehicle, const Node* node);

	/// @brief 插入节点,非开头末尾
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @return true，false
	bool insert(Vehicle& vehicle, const Node* node, const uint32_t pos);

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
	/// @return true -> 删除的节点指针，false -> nullptr
	const Node* remove(Vehicle& vehicle, const uint32_t pos);

	// 交换路线（车辆）两节点位置，成功返回true, 至少有四个节点， 开头和末尾为厂站。

	/// @brief 交换同一路线（车辆）节点, 开头和末尾为厂站
	/// @param vehicle 待交换的路线（车辆）
	/// @param pos_i 位置 i [1, size-1)
	/// @param pos_j 位置 j [1, size-1)
	/// @return true，false
	bool swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j);

	/// @brief 交换不同路线（车辆）两节点, 开头和末尾为厂站
	/// @param vehicle_a 待交换的路线（车辆）A
	/// @param vehicle_b 待交换的路线（车辆）B
	/// @param pos_a 路线（车辆）A 的位置 [1, size-1)
	/// @param pos_b 路线（车辆）B 的位置 [1, size-1)
	/// @return true，false
	bool twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b);

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

#endif