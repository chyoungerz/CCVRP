#pragma once

#include <vector>
#ifndef _OPERATOR_HPP_
#define _OPERATOR_HPP_

#include "node.hpp"

/// @brief 节点操作
namespace OP {
	//-------------------- node opt --------------------//

	/// @brief 插入节点后
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insertb(Vehicle& vehicle, Node* node, const u32 pos, const double diflength);

	/// @brief 插入节点前
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insertf(Vehicle& vehicle, Node* node, const u32 pos, const double diflength);

	/// @brief 插入节点,非开头末尾
	/// @param vehicle 待插入的路线（车辆）
	/// @param node 插入节点
	/// @param pos 位置（之后）[1, size-1)
	/// @param diflength 差值
	void insert(Vehicle& vehicle, Node* node, const u32 pos, const double diflength);

	/// @brief 末尾删除
	/// @param vehicle 待删除的路线（车辆）
	/// @return true -> 删除的节点指针，false -> nullptr
	Node* removeback(Vehicle& vehicle);

	/// @brief 开头删除
	/// @param vehicle 待删除的路线（车辆）
	/// @return true -> 删除的节点指针，false -> nullptr
	Node* removefront(Vehicle& vehicle);

	/// @brief 删除节点, 非开头和末尾
	/// @param vehicle 待删除的路线（车辆）
	/// @param pos 位置 [1, size-1)
	/// @param diflength 差值
	/// @return 删除的节点指针
	Node* erase(Vehicle& vehicle, const u32 pos, const double diflength);

	/// @brief 删除无用节点，配合remove使用
	/// @param vehicle 待去除的路线（车辆）
	void erase(Vehicle& vehicle);

	/// @brief 去除节点(将位置替换为首节点), 非开头和末尾
	/// @param vehicle 待去除的路线（车辆）
	/// @param pos 位置 [1, size-1)
	/// @param diflength 差值
	/// @return 删除的节点指针
	Node* remove(Vehicle& vehicle, const u32 pos, const double diflength);

	/// @brief 两点交换算子(使用邻域)
	/// @param sol 解
	void twoNSwap(Solution& sol);

	/// @brief 一点移动算子(使用邻域)
	/// @param sol 解
	/// @param nodes 节点
	/// @param depot_num 厂站
	void oneNMove(Solution& sol, std::vector<Node*>& nodes, const u32 depot_num);

	/// @brief 一点移动算子
	/// @param vehicle_a
	/// @param vehicle_b
	/// @param flag
	void oneMove(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag);

	/// @brief 两点交换算子
	/// @param vehicle_a
	/// @param vehicle_b
	/// @param flag
	void twoSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag);

	/// @brief 2-opt
	/// @param sol
	void twoOpt(Solution& sol);

	/// @brief
	/// @param vehicle_a
	/// @param vehicle_b
	/// @param flag
	void threeSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag);

	//-------------------- string opt --------------------//

	/// @brief 反转路线（车辆）
	/// @param vehicle 待反转的路线（车辆）
	/// @param from_pos 开始位置 [1, size-1)
	/// @param to_pos 结束位置 [1, size-1)
	/// @return true，false
	bool reverse(Vehicle& vehicle, const u32 from_pos, const u32 to_pos);

	/// @brief 交换两路线（车辆）
	/// @param vehicle_a 待交换的路线（车辆）A
	/// @param vehicle_b 待交换的路线（车辆）B
	/// @param from_a_pos 路线（车辆）A 开始位置 [1, size-1)
	/// @param to_a_pos 路线（车辆）A 结束位置 [1, size-1)
	/// @param from_b_pos 路线（车辆）B 开始位置 [1, size-1)
	/// @param to_b_pos 路线（车辆）B 结束位置 [1, size-1)
	/// @return true，false
	bool twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 from_a_pos, const u32 to_a_pos, const u32 from_b_pos, const u32 to_b_pos);

}  // namespace OP
/*
/// @brief 计算评估操作的差值
/// @warning 已弃用
namespace COST {

    /// @brief 计算前插操作的差值
    /// @param vehicle 待插入的路线（车辆）
    /// @param node 插入的节点
    /// @param pos 位置（之前）[1, size-1)
    /// @return 差值
    double insertf(std::vector<Node*>& route, Node* node, const u32 pos);

    /// @brief 计算后插操作的差值
    /// @param vehicle 待插入的路线（车辆）
    /// @param node 插入的节点
    /// @param pos 位置（之后）[1, size-1)
    /// @return 差值
    double insertb(std::vector<Node*>& route, Node* node, const u32 pos);

    /// @brief 计算删除操作的差值
    /// @param vehicle 待删除的路线（车辆）
    /// @param pos 位置（之后）[1, size-1)
    /// @return 差值
    double erase(std::vector<Node*>& route, const u32 pos);

    /// @brief 计算交换同一路线（车辆）节点的差值, 开头和末尾为厂站
    /// @param vehicle 待交换的路线（车辆）
    /// @param pos_i 位置 i [1, size-1)
    /// @param pos_j 位置 j [1, size-1)
    /// @return 差值
    double swaptwo(Vehicle& vehicle, const u32 pos_i, const u32 pos_j);

    /// @brief 计算交换不同路线（车辆）两节点的差值, 开头和末尾为厂站
    /// @param vehicle_a 待交换的路线（车辆） A
    /// @param vehicle_b 待交换的路线（车辆） B
    /// @param pos_a A的位置 [1, size-1)
    /// @param pos_b B的位置 [1, size-1)
    /// @return pair<差值A, 差值B>
    std::pair<double, double> twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b);

    /// @brief 计算反转路线（车辆）的差值（未实现）
    /// @param vehicle
    /// @param from_pos
    /// @param to_pos
    /// @return 差值
    double reverse(Vehicle& vehicle, const u32 from_pos, const u32 to_pos);

    /// @brief 计算交换两路线（车辆）的差值（未实现）
    /// @param vehicle_a
    /// @param vehicle_b
    /// @param from_a_pos
    /// @param to_a_pos
    /// @param from_b_pos
    /// @param to_b_pos
    /// @return pair<差值A, 差值B>
    std::pair<double, double> twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 from_a_pos, const u32 to_a_pos, const u32 from_b_pos, const u32 to_b_pos);
}  // namespace COST
*/
namespace CHK {

	/// @brief 查找
	/// @param route
	/// @param seq(序号或指针)
	/// @return 位置
	u32 find(std::vector<Node*>& route, const u32 seq, u32 where);
	u32 find(std::vector<Node*>& route, Node* seq, u32 where);

	/// @brief 计算移动一个节点到同一路线（车辆）的差值, 开头和末尾为厂站。
	/// @param vehicle 待移动的路线（车辆）
	/// @param h 起始位置
	/// @param d 目标位置
	/// @param out_d 差值
	/// @return 是否可行。可行时已移动好
	/// @note 自动判断目标位置的前边和后边
	bool oneMove(Vehicle& vehicle, const u32 h, const u32 d, double& out_d);

	/// @brief 计算移动一个节点到不同路线（车辆）的差值, 开头和末尾为厂站。
	/// @param vehicle_h 待移动的起始路线（车辆）
	/// @param vehicle_d 待移动的目标路线（车辆）
	/// @param h 起始位置
	/// @param d 目标位置
	/// @param out_da 起始路线（车辆）差值
	/// @param out_db 目标路线（车辆）差值
	/// @return 是否可行。可行时已移动好
	/// @note 自动判断目标位置的前边和后边
	bool oneMove(Vehicle& vehicle_h, Vehicle& vehicle_d, const u32 h, const u32 d, double& out_da, double& out_db);

	/// @brief 计算交换同一路线（车辆）节点的差值, 开头和末尾为厂站
	/// @param vehicle 待交换的路线（车辆）
	/// @param pos_i 位置 i [1, size-1)
	/// @param pos_j 位置 j [1, size-1)
	/// @param out_d 路径差值
	/// @return 是否可行
	bool twoSwap(Vehicle& vehicle, const u32 pos_i, const u32 pos_j, double& out_d);

	/// @brief 计算交换不同路线（车辆）两节点的差值, 开头和末尾为厂站
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a A的位置 [1, size-1)
	/// @param pos_b B的位置 [1, size-1)
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行
	bool twoSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db);

	/// @brief 2-opt-cross
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a A的位置 [1, size-1)
	/// @param pos_b B的位置 [1, size-1)
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行, 可行时已交换好
	bool twoCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db);

	/// @brief
	/// @param vehicle_a
	/// @param vehicle_b
	/// @param pos_a_f
	/// @param pos_a_t
	/// @param pos_b_f
	/// @param pos_b_t
	/// @param out_da
	/// @param out_db
	/// @return
	/// @todo 未实现
	bool strCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_a_t, const u32 pos_b_f, const u32 pos_b_t, double& out_da, double& out_db);

	/// @brief 反转路径
	/// @param vehicle 待反转的路线
	/// @param pos_f 从
	/// @param pos_t 到
	/// @param out_d 差值
	/// @return 是否可行, 可行时已交换好
	/// @todo 待优化
	bool reverse(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, double& out_d);

	/// @brief 两路径交换
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a_f A的起始位置 [1, size-1)
	/// @param pos_b_f B的起始位置 [1, size-1)
	/// @param len_t 交换的长度
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行, 可行时已交换好
	/// @note 必须满足 pos_a_f + len_t < size() - 2 && pos_b_f + len_t < size() - 2
	bool strSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_b_f, const u32 len_t, double& out_da, double& out_db);

	/// @brief 两路径反转交换
	/// @param vehicle_a 待交换的路线（车辆） A
	/// @param vehicle_b 待交换的路线（车辆） B
	/// @param pos_a_f A的起始位置 [1, size-1)
	/// @param pos_b_f B的起始位置 [1, size-1)
	/// @param len_t 交换的长度
	/// @param out_da A的路径差值
	/// @param out_db B的路径差值
	/// @return 是否可行, 可行时已交换好
	/// @note 必须满足 pos_a_f + len_t < size() - 2 && pos_b_f + len_t < size() - 2
	bool strSwapR(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_b_f, const u32 len_t, double& out_da, double& out_db);

	/// @brief 连续的点移动
	/// @param vehicle 待移动的路线（车辆）
	/// @param pos_f 点起始位置
	/// @param pos_t 点终止位置
	/// @param pos 到
	/// @param out_da 路径差值
	/// @return 是否可行, 可行时已移动好
	/// @note 必须保证不满足 pos_f < pos < pos_t \n
	/// @note 自动判断目标位置的前边和后边
	bool strMove(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, const u32 pos, double& out_d);

	/// @brief 不同路径（车辆）连续的点移动
	/// @param vehicle_a 从路线（车辆）A
	/// @param vehicle_b 到路线（车辆）B
	/// @param pos_a_f A起始位置
	/// @param pos_a_t A终止位置
	/// @param pos_b 到B的位置
	/// @param out_da 路径差值A
	/// @param out_db 路径差值A
	/// @return 是否可行, 可行时已移动好
	/// @note 自动判断目标位置的前边和后边
	bool strMove(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_a_t, const u32 pos_b, double& out_da, double& out_db);

	/// @brief 点边交换，同路线（车辆）
	/// @param vehicle 待交换的路线（车辆）
	/// @param pos_f 从 [1, size-1)
	/// @param pos_t 到 (1, size - 2)
	/// @param out_d 路径差值
	/// @return 是否可行, 可行时已移动好
	/// @note 自动判断目标位置的前边和后边
	/// @warning 必须满足 1 < pos_t < size - 2 && pos_t - pos_f != 1
	bool PESwap(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, double& out_d);

	/// @brief 点边交换，不同路线（车辆）
	/// @param vehicle_a 从路线（车辆）A
	/// @param vehicle_b 到路线（车辆）B
	/// @param pos_a 从 [1, size-1)
	/// @param pos_b 到 (1, size - 2)
	/// @param out_da 路径差值A
	/// @param out_db 路径差值A
	/// @return 是否可行, 可行时已移动好
	/// @note 自动判断目标位置的前边和后边
	bool PESwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db);

}  // namespace CHK

/// @brief 扰动算子
/// @todo 破坏后的重建使用随机插入顺序，找到最好的插入顺序（重复大概50次），然后插入。
namespace PER {

	double cumlength(std::vector<Node*>& path);
	bool insert(Vehicle& vehicle, Node* node, u32 ctrl);
	bool insert(Solution& sol, Node* node, u32 ctrl);

	/// @brief 抛射链扰动
	/// @param sol 解
	/// @param k k条路径
	/// @param epoch 扰动最大失败次数
	void EjecChain(Solution& sol, u32 k, u32 epoch);

	/// @brief 破坏重建扰动
	/// @param sol 解
	/// @param k 破坏的邻域个数
	/// @param maxnode 最大节点数
	void RuinCreate(Solution& sol, u32 k, u32 maxnode);

	/// @brief 破坏重建优化扰动
	/// @param sol 解
	/// @param k 破坏的邻域个数
	/// @param maxnode 最大节点数
	/// @param epoch 最大停止次数
	void RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch, u32 rule);

	/// @brief 抛射链扰动
	/// @param sol 解
	/// @param k k条路径
	/// @param epoch 扰动最大失败次数
	void EjecChain(Solution& sol, u32 k, u32 epoch, u32 rule);
}  // namespace PER

namespace OPS {
	bool remove(Solution& s, std::vector<Node*> r, const u32 k, double& saving);
	bool remove(Solution& s, std::vector<Node*> r, const u32 k, const u32 len, double& saving);
	bool insert(Solution& s, std::vector<Node*> r1, std::vector<Node*> r2, const u32 k, const u32 len, double& saving, bool& location);
	bool insert(Solution& s, std::vector<Node*> r, const u32 k, Node* node, double& saving, bool& location);
	bool reverse(Solution& s, Vehicle& r, const u32 f, const u32 t, u32 ctrl);
	bool swapmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool onepointmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool onepointmove(Solution& s, Vehicle& r, const u32 a, const u32 b, u32 ctrl);
	bool oropt(Solution& s, Vehicle& r1, Vehicle& r2, const u32 f, const u32 t, const u32 len, u32 ctrl);
	bool oropt(Solution& s, Vehicle& r, const u32 f, const u32 t, const u32 len, u32 ctrl);
	bool arcnode(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool arcswap(Solution& s, Vehicle& r1, Vehicle& r2, float c, const u32 a, const u32 b, u32 ctrl);
};  // namespace OPS
#endif