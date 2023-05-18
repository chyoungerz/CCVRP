#pragma once

#ifndef _OPERATION_HPP
#define _OPERATION_HPP

#include "node.hpp"

//-------------------- node opt --------------------//
// 在路线（车辆）vehicle的末尾插入节点node，成功返回true
bool insertback(Vehicle& vehicle, const Node* node);

// 在路线（车辆）vehicle的开头插入节点node，成功返回true
bool insertfront(Vehicle& vehicle, const Node* node);

// 在路线（车辆）vehicle的pos位置后插入节点node，成功返回true (非开头和末尾)
bool insert(Vehicle& vehicle, const Node* node, const uint32_t pos);

// 删除路线（车辆）vehicle的末尾位置节点node，成功返回删除的节点指针，失败返回nullptr
const Node* removeback(Vehicle& vehicle);

// 删除路线（车辆）vehicle的开头位置节点node，成功返回删除的节点指针，失败返回nullptr
const Node* removefront(Vehicle& vehicle);

// 删除路线（车辆）vehicle的pos位置节点node，成功返回删除的节点指针，失败返回nullptr， (非开头和末尾)
const Node* remove(Vehicle& vehicle, const uint32_t pos);

// 交换路线（车辆）两节点位置，成功返回true, 至少有四个节点， 开头和末尾为厂站。
bool swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j);

// 交换两路线（车辆）两节点位置，成功返回true， 至少有三个节点， 开头和末尾为厂站。
bool twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b);

//-------------------- string opt --------------------//
// 反转路线（车辆）posA 和 posB之间的路线，
bool reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos);

// 交换两路线（车辆）两路线段位置，成功返回true
bool twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos);

#endif