#pragma once

#ifndef _OPERATION_HPP
#define _OPERATION_HPP

#include "node.hpp"

// 在路线（车辆）vehicle的末尾插入节点node，成功返回true (传指针)
bool insertback(Vehicle& vehicle, const Node* node);

// 在路线（车辆）vehicle的pos位置后插入节点node，成功返回true
bool insert(Vehicle& vehicle, const Node* node, const uint32_t pos);

// 删除路线（车辆）vehicle的pos位置节点node，成功返回true
bool remove(Vehicle& vehicle, const uint32_t pos);

#endif