#pragma once

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <ctime>
#include <string>

#include "node.hpp"

// 读取文件节点返回向量nodes中
std::vector<Node*> read(const std::string& file);

// 创建保存结果的文件，按日期命名，并设置随机数种子。
void create(std::string& filename, const time_t now);

// 计算节点距离矩阵
void init_distance(std::vector<Node*>& nodes);

#endif /*_UTILS_HPP_*/