#pragma once

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <string>

#include "node.hpp"

extern std::vector<Node> nodes;  // 节点(全局变量)
extern Eigen::MatrixXf dists;    // 距离矩阵(全局变量)

// 读取文件节点到全局变量nodes中
bool read(const std::string& file);

// 创建保存结果的文件，按日期命名，并设置随机数种子。
void create(std::string& filename);

// 计算节点距离矩阵到全局变量dists中
void init_distance();

#endif /*_UTILS_HPP_*/