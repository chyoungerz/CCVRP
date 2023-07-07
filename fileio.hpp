#pragma once

#ifndef _FILEIO_HPP_
#define _FILEIO_HPP_

#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>

#include "node.hpp"

// 读取文件节点返回向量nodes中
inline std::vector<Node*> read(const std::string& file, uint32_t& maxload, uint32_t& despot) {
	std::ifstream config(file);
	if (config.fail()) {
		std::vector<Node*> empty;
		return empty;
	}
	// std::vector<Node> nodes;
	std::vector<Node*> nodes_ptr;
	// nodes.reserve(100);
	int temp_x, temp_y, a, b, c, num;
	config >> a >> b >> num >> despot >> c >> maxload;
	unsigned int temp_duration, temp_demand, temp_start{}, temp_end{}, seq{0};
	// customer
	while (config >> a >> temp_x >> temp_y >> temp_duration >> temp_demand) {
		// Node temp(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		// Node* node = new Node(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		// nodes.push_back(temp);
		nodes_ptr.emplace_back(new Node(seq++, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end));
	}
	config.close();
	return nodes_ptr;
}

// 创建保存结果的文件，按日期命名，并设置随机数种子。
inline void create(const std::string& filename, const time_t now) {
	char str[12];
	strftime(str, 96, "%Y%m%d%H%M", localtime(&now));
	std::ofstream out(filename, std::ofstream::app);  // 输出, 追加末尾
	out << "日期：" << str << "\n";
	out << "随机数种子：" << now << "\n";
	out.close();
}

// 将结果写入到文件中
inline void write(const std::string& filename, const Solution& sol) {
	std::ofstream out(filename, std::ofstream::app);  // 输出, 追加末尾
	out << "total length:" << sol.allength << std::endl;
	for (auto& i : (sol.solution)) {
		out << i << std::endl;
	}
	out.close();
}

// 计算节点距离矩阵
inline void init_distance(std::vector<Node*>& nodes, uint32_t depot_num) {
	uint32_t size = nodes.size();
	// Eigen::MatrixXf dists(size, size);
	std::vector<Edge> temp_edges(size);
	for (unsigned int i = 0; i < size; i++) {
		for (unsigned int j = 0; j < size; j++) {
			temp_edges[j] = {dist(nodes[i], nodes[j]), j, nodes[j]};  // i行-j列（row, col)
		}
		nodes[i]->dists.assign(temp_edges.begin(), temp_edges.end());
		std::sort(temp_edges.begin(), temp_edges.end() - depot_num, [](Edge& a, Edge& b) { return a.dist < b.dist; });
		nodes[i]->distsort.assign(temp_edges.begin(), temp_edges.end());
	}
	// return dists;
}

// 释放nodes内存（包括厂站）
inline void release(std::vector<Node*>& nodes) {
	for (auto& node : nodes) {
		delete node;
	}
	nodes.clear();
}

#endif /*_FILEIO_HPP_*/