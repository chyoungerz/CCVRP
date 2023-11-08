#pragma once

#include <vector>
#ifndef _FILEIO_HPP_
#define _FILEIO_HPP_

#include <algorithm>
#include <ctime>
#include <fstream>
// #include <ostream>
#include <string>

#include "node.hpp"

// 读取文件节点返回向量nodes中
inline std::vector<Node*> read(const std::string& file, u32& maxload, u32& despot, u32& routes) {
	std::ifstream config(file);
	if (config.fail()) {
		std::vector<Node*> empty;
		return empty;
	}
	// std::vector<Node> nodes;
	std::vector<Node*> nodes_ptr;
	// nodes.reserve(100);
	int temp_x, temp_y, num, a;
	config >> num >> despot >> routes >> maxload;
	unsigned int temp_duration{}, temp_demand{}, temp_start{}, temp_end{}, seq{0};
	// customer
	while (config >> a >> temp_x >> temp_y >> temp_demand) {
		// Node temp(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		// Node* node = new Node(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		// nodes.push_back(temp);
		nodes_ptr.emplace_back(new Node(seq++, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end));
	}
	config.close();
	return nodes_ptr;
}

/// @brief 读取VRP文件
/// @param file 文件路径
/// @param maxload 最大负载
/// @param despot 仓库数量
/// @param routes 路线数量
/// @param nodes 节点列表
/// @return true | false
inline bool read_vrp(const std::string& file, u32& maxload, u32& despot, u32& routes, std::vector<Node*>& nodes) {
	std::ifstream config(file);
	if (config.fail()) {
		return false;
	}
	std::string line;
	u32 max_node{}, demand{};
	int seq{};
	double x{}, y{};
	while (std::getline(config, line)) {
		// 忽略包含特定关键字的行
		if (line.find("NAME") != std::string::npos ||
		    line.find("COMMENT") != std::string::npos ||
		    line.find("TYPE") != std::string::npos ||
		    line.find("EDGE_WEIGHT_TYPE") != std::string::npos) {
			continue;
		}
		// 读取最大节点数
		if (line.find("DIMENSION") != std::string::npos) {
			max_node = std::stoi(line.substr(line.find(":") + 1));
			continue;
		}
		// 读取容量
		if (line.find("CAPACITY") != std::string::npos) {
			maxload = std::stoi(line.substr(line.find(":") + 1));
			continue;
		}
		// 读取NODE_COORD_SECTION
		if (line.find("NODE_COORD_SECTION") != std::string::npos) {
			for (u32 i{0}; i < max_node; i++) {
				config >> seq >> x >> y;
				nodes.emplace_back(new Node(i, x, y, 0, 0, 0, 0));
			}
			std::getline(config, line);
			continue;
		}
		// 读取DEMAND_SECTION
		if (line.find("DEMAND_SECTION") != std::string::npos) {
			for (u32 i{0}; i < max_node; i++) {
				config >> seq >> demand;
				nodes[i]->demand = demand;
			}
			std::getline(config, line);
			continue;
		}
		// 读取DEPOT_SECTION
		if (line.find("DEPOT_SECTION") != std::string::npos) {
			config >> seq;
			while (seq != -1) {
				despot++;
				nodes[seq - 1]->isdepot = true;
				config >> seq;
			}
			std::getline(config, line);
			continue;
		}
		// 结束
		if (line.find("EOF") != std::string::npos) {
			break;
		}
	}
	return true;
}

// 创建保存结果的文件，按日期命名，并设置随机数种子。
inline void create(const std::string& filename) {
	char str[12];
	std::time_t now = std::time(nullptr);
	strftime(str, 96, "%Y%m%d%H%M", std::localtime(&now));
	std::ofstream out(filename, std::ios::app);  // 输出, 追加末尾
	if (out.fail()) {
		std::cerr << "Error! cannot write to file: " << filename << std::endl;
		return;
	}
	out << "日期：" << str << "\n";
	// out << "随机数种子：" << 0 << "\n";
	out.close();
}

// 将结果写入到文件中
inline void write(const std::string& filename, const Solution& sol) {
	std::fstream out(filename, std::ios::out | std::ios::app);  // 输出, 追加末尾
	if (out.fail()) {
		std::cerr << "Error! cannot write to file: " << filename << std::endl;
		return;
	}
	char chs[12];
	const time_t now = std::time(nullptr);
	strftime(chs, 96, "%Y%m%d%H%M", localtime(&now));
	out << "日期：" << chs << std::endl;
	u32 num{}, routes{};
	for (u32 i = 0, n = sol.solution.size(); i < n; i++) {
		if (sol.solution[i].path.size() - 2 == 0) continue;
		out << sol.solution[i] << " : " << sol.solution[i].path.size() - 2 << "\n";
		num += sol.solution[i].path.size() - 2;
		routes++;
	}
	out << "总路线长度: " << sol.allength << "\t 总客户数: " << num << "\t 总路线: " << routes << std::endl;
	out.close();
}

// 计算节点距离矩阵
inline void init_distance(std::vector<Node*>& nodes, const u32 depot_num, std::vector<Node*>& depots, std::vector<Node*>& customers) {
	depots.assign(nodes.begin(), nodes.begin() + depot_num);
	customers.assign(nodes.begin() + depot_num, nodes.end());
	u32 size = customers.size();
	std::for_each(depots.begin(), depots.end(), [](Node* node) { node->isdepot = true; });
	// Eigen::MatrixXf dists(size, size);
	std::vector<Edge> temp_edges(size + depot_num);
	// 总距离矩阵
	for (u32 i = 0, n{size + depot_num}; i < n; i++) {
		for (u32 j = 0; j < n; j++) {
			temp_edges[j] = {dist(nodes[i], nodes[j]), j, nodes[j]};  // i行-j列（row, col)
		}
		nodes[i]->dists.assign(temp_edges.begin(), temp_edges.end());
	}
	temp_edges.resize(size);
	// 客户->客户
	for (u32 i = 0; i < size; i++) {
		for (u32 j = 0; j < size; j++) {
			if (i == j) continue;
			temp_edges[j] = {dist(customers[i], customers[j]), j, customers[j]};  // i行-j列（row, col)
		}
		std::sort(temp_edges.begin(), temp_edges.end(), [](Edge& a, Edge& b) { return a.dist < b.dist; });
		customers[i]->distsort.assign(temp_edges.begin(), temp_edges.end());
	}
	// 客户->场站
	temp_edges.resize(depot_num);
	for (u32 i = 0; i < size; i++) {
		for (u32 j = 0; j < depot_num; j++) {
			temp_edges[j] = {dist(customers[i], depots[j]), j, depots[j]};  // i行-j列（row, col)
		}
		if (depot_num > 1) std::sort(temp_edges.begin(), temp_edges.end(), [](Edge& a, Edge& b) { return a.dist < b.dist; });
		customers[i]->depotsort.assign(temp_edges.begin(), temp_edges.end());
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