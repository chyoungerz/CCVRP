#include "utils.hpp"

#include <ctime>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>

std::vector<Node> read(const std::string& file) {
	std::ifstream config(file);
	if (config.fail()) {
		std::vector<Node> empty;
		return empty;
	}
	std::vector<Node> nodes;
	nodes.reserve(100);
	int temp_x, temp_y, a, b, c;
	unsigned int temp_duration, temp_demand, temp_start, temp_end, seq;
	while (config >> seq >> temp_x >> temp_y >> temp_duration >> temp_demand >> a >> b >> c >> temp_start >> temp_end) {
		Node temp(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		nodes.push_back(temp);
	}
	config.close();
	return nodes;
}

void create(std::string& filename, const time_t now) {
	char str[12];
	strftime(str, 96, "%Y%m%d%H%M", localtime(&now));
	std::ofstream out(filename, std::ofstream::app);  // 输出, 追加末尾
	out << "日期：" << str << "\n";
	out << "随机数种子：" << now << "\n";
	out.close();
}

Eigen::MatrixXf init_distance(std::vector<Node>& nodes) {
	uint32_t size = nodes.size();
	Eigen::MatrixXf dists(size, size);
	std::vector<Edge> temp_edges(size);
	for (unsigned int i = 0; i < size; i++) {
		for (unsigned int j = 0; j < size; j++) {
			float d = dist(nodes[i], nodes[j]);  // i行-j列（row, col)
			temp_edges.push_back({d, j});
		}
		nodes[i].distances.assign(temp_edges.begin(), temp_edges.end());
		temp_edges.clear();
	}
	return dists;
}