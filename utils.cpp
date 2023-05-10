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
	int maxload{200};
	config >> maxload;  // 最大载货量
	std::vector<Node> nodes;
	nodes.reserve(maxload);
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
	for (unsigned int i = 0; i < size; i++) {
		for (unsigned int j = 0; j < size; j++) {
			dists(i, j) = dist(nodes[i], nodes[j]);
		}
	}
	return dists;
}