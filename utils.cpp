#include "utils.hpp"

#include <ctime>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>

#include "distribution.hpp"

bool read(const std::string& file) {
	std::ifstream config(file);
	if (config.fail()) {
		return false;
	}
	int maxload{200};
	config >> maxload;  // 最大载货量
	nodes.reserve(maxload);
	int temp_x, temp_y, a, b, c;
	unsigned int temp_duration, temp_demand, temp_start, temp_end, seq;
	while (config >> seq >> temp_x >> temp_y >> temp_duration >> temp_demand >> a >> b >> c >> temp_start >> temp_end) {
		Node temp(seq, temp_x, temp_y, temp_duration, temp_demand, temp_start, temp_end);
		nodes.push_back(temp);
	}
	config.close();
	return true;
}

void create(std::string& filename) {
	char str[12];
	now = time(nullptr);
	strftime(str, 96, "%Y%m%d%H%M", localtime(&now));
	std::ofstream out(filename, std::ofstream::app);  // 输出, 追加末尾
	out << "日期：" << str << "\n";
	out << "随机数种子：" << now << "\n";
	out.close();
}

void init_distance() {
	uint32_t size = nodes.size();
	dists.resize(size, size);
	for (unsigned int i = 0; i < size; i++) {
		for (unsigned int j = 0; j < size; j++) {
			dists(i, j) = dist(nodes[i], nodes[j]);
		}
	}
}