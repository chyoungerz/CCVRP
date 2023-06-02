#include "operator.hpp"

#include <algorithm>

// constexpr int MAXLOAD = 200;  // 定义最大载货

bool OP::insertback(Vehicle& vehicle, const Node* node) {
	if ((vehicle.load + node->demand) > vehicle.capacity) return false;  // 超重
	double diflength{0.0};
	vehicle.path.push_back(node);
	for (uint32_t i = 0; i + 2 < vehicle.path.size(); i++) {  // 计算插入的节点
		diflength += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

bool OP::insertfront(Vehicle& vehicle, const Node* node) {
	if ((vehicle.load + node->demand) > vehicle.capacity) return false;  // 超重
	double diflength = node->dists[vehicle.path[0]->seq].dist * vehicle.path.size();
	vehicle.path.push_back(node);
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

inline void OP::insert(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength) {
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.insert(vehicle.path.begin() + pos, node);
	vehicle.load += node->demand;
}

const Node* OP::removeback(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	const Node* node = vehicle.path.back();
	double diflength{0.0};
	for (uint32_t i = 0; i + 1 < vehicle.path.size(); i++) {  // 计算删除的节点
		diflength += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
	}
	vehicle.path.pop_back();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

const Node* OP::removefront(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	const Node* node = vehicle.path.front();
	vehicle.path.erase(vehicle.path.begin());
	double diflength = node->dists[vehicle.path[0]->seq].dist * vehicle.path.size();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

inline const Node* OP::remove(Vehicle& vehicle, const uint32_t pos, const double diflength) {
	const Node* node = vehicle.path[pos];
	vehicle.path.erase(vehicle.path.begin() + pos);
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= vehicle.path[pos]->demand;
	return node;
}

inline void OP::swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j, const double diflength) {
	std::swap(vehicle.path[pos_i], vehicle.path[pos_j]);  // 交换
	vehicle.cumlength += diflength;                       // 更新距离（时间）
}

inline void OP::twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, const std::pair<double, double> diflength) {
	std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);  // 交换
	vehicle_a.cumlength += diflength.first;
	vehicle_b.cumlength += diflength.second;                  // 更新距离（时间）
}

bool OP::reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos) {
	if (from_pos >= to_pos || from_pos == 0 || to_pos == vehicle.path.size() - 1) return false;                                                                                                       // 不合法
	double diflength = (vehicle.path.size() - from_pos) * (vehicle.path[to_pos]->dists[vehicle.path[from_pos - 1]->seq].dist - vehicle.path[from_pos]->dists[vehicle.path[from_pos - 1]->seq].dist);  // 反转后from之后的差值
	diflength += (vehicle.path.size() - to_pos - 1) * (vehicle.path[from_pos]->dists[vehicle.path[to_pos + 1]->seq].dist - vehicle.path[to_pos]->dists[vehicle.path[to_pos + 1]->seq].dist);          //// 反转后to之后的差值
	std::reverse(vehicle.path.begin() + from_pos, vehicle.path.begin() + to_pos);                                                                                                                     // 反转
	vehicle.cumlength += diflength;                                                                                                                                                                   // 更新距离（时间）
	return true;
}

bool OP::twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos) {
	if (from_a_pos >= to_a_pos || from_a_pos == 0 || to_a_pos == vehicle_a.path.size() - 1) return false;
	if (from_b_pos >= to_b_pos || from_b_pos == 0 || to_b_pos == vehicle_b.path.size() - 1) return false;  // 不合法
	uint32_t diff{to_a_pos - from_a_pos}, diff_a{static_cast<uint32_t>(vehicle_a.path.size()) - from_a_pos}, diff_b{static_cast<uint32_t>(vehicle_b.path.size()) - from_b_pos};
	if (diff != (to_b_pos - from_b_pos)) return false;
	double diflength_a{0.0}, diflength_b{0.0};
	int difload{0};
	for (uint32_t i = 0; i + 1 < diff; i++) {
		double diflength = (vehicle_a.path[from_a_pos + i]->dists[vehicle_a.path[from_a_pos + i + 1]->seq].dist - vehicle_b.path[from_b_pos + i]->dists[vehicle_b.path[from_b_pos + i + 1]->seq].dist);  // 要交换的路线的差异
		diflength_a -= (diff_a - 1 - i) * diflength;                                                                                                                                                     // 由于交换的相对位置不同，而造成的差异次数不同。
		diflength_b += (diff_b - 1 - i) * diflength;
		difload += vehicle_a.path[from_a_pos + i]->demand - vehicle_b.path[from_b_pos + i]->demand;                                                                                                      // 载货量差别
		std::swap(vehicle_a.path[from_a_pos + i], vehicle_b.path[from_b_pos + i]);                                                                                                                       // 交换
	}
	std::swap(vehicle_a.path[to_a_pos], vehicle_b.path[to_b_pos]);                                                                                                                                       // 完成剩下的交换
	if ((vehicle_a.load - difload) > vehicle_a.capacity || (vehicle_b.load + difload) > vehicle_b.capacity) return false;                                                                                // 超重
	diflength_a += diff_a * (vehicle_a.path[from_a_pos - 1]->dists[vehicle_b.path[to_b_pos]->seq].dist - vehicle_a.path[from_a_pos - 1]->dists[vehicle_a.path[from_a_pos]->seq].dist);
	diflength_a += (diff_a - diff - 1) * (vehicle_a.path[to_a_pos + 1]->dists[vehicle_b.path[to_b_pos]->seq].dist - vehicle_a.path[to_a_pos + 1]->dists[vehicle_a.path[to_a_pos]->seq].dist);            // 路线（车辆）a 受影响的差异
	diflength_b += diff_b * (vehicle_b.path[from_b_pos - 1]->dists[vehicle_a.path[from_a_pos]->seq].dist - vehicle_b.path[from_b_pos - 1]->dists[vehicle_b.path[from_b_pos]->seq].dist);
	diflength_b += (diff_b - diff - 1) * (vehicle_b.path[to_b_pos + 1]->dists[vehicle_a.path[to_a_pos]->seq].dist - vehicle_b.path[to_b_pos + 1]->dists[vehicle_b.path[to_b_pos]->seq].dist);            ////路线（车辆）b 受影响的差异

	vehicle_a.cumlength += diflength_a;                                                                                                                                                                  // 更新a
	vehicle_a.load -= difload;
	vehicle_b.cumlength += diflength_b;                                                                                                                                                                  // 更新b
	vehicle_b.load += difload;
	return true;
}

double COST::insert(Vehicle& vehicle, const Node* node, const uint32_t pos) {
	// 计算插入位置之后的时间（距离）
	double diflength = (vehicle.path.size() - pos - 2) * (vehicle.path[pos]->dists[node->seq].dist + vehicle.path[pos + 1]->dists[node->seq].dist - vehicle.path[pos]->dists[vehicle.path[pos + 1]->seq].dist);
	// 计算插入的节点
	for (uint32_t i = 0; i < pos; i++) {
		diflength += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
	}
	diflength += node->dists[vehicle.path[pos]->seq].dist;
	return diflength;
}

double COST::remove(Vehicle& vehicle, const uint32_t pos) {
	// 计算删除位置之后的时间（距离）
	double diflength = (vehicle.path.size() - pos - 2) * (vehicle.path[pos - 1]->dists[vehicle.path[pos + 1]->seq].dist - vehicle.path[pos]->dists[pos + 1].dist - vehicle.path[pos - 1]->dists[pos].dist);
	// 计算删除的节点
	for (uint32_t i = 0; i < pos; i++) {
		diflength -= vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
	}
	return diflength;
}

double COST::reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos) {
	double diflength = 0.0;
	return diflength;
}

double COST::swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j) {
	double lpos_i = vehicle.path[pos_j]->dists[vehicle.path[pos_i - 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i - 1]->seq].dist;                                                 // pos_i左的差值
	double rpos_i = vehicle.path[pos_j]->dists[vehicle.path[pos_i + 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i + 1]->seq].dist;                                                 // pos_i右的差值
	double lpos_j = vehicle.path[pos_i]->dists[vehicle.path[pos_j - 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j - 1]->seq].dist;                                                 // pos_j左的差值
	double rpos_j = vehicle.path[pos_i]->dists[vehicle.path[pos_j + 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j + 1]->seq].dist;                                                 // pos_j右的差值
	double diflength = (vehicle.path.size() - pos_i) * lpos_i + (vehicle.path.size() - pos_i - 1) * rpos_i + (vehicle.path.size() - pos_j) * lpos_j + (vehicle.path.size() - pos_j - 1) * rpos_j;  // 路线（车辆）所有受影响的差值
	return diflength;
}

std::pair<double, double> COST::twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b) {
	double lpos_a = vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a - 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a - 1]->seq].dist;  // pos_a左的差值
	double rpos_a = vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a + 1]->seq].dist;  // pos_a右的差值
	double lpos_b = vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b - 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b - 1]->seq].dist;  // pos_b左的差值
	double rpos_b = vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist;  // pos_b右的差值
	double diflengtha = (vehicle_a.path.size() - pos_a) * lpos_a + (vehicle_a.path.size() - pos_a - 1) * rpos_a;                                            // 路线（车辆）a所有受影响的差值
	double diflengthb = (vehicle_b.path.size() - pos_b) * lpos_b + (vehicle_b.path.size() - pos_b - 1) * rpos_b;                                            // 路线（车辆）b所有受影响的差值
	return std::make_pair(diflengtha, diflengthb);
}

std::pair<double, double> COST::twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos) {
	double diflengtha = 0.0;
	double diflengthb = 0.0;
	return std::make_pair(diflengtha, diflengthb);
}