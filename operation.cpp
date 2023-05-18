#include "operation.hpp"

constexpr int MAXLOAD = 200;  // 定义最大载货

bool insertback(Vehicle& vehicle, const Node* node) {
	if ((vehicle.load + node->demand) > MAXLOAD) return false;  // 超重
	double diflength{0.0};
	vehicle.path.push_back(node);
	for (int i = 0; i + 1 < vehicle.path.size(); i++) {  // 计算插入的节点
		diflength += vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

bool insertfront(Vehicle& vehicle, const Node* node) {
	if ((vehicle.load + node->demand) > MAXLOAD) return false;  // 超重
	double diflength = node->distances[vehicle.path[0]->seq].distance * vehicle.path.size();
	vehicle.path.push_back(node);
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

bool insert(Vehicle& vehicle, const Node* node, const uint32_t pos) {
	if ((vehicle.load + node->demand) > MAXLOAD) return false;                                                                 // 超重                                                                                                                 // 超重
	if (pos >= vehicle.path.size() - 1 && pos == 0) return false;                                                              // 不合法
	double diflength = (vehicle.path.size() - pos - 1) * (vehicle.path[pos]->distances[node->seq].distance + vehicle.path[pos + 1]->distances[node->seq].distance -
														  vehicle.path[pos]->distances[vehicle.path[pos + 1]->seq].distance);  // 计算插入位置之后的时间（距离）
	vehicle.path.insert(vehicle.path.begin() + pos, node);
	for (int i = 0; i <= pos; i++) {                                                                                           // 计算插入的节点
		diflength += vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

const Node* removeback(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	const Node* node = vehicle.path.back();
	double diflength{0.0};
	for (int i = 0; i + 1 < vehicle.path.size(); i++) {  // 计算删除的节点
		diflength += vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.path.pop_back();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

const Node* removefront(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	const Node* node = vehicle.path.front();
	vehicle.path.erase(vehicle.path.begin());
	double diflength = node->distances[vehicle.path[0]->seq].distance * vehicle.path.size();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

const Node* remove(Vehicle& vehicle, const uint32_t pos) {
	if (vehicle.load == 0) return nullptr;                           // 没法删
	if (pos >= vehicle.path.size() - 1 || pos == 0) return nullptr;  // 不合法
	const Node* node = vehicle.path[pos];
	double diflength = (vehicle.path.size() - pos - 1) * (vehicle.path[pos - 1]->distances[vehicle.path[pos + 1]->seq].distance -
														  vehicle.path[pos]->distances[pos + 1].distance - vehicle.path[pos - 1]->distances[pos].distance);  // 计算删除位置之后的时间（距离）
	vehicle.path.erase(vehicle.path.begin() + pos);
	for (int i = 0; i < pos; i++) {                                                                                                                          // 计算删除的节点
		diflength -= vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= vehicle.path[pos]->demand;
	return node;
}

bool swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j) {
	if (pos_i == pos_j || pos_i == 0 || pos_j == 0 || pos_i >= vehicle.path.size() - 1 || pos_j >= vehicle.path.size() - 1) return false;                                                          // 不合法
	double lpos_i = vehicle.path[pos_j]->distances[vehicle.path[pos_i - 1]->seq].distance - vehicle.path[pos_i]->distances[vehicle.path[pos_i - 1]->seq].distance;                                 // pos_i左的差值
	double rpos_i = vehicle.path[pos_j]->distances[vehicle.path[pos_i + 1]->seq].distance - vehicle.path[pos_i]->distances[vehicle.path[pos_i + 1]->seq].distance;                                 // pos_i右的差值
	double lpos_j = vehicle.path[pos_i]->distances[vehicle.path[pos_j - 1]->seq].distance - vehicle.path[pos_j]->distances[vehicle.path[pos_j - 1]->seq].distance;                                 // pos_j左的差值
	double rpos_j = vehicle.path[pos_i]->distances[vehicle.path[pos_j + 1]->seq].distance - vehicle.path[pos_j]->distances[vehicle.path[pos_j + 1]->seq].distance;                                 // pos_j右的差值
	double diflength = (vehicle.path.size() - pos_i) * lpos_i + (vehicle.path.size() - pos_i - 1) * rpos_i + (vehicle.path.size() - pos_j) * lpos_j + (vehicle.path.size() - pos_j - 1) * rpos_j;  // 路线（车辆）所有受影响的差值
	std::swap(vehicle.path[pos_i], vehicle.path[pos_j]);                                                                                                                                           // 交换
	vehicle.cumlength += diflength;                                                                                                                                                                // 更新距离（时间）
	return true;
}

bool twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b) {
	if (pos_a == 0 || pos_b == 0 || pos_a >= vehicle_a.path.size() - 1 || pos_b >= vehicle_b.path.size() - 1) return false;  // 不合法
	int difload = vehicle_b.path[pos_b]->demand - vehicle_a.path[pos_a]->demand;
	if ((vehicle_a.load + difload) > MAXLOAD || (vehicle_a.load - difload) > MAXLOAD) return false;                          // 超重                                                                                                                        // 更新距离（时间）
	vehicle_a.load += difload;
	vehicle_b.load -= difload;
	double lpos_a = vehicle_b.path[pos_b]->distances[vehicle_a.path[pos_a - 1]->seq].distance - vehicle_a.path[pos_a]->distances[vehicle_a.path[pos_a - 1]->seq].distance;  // pos_a左的差值
	double rpos_a = vehicle_b.path[pos_b]->distances[vehicle_a.path[pos_a + 1]->seq].distance - vehicle_a.path[pos_a]->distances[vehicle_a.path[pos_a + 1]->seq].distance;  // pos_a右的差值
	double lpos_b = vehicle_a.path[pos_a]->distances[vehicle_b.path[pos_b - 1]->seq].distance - vehicle_b.path[pos_b]->distances[vehicle_b.path[pos_b - 1]->seq].distance;  // pos_b左的差值
	double rpos_b = vehicle_a.path[pos_a]->distances[vehicle_b.path[pos_b + 1]->seq].distance - vehicle_b.path[pos_b]->distances[vehicle_b.path[pos_b + 1]->seq].distance;  // pos_b右的差值
	double diflength_a = (vehicle_a.path.size() - pos_a) * lpos_a + (vehicle_a.path.size() - pos_a - 1) * rpos_b;                                                           // 路线（车辆）a所有受影响的差值
	double diflength_b = (vehicle_b.path.size() - pos_b) * lpos_b + (vehicle_b.path.size() - pos_b - 1) * rpos_b;                                                           // 路线（车辆）b所有受影响的差值
	std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);                                                                                                                // 交换
	vehicle_a.cumlength += diflength_a;
	vehicle_b.cumlength += diflength_b;                                                                                                                                     // 更新距离（时间）
	return true;
}

bool reverse(Vehicle& vehicle, const uint32_t from_pos, const uint32_t to_pos) {
}

bool twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t from_a_pos, const uint32_t to_a_pos, const uint32_t from_b_pos, const uint32_t to_b_pos) {
}