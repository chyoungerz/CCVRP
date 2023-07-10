#include "operator.hpp"

#include <algorithm>

// constexpr int MAXLOAD = 200;  // 定义最大载货

void OP::insertb(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos + 1, node);
	vehicle.load += node->demand;
}

void OP::insertf(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos, node);
	vehicle.load += node->demand;
}

void OP::insert(Vehicle& vehicle, const Node* node, const uint32_t pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos + 1, node);
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

const Node* OP::erase(Vehicle& vehicle, const uint32_t pos, const double diflength) {
	const Node* node = vehicle.path[pos];
	vehicle.path.erase(vehicle.path.begin() + pos);
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

const Node* OP::remove(Vehicle& vehicle, const uint32_t pos, const double diflength) {
	const Node* node = vehicle.path[pos];
	vehicle.path[pos] = vehicle.path.front();
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

void OP::erase(Vehicle& vehicle) {
	std::vector<const Node*> tmp_path{vehicle.path.front()};
	tmp_path.reserve(vehicle.path.size());
	for (uint32_t i = 1; i < vehicle.path.size() - 1; i++) {
		if (vehicle.path[i] != tmp_path.front()) tmp_path.emplace_back(vehicle.path[i]);
	}
	tmp_path.emplace_back(tmp_path.front());
	vehicle.path.assign(tmp_path.begin(), tmp_path.end());  // 重新分配
}

void OP::twoNSwap(Solution& sol) {
	uint32_t locate{};  // 为了降低一行代码的长度
	double difla{}, diflb{};
	for (uint32_t r{0}, n = sol.solution.size(); r < n; r++) {
		for (uint32_t i = 1; i < sol.solution[r].path.size() - 1; i++) {
			locate = sol.shash[sol.solution[r].path[i]->distsort[1].to];                                        // 邻域的位置
			uint32_t near = CHK::find(sol.solution[locate].path, sol.solution[r].path[i]->distsort[1].toNode);  // 找到最近邻域的节点
			if (locate == sol.solution[r].seq) {                                                                // 是否在同路线
				if (CHK::swaptwo(sol.solution[r], near, i, difla)) {
					sol.solution[r].cumlength += difla;
					std::swap(sol.solution[r].path[near], sol.solution[r].path[i]);
				}
			} else {
				if (CHK::twoswap(sol.solution[r], sol.solution[locate], i, near, difla, diflb)) {
					sol.solution[r].cumlength += difla, sol.solution[locate].cumlength += diflb;
					std::swap(sol.solution[r].path[i], sol.solution[locate].path[near]);
					sol.shash[sol.solution[r].path[i]->seq] = sol.solution[r].seq;  // 更新hash表
					sol.shash[sol.solution[locate].path[near]->seq] = locate;
				}
			}
		}
	}
}

void OP::oneNMove(Solution& sol) {
	uint32_t locate{}, h{}, d{};  // 为了降低一行代码的长度
	double difla{}, diflb{};      // 路线A和B的差值
	for (auto& r : sol.solution) {
		for (uint32_t i = 1; i < r.path.size() - 1; i++) {
			locate = sol.shash[r.path[i]->distsort[1].to];                                    // 邻域的位置
			uint32_t near = CHK::find(sol.solution[locate].path, r.path[i]->distsort[1].toNode);  // 找到最近邻域的节点
			if (locate == r.seq) {                                                            // 是否在同路线
				if (CHK::move(r, i, near, difla)) {
					if (i < near) {                                                           // 判断大小
						h = i;
						d = near;
					} else {
						h = near;
						d = i;
					}
					for (uint32_t j = h; j < d; j++) {  // 开始移动
						std::swap(r.path[j], r.path[j + 1]);
					}
					r.cumlength += difla;  // 更新距离
				}
			} else {
				if (CHK::move(r, sol.solution[locate], i, near, difla, diflb)) {
					const Node* node{r.path[i]};
					r.cumlength += difla, sol.solution[locate].cumlength += diflb;      // 更新距离
					r.load -= node->demand, sol.solution[locate].load += node->demand;  // 更新容量
					r.path.erase(r.path.begin() + i);                                   // 移动
					sol.solution[locate].path.emplace(sol.solution[locate].path.begin() + near + 1, node);
					sol.shash[r.path[i]->seq] = locate;  // 更新hash表
				}
			}
		}
	}
}

void OP::twoOpt(Solution& sol) {
	double dif{};  // 距离差值
	for (auto& s : sol.solution) {
		for (uint32_t i = s.path.size() - 2; i > 1; i--) {
			if (CHK::swaptwo(s, i - 1, i, dif)) {
				s.cumlength += dif;  // 更新距离
				std::swap(s.path[i - 1], s.path[i]);
			}
		}
	}
}

void OP::twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, const std::pair<double, double> diflength) {
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

double COST::insertb(std::vector<const Node*>& route, const Node* node, const uint32_t pos) {
	// 计算插入位置之后的时间（距离）
	double diflength = (route.size() - pos - 2) * (route[pos]->dists[node->seq].dist + route[pos + 1]->dists[node->seq].dist - route[pos]->dists[route[pos + 1]->seq].dist);
	// 计算插入的节点
	for (uint32_t i = 0; i < pos; i++) {
		diflength += route[i]->dists[route[i + 1]->seq].dist;
	}
	diflength += node->dists[route[pos]->seq].dist;
	return diflength;
}

double COST::insertf(std::vector<const Node*>& route, const Node* node, const uint32_t pos) {
	// 计算插入位置之后的时间（距离）
	double diflength = (route.size() - pos - 1) * (route[pos - 1]->dists[node->seq].dist + route[pos]->dists[node->seq].dist - route[pos - 1]->dists[route[pos]->seq].dist);
	// 计算插入的节点
	for (uint32_t i = 0; i < pos - 1; i++) {
		diflength += route[i]->dists[route[i + 1]->seq].dist;
	}
	diflength += node->dists[route[pos - 1]->seq].dist;
	return diflength;
}

double COST::erase(std::vector<const Node*>& route, const uint32_t pos) {
	// 计算删除位置之后的时间（距离）
	double diflength = (route.size() - pos - 2) * (route[pos - 1]->dists[route[pos + 1]->seq].dist - route[pos]->dists[route[pos + 1]->seq].dist - route[pos - 1]->dists[route[pos]->seq].dist);
	// 计算删除的节点
	for (uint32_t i = 0; i < pos; i++) {
		diflength -= route[i]->dists[route[i + 1]->seq].dist;
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

uint32_t CHK::find(std::vector<const Node*>& route, const uint32_t seq) {
	for (uint32_t i = route.size() - 1; i > 0; i--) {
		if (route[i]->seq == seq) return i;
	}
	return 0;
}

uint32_t CHK::find(std::vector<const Node*>& route, const Node* seq) {
	for (uint32_t i = route.size() - 1; i > 0; i--) {
		if (route[i] == seq) return i;
	}
	return 0;
}

bool CHK::move(Vehicle& vehicle, const uint32_t h, const uint32_t d, double& out_d) {
	double dif{};
	out_d = -vehicle.cumlength;
	for (uint32_t i{0}, n = vehicle.path.size() - 2; i < n; i++) {
		if (i == h - 1) {
			dif += vehicle.path[h - 1]->dists[vehicle.path[h + 1]->seq].dist;
			out_d += dif;
			i++;
			continue;
		}
		if (i == d) {
			dif += vehicle.path[i]->dists[vehicle.path[h]->seq].dist;
			out_d += dif;
			dif += vehicle.path[h]->dists[vehicle.path[i + 1]->seq].dist;
			out_d += dif;
			continue;
		}
		dif += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
		out_d += dif;
	}
	if (out_d > 0)
		return false;
	else
		return true;
}

bool CHK::move(Vehicle& vehicle_h, Vehicle& vehicle_d, const uint32_t h, const uint32_t d, double& out_da, double& out_db) {
	if (vehicle_d.load + vehicle_h.path[h]->demand > vehicle_d.capacity) return false;
	out_da = (vehicle_h.path.size() - 2 - h) * (vehicle_h.path[h - 1]->dists[vehicle_h.path[h + 1]->seq].dist - vehicle_h.path[h]->dists[vehicle_h.path[h + 1]->seq].dist - vehicle_h.path[h - 1]->dists[vehicle_h.path[h]->seq].dist);
	out_db = (vehicle_d.path.size() - 2 - d) * (vehicle_d.path[d]->dists[vehicle_h.path[h]->seq].dist + vehicle_d.path[d + 1]->dists[vehicle_h.path[h]->seq].dist - vehicle_d.path[d]->dists[vehicle_d.path[d + 1]->seq].dist);
	if (h <= d) {
		uint32_t i = 0;
		for (; i < h; i++) {
			out_da -= vehicle_h.path[i]->dists[vehicle_h.path[i + 1]->seq].dist;
			out_db += vehicle_d.path[i]->dists[vehicle_d.path[i + 1]->seq].dist;
		}
		for (; i < d; i++) {
			out_db += vehicle_d.path[i]->dists[vehicle_d.path[i + 1]->seq].dist;
		}
		out_db += vehicle_d.path[d]->dists[vehicle_h.path[h]->seq].dist;
	} else {
		uint32_t i = 0;
		for (; i < d; i++) {
			out_da -= vehicle_h.path[i]->dists[vehicle_h.path[i + 1]->seq].dist;
			out_db += vehicle_d.path[i]->dists[vehicle_d.path[i + 1]->seq].dist;
		}
		for (; i < h; i++) {
			out_da -= vehicle_d.path[i]->dists[vehicle_d.path[i + 1]->seq].dist;
		}
		out_db += vehicle_d.path[d]->dists[vehicle_h.path[h]->seq].dist;
	}
	if (out_da + out_db > 0) return false;
	/*const Node* node{vehicle_h.path.erase(vehicle_h.path.begin() + h)}; //移动
	vehicle_d.path.emplace(vehicle_d.path.begin() + d, node);*/
	return true;
}

bool CHK::swaptwo(Vehicle& vehicle, const uint32_t pos_i, const uint32_t pos_j, double& out_d) {
	uint32_t size = vehicle.path.size() - 2;
	double difa = vehicle.path[pos_i - 1]->dists[vehicle.path[pos_j]->seq].dist + vehicle.path[pos_i + 1]->dists[vehicle.path[pos_j]->seq].dist - vehicle.path[pos_i - 1]->dists[vehicle.path[pos_i]->seq].dist - vehicle.path[pos_i + 1]->dists[vehicle.path[pos_i]->seq].dist;
	double difb = vehicle.path[pos_j - 1]->dists[vehicle.path[pos_i]->seq].dist + vehicle.path[pos_j + 1]->dists[vehicle.path[pos_i]->seq].dist - vehicle.path[pos_j - 1]->dists[vehicle.path[pos_j]->seq].dist - vehicle.path[pos_j + 1]->dists[vehicle.path[pos_j]->seq].dist;
	difa *= size - pos_i;
	difb *= size - pos_j;
	difa += vehicle.path[pos_i - 1]->dists[vehicle.path[pos_j]->seq].dist - vehicle.path[pos_i - 1]->dists[vehicle.path[pos_i]->seq].dist;
	difb += vehicle.path[pos_j - 1]->dists[vehicle.path[pos_i]->seq].dist - vehicle.path[pos_j - 1]->dists[vehicle.path[pos_j]->seq].dist;
	out_d = difa + difb;
	if (out_d > 0)
		return false;
	else
		return true;
}

bool CHK::twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, double& out_da, double& out_db) {
	int difload = vehicle_a.path[pos_a]->demand - vehicle_b.path[pos_b]->demand;
	if ((vehicle_a.load - difload) > vehicle_a.capacity || (vehicle_b.load + difload) > vehicle_b.capacity) return false;  // 超载
	vehicle_a.load -= difload, vehicle_b.load += difload;
	out_da = vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_a.path[pos_a + 1]->dists[vehicle_b.path[pos_b]->seq].dist - vehicle_a.path[pos_a - 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_a.path[pos_a + 1]->dists[vehicle_a.path[pos_a]->seq].dist;
	out_db = vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist + vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist - vehicle_b.path[pos_b + 1]->dists[vehicle_b.path[pos_b]->seq].dist;
	out_da *= vehicle_a.path.size() - pos_a - 2;
	out_db *= vehicle_b.path.size() - pos_b - 2;
	out_da += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist - vehicle_a.path[pos_a - 1]->dists[vehicle_a.path[pos_a]->seq].dist;
	out_db += vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist;
	if (out_da + out_db > 0)
		return false;
	else
		return true;
}

bool CHK::twoCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const uint32_t pos_a, const uint32_t pos_b, double& out_da, double& out_db) {
	uint32_t index_a = vehicle_a.path.size() - 2 - pos_a, index_b = vehicle_b.path.size() - 2 - pos_b;
	uint32_t difload{};
	if (index_a > index_b) {  // 判断容量约束
		uint32_t i{0};
		for (; i < index_b; i++) {
			difload += vehicle_a.path[pos_a + i]->demand - vehicle_b.path[pos_b + i]->demand;
		}
		for (; i < index_a; i++) {
			difload += vehicle_a.path[pos_a + i]->demand;
		}
	} else {
		uint32_t i{0};
		for (; i < index_a; i++) {
			difload += vehicle_a.path[pos_a + i]->demand - vehicle_b.path[pos_b + i]->demand;
		}
		for (; i < index_b; i++) {
			difload -= vehicle_b.path[pos_b + i]->demand;
		}
	}
	if (vehicle_a.load - difload > vehicle_a.capacity || vehicle_b.load + difload > vehicle_b.capacity) return false;  // 超载
	double difa{}, difb{};
	out_da = -vehicle_a.cumlength, out_db = -vehicle_b.cumlength;
	for (uint32_t i{1}; i < pos_a; i++) {  // 计算a距离差值
		difa += vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		out_da += difa;
	}
	difa += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist;
	out_da += difa;
	for (uint32_t i{pos_b}, n{index_b + pos_b}; i < n; i++) {  // 计算a距离差值
		difa += vehicle_b.path[i]->dists[vehicle_b.path[i + 1]->seq].dist;
		out_da += difa;
	}

	for (uint32_t i{1}; i < pos_b; i++) {  // 计算b距离差值
		difb += vehicle_b.path[i - 1]->dists[vehicle_b.path[i]->seq].dist;
		out_db += difb;
	}
	difb += vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist;
	out_db += difb;
	for (uint32_t i{pos_a}, n{index_a + pos_a}; i < n; i++) {  // 计算b距离差值
		difb += vehicle_a.path[i]->dists[vehicle_a.path[i + 1]->seq].dist;
		out_db += difb;
	}
	if (out_da + out_db > 0) return false;  // 不满足约束
	index_a++, index_b++;
	vehicle_a.path.pop_back();
	vehicle_b.path.pop_back();
	if (index_a > index_b) {                // 移动
		uint32_t i{0};
		for (; i < index_b; i++) {
			std::swap(vehicle_a.path[pos_a + i], vehicle_b.path[pos_b + i]);
		}
		for (; i < index_a; i++) {
			vehicle_b.path.emplace_back(vehicle_a.path[pos_a + i]);
		}
		vehicle_b.path.emplace_back(vehicle_b.path.front());
		vehicle_a.path.resize(pos_a + index_b);
		vehicle_a.path.emplace_back(vehicle_a.path.front());
	} else {
		uint32_t i{0};
		for (; i < index_a; i++) {
			std::swap(vehicle_a.path[pos_a + i], vehicle_b.path[pos_b + i]);
		}
		for (; i < index_b; i++) {
			vehicle_a.path.emplace_back(vehicle_b.path[pos_b + i]);
		}
		vehicle_a.path.emplace_back(vehicle_a.path.front());
		vehicle_b.path.resize(pos_b + index_a);
		vehicle_b.path.emplace_back(vehicle_b.path.front());
	}
	return true;
}