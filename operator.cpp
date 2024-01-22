#include "operator.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include "algorithm.hpp"

// constexpr int MAXLOAD = 200;  // 定义最大载货

void OP::insertb(Vehicle& vehicle, Node* node, const u32 pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos + 1, node);
	vehicle.load += node->demand;
}

void OP::insertf(Vehicle& vehicle, Node* node, const u32 pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos, node);
	vehicle.load += node->demand;
}

void OP::insert(Vehicle& vehicle, Node* node, const u32 pos, const double diflength) {
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.path.emplace(vehicle.path.begin() + pos + 1, node);
	vehicle.load += node->demand;
}

Node* OP::removeback(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	Node* node = vehicle.path.back();
	double diflength{0.0};
	for (u32 i = 0; i + 1 < vehicle.path.size(); i++) {  // 计算删除的节点
		diflength += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
	}
	vehicle.path.pop_back();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

Node* OP::removefront(Vehicle& vehicle) {
	if (vehicle.load == 0) return nullptr;  // 没法删
	Node* node = vehicle.path.front();
	vehicle.path.erase(vehicle.path.begin());
	double diflength = node->dists[vehicle.path[0]->seq].dist * vehicle.path.size();
	vehicle.cumlength -= diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

Node* OP::erase(Vehicle& vehicle, const u32 pos, const double diflength) {
	Node* node = vehicle.path[pos];
	vehicle.path.erase(vehicle.path.begin() + pos);
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

Node* OP::remove(Vehicle& vehicle, const u32 pos, const double diflength) {
	Node* node = vehicle.path[pos];
	vehicle.path[pos] = vehicle.path.front();
	// vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= node->demand;
	return node;
}

void OP::erase(Vehicle& vehicle) {
	std::vector<Node*> tmp_path{vehicle.path.front()};
	tmp_path.reserve(vehicle.path.size());
	for (u32 i = 1; i < vehicle.path.size() - 1; i++) {
		if (vehicle.path[i] != tmp_path.front()) tmp_path.emplace_back(vehicle.path[i]);
	}
	tmp_path.emplace_back(tmp_path.front());
	vehicle.path.assign(tmp_path.begin(), tmp_path.end());  // 重新分配
}

void OP::twoNSwap(Solution& sol) {
	u32 locate{};  // 为了降低一行代码的长度
	double difla{}, diflb{};
	for (u32 r{0}, n = sol.solution.size(); r < n; r++) {
		for (u32 i = 1; i < sol.solution[r].path.size() - 1; i++) {
			locate = sol.shash[sol.solution[r].path[i]->distsort[1].to];                                        // 邻域的位置
			u32 near = CHK::find(sol.solution[locate].path, sol.solution[r].path[i]->distsort[1].toNode, 0);    // 找到最近邻域的节点
			if (locate == sol.solution[r].seq) {                                                                // 是否在同路线
				if (CHK::twoSwap(sol.solution[r], near, i, difla)) {
					sol.solution[r].cumlength += difla;
					std::swap(sol.solution[r].path[near], sol.solution[r].path[i]);
				}
			} else {
				if (CHK::twoSwap(sol.solution[r], sol.solution[locate], i, near, difla, diflb)) {
					sol.solution[r].cumlength += difla, sol.solution[locate].cumlength += diflb;
					std::swap(sol.solution[r].path[i], sol.solution[locate].path[near]);
					sol.shash[sol.solution[r].path[i]->seq] = sol.solution[r].seq;  // 更新hash表
					sol.shash[sol.solution[locate].path[near]->seq] = locate;
				}
			}
		}
	}
}

void OP::oneNMove(Solution& sol, std::vector<Node*>& nodes, const u32 depot_num) {
	double difla{}, diflb{};      // 路线A和B的差值
	u32 from{}, home{}, to{}, dest{};
	for (u32 i{0}, n = nodes.size() - depot_num; i < n; i++) {
		from = sol.shash[nodes[i]->seq];
		home = CHK::find(sol.solution[from].path, nodes[i], 0);
		to = sol.shash[nodes[i]->distsort[1].toNode->seq];
		dest = CHK::find(sol.solution[to].path, nodes[i]->distsort[1].toNode, 0);
		if (from == to) {  // 是否在同路线
			if (CHK::oneMove(sol.solution[from], home, dest, difla))
				sol.solution[from].cumlength += difla;
		} else {
			if (CHK::oneMove(sol.solution[from], sol.solution[to], home, dest, difla, diflb)) {
				sol.solution[from].cumlength += difla;
				sol.solution[to].cumlength += diflb;
				sol.shash[nodes[i]->seq] = to;  // 更新hash表
			}
		}
	}
}

void OP::oneMove(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag) {
	double difa{}, difb{};  // 距离差值
	for (u32 i{1}, n = vehicle_a.path.size() - 1; i < n; i++) {
		for (u32 j{1}, m = vehicle_b.path.size() - 1; j < m;) {
			if (i == n) break;
			if (CHK::oneMove(vehicle_a, vehicle_b, i, j, difa, difb)) {
				vehicle_a.cumlength += difa, vehicle_b.cumlength += difb;
				flag = true;
				n--;
				m++;
			} else {
				j++;
			}
		}
	}
}

void OP::twoOpt(Solution& sol) {
	double dif{};  // 距离差值
	for (u32 s{0}, size = sol.solution.size(); s < size; s++) {
		if (sol.solution[s].path.size() <= 4) continue;
		for (u32 i{1}, n = sol.solution[s].path.size() - 3; i < n; i++) {
			for (u32 j{i + 2}, m = n + 2; j < m; j++) {
				if (CHK::reverse(sol.solution[s], i, j, dif)) {
					sol.solution[s].cumlength += dif;  // 更新距离
				}
			}
		}
	}
}

void OP::twoSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag) {
	double difa{}, difb{};  // 距离差值
	for (u32 i{1}, n = vehicle_a.path.size() - 1; i < n; i++) {
		for (u32 j{1}, m = vehicle_b.path.size() - 1; j < m; j++) {
			if (CHK::twoSwap(vehicle_a, vehicle_b, i, j, difa, difb)) {
				std::swap(vehicle_a.path[i], vehicle_b.path[j]);
				vehicle_a.cumlength += difa, vehicle_b.cumlength += difb;
				flag = true;
			}
		}
	}
}

void OP::threeSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, bool& flag) {
	double difa{}, difb{};  // 距离差值
	for (u32 i{2}, n = vehicle_b.path.size() - 2; i < n; i++) {
		for (u32 j{1}, m = vehicle_a.path.size() - 1; j < m;) {
			if (i == n) break;
			if (CHK::PESwap(vehicle_a, vehicle_b, j, i, difa, difb)) {
				vehicle_a.cumlength += difa, vehicle_b.cumlength += difb;
				flag = true;
				m++;
				n--;
			} else {
				j++;
			}
		}
	}
}

bool OP::reverse(Vehicle& vehicle, const u32 from_pos, const u32 to_pos) {
	if (from_pos >= to_pos || from_pos == 0 || to_pos == vehicle.path.size() - 1) return false;                                                                                                       // 不合法
	double diflength = (vehicle.path.size() - from_pos) * (vehicle.path[to_pos]->dists[vehicle.path[from_pos - 1]->seq].dist - vehicle.path[from_pos]->dists[vehicle.path[from_pos - 1]->seq].dist);  // 反转后from之后的差值
	diflength += (vehicle.path.size() - to_pos - 1) * (vehicle.path[from_pos]->dists[vehicle.path[to_pos + 1]->seq].dist - vehicle.path[to_pos]->dists[vehicle.path[to_pos + 1]->seq].dist);          //// 反转后to之后的差值
	std::reverse(vehicle.path.begin() + from_pos, vehicle.path.begin() + to_pos);                                                                                                                     // 反转
	vehicle.cumlength += diflength;                                                                                                                                                                   // 更新距离（时间）
	return true;
}

bool OP::twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 from_a_pos, const u32 to_a_pos, const u32 from_b_pos, const u32 to_b_pos) {
	if (from_a_pos >= to_a_pos || from_a_pos == 0 || to_a_pos == vehicle_a.path.size() - 1) return false;
	if (from_b_pos >= to_b_pos || from_b_pos == 0 || to_b_pos == vehicle_b.path.size() - 1) return false;  // 不合法
	u32 diff{to_a_pos - from_a_pos}, diff_a{static_cast<u32>(vehicle_a.path.size()) - from_a_pos}, diff_b{static_cast<u32>(vehicle_b.path.size()) - from_b_pos};
	if (diff != (to_b_pos - from_b_pos)) return false;
	double diflength_a{0.0}, diflength_b{0.0};
	int difload{0};
	for (u32 i = 0; i + 1 < diff; i++) {
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
/*
double COST::insertb(std::vector<Node*>& route, Node* node, const u32 pos) {
    // 计算插入位置之后的时间（距离）
    double diflength = (route.size() - pos - 2) * (route[pos]->dists[node->seq].dist + route[pos + 1]->dists[node->seq].dist - route[pos]->dists[route[pos + 1]->seq].dist);
    // 计算插入的节点
    for (u32 i = 0; i < pos; i++) {
        diflength += route[i]->dists[route[i + 1]->seq].dist;
    }
    diflength += node->dists[route[pos]->seq].dist;
    return diflength;
}

double COST::insertf(std::vector<Node*>& route, Node* node, const u32 pos) {
    // 计算插入位置之前的时间（距离）
    double diflength = (route.size() - pos - 1) * (route[pos - 1]->dists[node->seq].dist + route[pos]->dists[node->seq].dist - route[pos - 1]->dists[route[pos]->seq].dist);
    // 计算插入的节点
    for (u32 i = 0; i < pos - 1; i++) {
        diflength += route[i]->dists[route[i + 1]->seq].dist;
    }
    diflength += node->dists[route[pos - 1]->seq].dist;
    return diflength;
}

double COST::erase(std::vector<Node*>& route, const u32 pos) {
    // 计算删除位置之后的时间（距离）
    double diflength = (route.size() - pos - 2) * (route[pos - 1]->dists[route[pos + 1]->seq].dist - route[pos]->dists[route[pos + 1]->seq].dist - route[pos - 1]->dists[route[pos]->seq].dist);
    // 计算删除的节点
    for (u32 i = 0; i < pos; i++) {
        diflength -= route[i]->dists[route[i + 1]->seq].dist;
    }
    return diflength;
}

double COST::reverse(Vehicle& vehicle, const u32 from_pos, const u32 to_pos) {
    double diflength = 0.0;
    return diflength;
}

double COST::swaptwo(Vehicle& vehicle, const u32 pos_i, const u32 pos_j) {
    double lpos_i = vehicle.path[pos_j]->dists[vehicle.path[pos_i - 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i - 1]->seq].dist;                                                 // pos_i左的差值
    double rpos_i = vehicle.path[pos_j]->dists[vehicle.path[pos_i + 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i + 1]->seq].dist;                                                 // pos_i右的差值
    double lpos_j = vehicle.path[pos_i]->dists[vehicle.path[pos_j - 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j - 1]->seq].dist;                                                 // pos_j左的差值
    double rpos_j = vehicle.path[pos_i]->dists[vehicle.path[pos_j + 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j + 1]->seq].dist;                                                 // pos_j右的差值
    double diflength = (vehicle.path.size() - pos_i) * lpos_i + (vehicle.path.size() - pos_i - 1) * rpos_i + (vehicle.path.size() - pos_j) * lpos_j + (vehicle.path.size() - pos_j - 1) * rpos_j;  // 路线（车辆）所有受影响的差值
    return diflength;
}

std::pair<double, double> COST::twoswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b) {
    double lpos_a = vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a - 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a - 1]->seq].dist;  // pos_a左的差值
    double rpos_a = vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a + 1]->seq].dist;  // pos_a右的差值
    double lpos_b = vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b - 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b - 1]->seq].dist;  // pos_b左的差值
    double rpos_b = vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist;  // pos_b右的差值
    double diflengtha = (vehicle_a.path.size() - pos_a) * lpos_a + (vehicle_a.path.size() - pos_a - 1) * rpos_a;                                            // 路线（车辆）a所有受影响的差值
    double diflengthb = (vehicle_b.path.size() - pos_b) * lpos_b + (vehicle_b.path.size() - pos_b - 1) * rpos_b;                                            // 路线（车辆）b所有受影响的差值
    return std::make_pair(diflengtha, diflengthb);
}

std::pair<double, double> COST::twostrswap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 from_a_pos, const u32 to_a_pos, const u32 from_b_pos, const u32 to_b_pos) {
    double diflengtha = 0.0;
    double diflengthb = 0.0;
    return std::make_pair(diflengtha, diflengthb);
}*/

u32 CHK::find(std::vector<Node*>& route, const u32 seq, u32 where) {
	for (u32 i = route.size() - 1; i > 0; i--) {
		if (route[i]->seq == seq) return i;
	}
#ifndef NDEBUG
	std::cerr << "没有找到节点:" << seq << "\n";
	throw "没有找到节点";
#endif
	std::exit(-1);
}

u32 CHK::find(std::vector<Node*>& route, Node* seq, u32 where) {
	for (u32 i = route.size() - 1; i > 0; i--) {
		if (route[i] == seq) return i;
	}
#ifndef NDEBUG
	std::cerr << "没有找到节点:" << seq->seq << " " << where << "\n";
	throw "没有找到节点";
#endif
	std::exit(-1);
}

bool CHK::oneMove(Vehicle& vehicle, const u32 h, const u32 d, double& out_d) {
	double dift{};  // d的后面
	out_d = -vehicle.cumlength;
	Node* node{vehicle.path[h]};
	if (h < d) {
		for (u32 i{h + 1}; i < d; i++) {  // d的前面
			vehicle.path[i - 1] = vehicle.path[i];
		}
		vehicle.path[d - 1] = node;
		for (u64 i{0}, n{vehicle.path.size() - 2}; i < n; i++) {  // 计算d前面的距离
			out_d += (n - i) * vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
		}
		u32 size = vehicle.path.size() - 1 - d;  // 0-a-b-c-h-d-0
		dift = (vehicle.path[d - 2]->dists[vehicle.path[d]->seq].dist - vehicle.path[d - 2]->dists[vehicle.path[d - 1]->seq].dist) * (size + 1);
		dift += (vehicle.path[d + 1]->dists[vehicle.path[d - 1]->seq].dist - vehicle.path[d + 1]->dists[vehicle.path[d]->seq].dist) * (size - 1);
		if (dift < 0) {                                // d的后面
			out_d += dift;
			if (out_d > 0) {                           // 不可行
				for (u32 i{d - 1}; i > h; i--) {       // 还原
					vehicle.path[i] = vehicle.path[i - 1];
				}
				vehicle.path[h] = node;
				return false;
			}
			vehicle.path[d - 1] = vehicle.path[d];  // 可行
			vehicle.path[d] = node;
			return true;
		}
		// d的前面
		if (out_d > 0) {                           // 不可行
			for (u32 i{d - 1}; i > h; i--) {       // 还原
				vehicle.path[i] = vehicle.path[i - 1];
			}
			vehicle.path[h] = node;
			return false;
		}
		return true;
	} else {
		for (u32 i{h - 1}; i > d; i--) {  // d的前面
			vehicle.path[i + 1] = vehicle.path[i];
		}
		vehicle.path[d + 1] = node;
		for (u64 i{0}, n{vehicle.path.size() - 2}; i < n; i++) {  // 计算d前面的距离
			out_d += (n - i) * vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
		}
		u32 size = vehicle.path.size() - 1 - d;  // 0-d-h-a-b-c-0
		dift = (vehicle.path[d - 1]->dists[vehicle.path[d + 1]->seq].dist - vehicle.path[d - 1]->dists[vehicle.path[d]->seq].dist) * size;
		dift += (vehicle.path[d + 2]->dists[vehicle.path[d]->seq].dist - vehicle.path[d + 2]->dists[vehicle.path[d + 1]->seq].dist) * (size - 2);
		if (dift < 0) {                                // d的前面
			out_d += dift;
			if (out_d > 0) {                           // 不可行
				for (u32 i{d + 1}; i < h; i++) {       // 还原
					vehicle.path[i] = vehicle.path[i + 1];
				}
				vehicle.path[h] = node;
				return false;
			}
			vehicle.path[d + 1] = vehicle.path[d];  // 可行
			vehicle.path[d] = node;
			return true;
		}
		// d的后面
		if (out_d > 0) {                           // 不可行
			for (u32 i{d + 1}; i < h; i++) {       // 还原
				vehicle.path[i] = vehicle.path[i + 1];
			}
			vehicle.path[h] = node;
			return false;
		}
		return true;
	}
}

bool CHK::oneMove(Vehicle& vehicle_h, Vehicle& vehicle_d, const u32 h, const u32 d, double& out_da, double& out_db) {
	if (vehicle_d.load + vehicle_h.path[h]->demand > vehicle_d.capacity) return false;
	out_da = (vehicle_h.path.size() - 2 - h) * (vehicle_h.path[h - 1]->dists[vehicle_h.path[h + 1]->seq].dist - vehicle_h.path[h]->dists[vehicle_h.path[h + 1]->seq].dist - vehicle_h.path[h - 1]->dists[vehicle_h.path[h]->seq].dist);
	for (u32 i{0}; i < h; i++) {  // 计算路径h
		out_da -= vehicle_h.path[i]->dists[vehicle_h.path[i + 1]->seq].dist;
	}
	u32 size = vehicle_d.path.size() - 1 - d;
	if (size == 0) {  // vehicle_d 为空
		out_db += vehicle_d.path[d - 1]->dists[vehicle_h.path[h]->seq].dist;
		if (out_da + out_db > 0) return false;
		Node* node{vehicle_h.path[h]};  // 移动d
		vehicle_d.path.emplace(vehicle_d.path.begin() + d, node);
	} else {
		out_db = size * (vehicle_d.path[d]->dists[vehicle_h.path[h]->seq].dist + vehicle_d.path[d - 1]->dists[vehicle_h.path[h]->seq].dist - vehicle_d.path[d]->dists[vehicle_d.path[d - 1]->seq].dist);
		for (u32 i{1}; i < d; i++) {  // 计算路径d前插
			out_db += vehicle_d.path[i - 1]->dists[vehicle_d.path[i]->seq].dist;
		}
		out_db += vehicle_d.path[d - 1]->dists[vehicle_h.path[h]->seq].dist;
		// 计算路径d后插
		double dif{(vehicle_d.path[d - 1]->dists[vehicle_d.path[d]->seq].dist - vehicle_d.path[d - 1]->dists[vehicle_h.path[h]->seq].dist) * (size + 1)};
		dif += (vehicle_d.path[d + 1]->dists[vehicle_h.path[h]->seq].dist - vehicle_d.path[d]->dists[vehicle_d.path[d + 1]->seq].dist) * (size - 1);
		if (dif < 0) {  // 后插可行
			out_db += dif;
			if (out_da + out_db > 0) return false;
			Node* node{vehicle_h.path[h]};  // 移动d
			vehicle_d.path.emplace(vehicle_d.path.begin() + d + 1, node);
		} else {                                  // 前插可行
			if (out_da + out_db > 0) return false;
			Node* node{vehicle_h.path[h]};  // 移动d
			vehicle_d.path.emplace(vehicle_d.path.begin() + d, node);
		}
	}
	vehicle_d.load += vehicle_h.path[h]->demand, vehicle_h.load -= vehicle_h.path[h]->demand;
	vehicle_h.path.erase(vehicle_h.path.begin() + h);
	return true;
}

bool CHK::twoSwap(Vehicle& vehicle, const u32 pos_i, const u32 pos_j, double& out_d) {
	int flag = pos_i - pos_j;
	if (flag == -1) {  // 相邻点特殊
		u32 size = vehicle.path.size() - 1 - pos_i;
		out_d = (vehicle.path[pos_i - 1]->dists[vehicle.path[pos_j]->seq].dist - vehicle.path[pos_i - 1]->dists[vehicle.path[pos_i]->seq].dist) * size;
		out_d += (vehicle.path[pos_i]->dists[vehicle.path[pos_j + 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j + 1]->seq].dist) * (size - 2);
	} else if (flag == 1) {
		u32 size = vehicle.path.size() - 1 - pos_j;
		out_d = (vehicle.path[pos_j - 1]->dists[vehicle.path[pos_i]->seq].dist - vehicle.path[pos_j - 1]->dists[vehicle.path[pos_j]->seq].dist) * size;
		out_d += (vehicle.path[pos_j]->dists[vehicle.path[pos_i + 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i + 1]->seq].dist) * (size - 2);
	} else {
		u32 size = vehicle.path.size() - 1;
		out_d = (vehicle.path[pos_i - 1]->dists[vehicle.path[pos_j]->seq].dist - vehicle.path[pos_i - 1]->dists[vehicle.path[pos_i]->seq].dist) * (size - pos_i);
		out_d += (vehicle.path[pos_j]->dists[vehicle.path[pos_i + 1]->seq].dist - vehicle.path[pos_i]->dists[vehicle.path[pos_i + 1]->seq].dist) * (size - pos_i - 1);
		out_d += (vehicle.path[pos_j - 1]->dists[vehicle.path[pos_i]->seq].dist - vehicle.path[pos_j - 1]->dists[vehicle.path[pos_j]->seq].dist) * (size - pos_j);
		out_d += (vehicle.path[pos_i]->dists[vehicle.path[pos_j + 1]->seq].dist - vehicle.path[pos_j]->dists[vehicle.path[pos_j + 1]->seq].dist) * (size - pos_j - 1);
	}
	if (out_d > 0)
		return false;
	else
		return true;
}

bool CHK::twoSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db) {
	int difload = vehicle_a.path[pos_a]->demand - vehicle_b.path[pos_b]->demand;
	if ((vehicle_b.load + difload) > vehicle_b.capacity || (vehicle_a.load - difload) > vehicle_a.capacity) return false;  // 超载
	u32 size_a = vehicle_a.path.size() - 1, size_b = vehicle_b.path.size() - 1;
	out_da = (vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist - vehicle_a.path[pos_a - 1]->dists[vehicle_a.path[pos_a]->seq].dist) * (size_a - pos_a);
	out_db = (vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist) * (size_b - pos_b);
	out_da += (vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a + 1]->seq].dist) * (size_a - pos_a - 1);
	out_db += (vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist) * (size_b - pos_b - 1);
	if (out_da + out_db > 0)
		return false;
	vehicle_a.load -= difload, vehicle_b.load += difload;
	return true;
}

bool CHK::twoCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db) {
	u32 index_a = vehicle_a.path.size() - 2 - pos_a, index_b = vehicle_b.path.size() - 2 - pos_b;
	int difload{};
	if (index_a > index_b) {  // 判断容量约束
		u32 i{0};
		for (; i < index_b; i++) {
			difload += vehicle_a.path[pos_a + i]->demand - vehicle_b.path[pos_b + i]->demand;
		}
		for (; i < index_a; i++) {
			difload += vehicle_a.path[pos_a + i]->demand;
		}
	} else {
		u32 i{0};
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
	for (u32 i{1}; i < pos_a; i++) {  // 计算a距离差值
		difa += vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		out_da += difa;
	}
	difa += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist;
	out_da += difa;
	for (u32 i{pos_b}, n{index_b + pos_b}; i < n; i++) {  // 计算a距离差值
		difa += vehicle_b.path[i]->dists[vehicle_b.path[i + 1]->seq].dist;
		out_da += difa;
	}

	for (u32 i{1}; i < pos_b; i++) {  // 计算b距离差值
		difb += vehicle_b.path[i - 1]->dists[vehicle_b.path[i]->seq].dist;
		out_db += difb;
	}
	difb += vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist;
	out_db += difb;
	for (u32 i{pos_a}, n{index_a + pos_a}; i < n; i++) {  // 计算b距离差值
		difb += vehicle_a.path[i]->dists[vehicle_a.path[i + 1]->seq].dist;
		out_db += difb;
	}
	if (out_da + out_db > 0) return false;  // 不满足约束
	index_a++, index_b++;
	vehicle_a.path.pop_back();
	vehicle_b.path.pop_back();
	if (index_a > index_b) {                // 移动
		u32 i{0};
		for (; i < index_b; i++) {
			std::swap(vehicle_a.path[pos_a + i], vehicle_b.path[pos_b + i]);
		}
		for (; i < index_a; i++) {
			vehicle_b.path.emplace_back(vehicle_a.path[pos_a + i]);
		}
		vehicle_b.path.emplace_back(vehicle_b.path.front());
		vehicle_a.path.erase(vehicle_a.path.begin() + pos_a + index_b, vehicle_a.path.end());
		vehicle_a.path.emplace_back(vehicle_a.path.front());
	} else {
		u32 i{0};
		for (; i < index_a; i++) {
			std::swap(vehicle_a.path[pos_a + i], vehicle_b.path[pos_b + i]);
		}
		for (; i < index_b; i++) {
			vehicle_a.path.emplace_back(vehicle_b.path[pos_b + i]);
		}
		vehicle_a.path.emplace_back(vehicle_a.path.front());
		vehicle_b.path.erase(vehicle_b.path.begin() + pos_b + index_a, vehicle_b.path.end());
		vehicle_b.path.emplace_back(vehicle_b.path.front());
	}
	return true;
}

bool CHK::reverse(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, double& out_d) {
	u32 f{pos_f}, t{pos_t}, size = vehicle.path.size() - 2;
	// if (pos_f < pos_t) {  // 保证 left < right
	//	f = pos_f, t = pos_t;
	// } else {
	//	f = pos_t, t = pos_f;
	// }
	/*out_d = (size - f) * (vehicle.path[f - 1]->dists[vehicle.path[t]->seq].dist - vehicle.path[f - 1]->dists[vehicle.path[f]->seq].dist);
	out_d += (size - 1 - t) * (vehicle.path[t + 1]->dists[vehicle.path[f]->seq].dist - vehicle.path[t + 1]->dists[vehicle.path[t]->seq].dist);
	for (u32 l{f}, r{t}; l < r; l++, r--) {
	    out_d += ((vehicle.path[t]->dists[vehicle.path[t - 1]->seq].dist - vehicle.path[f]->dists[vehicle.path[f + 1]->seq].dist) * (r - l - 1));
	}*/
	// if (out_d > 0) return false;  // 不合适
	out_d = -vehicle.cumlength;
	double cumlength{};
	for (u64 i{1}; i < f; i++) {
		cumlength += vehicle.path[i]->dists[vehicle.path[i - 1]->seq].dist;
		out_d += cumlength;
	}
	cumlength += vehicle.path[f - 1]->dists[vehicle.path[t]->seq].dist;
	out_d += cumlength;
	for (u64 i{t}; i > f; i--) {
		cumlength += vehicle.path[i]->dists[vehicle.path[i - 1]->seq].dist;
		out_d += cumlength;
	}
	if (size != t) {
		cumlength += vehicle.path[t + 1]->dists[vehicle.path[f]->seq].dist;
		out_d += cumlength;
		for (u64 i{t + 1}; i < size; i++) {
			cumlength += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
			out_d += cumlength;
		}
	}
	if (out_d > 0) return false;                                           // 不合适
	std::reverse(vehicle.path.begin() + f, vehicle.path.begin() + t + 1);  // 反转
	return true;
}

bool CHK::strSwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_b_f, const u32 len_t, double& out_da, double& out_db) {
	u32 pos_a_t{pos_a_f + len_t}, pos_b_t{pos_b_f + len_t}, size_a = vehicle_a.path.size() - 1 - pos_a_f, size_b = vehicle_b.path.size() - 1 - pos_b_f;
	out_da = size_a * (vehicle_b.path[pos_b_f]->dists[vehicle_a.path[pos_a_f - 1]->seq].dist - vehicle_a.path[pos_a_f]->dists[vehicle_a.path[pos_a_f - 1]->seq].dist);
	out_da += (size_a - 1 - len_t) * (vehicle_b.path[pos_b_t]->dists[vehicle_a.path[pos_a_t + 1]->seq].dist - vehicle_a.path[pos_a_t]->dists[vehicle_a.path[pos_a_t + 1]->seq].dist);
	out_db = size_b * (vehicle_a.path[pos_a_f]->dists[vehicle_b.path[pos_b_f - 1]->seq].dist - vehicle_b.path[pos_b_f]->dists[vehicle_b.path[pos_b_f - 1]->seq].dist);
	out_db += (size_b - 1 - len_t) * (vehicle_a.path[pos_a_t]->dists[vehicle_b.path[pos_b_t + 1]->seq].dist - vehicle_b.path[pos_b_t]->dists[vehicle_b.path[pos_b_t + 1]->seq].dist);
	double dif_len{};
	int dif_load = vehicle_b.path[pos_b_t]->demand - vehicle_a.path[pos_a_t]->demand;
	for (u32 i{0}; i < len_t; i++) {
		dif_len = vehicle_b.path[pos_b_f + i]->dists[vehicle_b.path[pos_b_f + i + 1]->seq].dist - vehicle_a.path[pos_a_f + i]->dists[vehicle_a.path[pos_a_f + i + 1]->seq].dist;
		dif_load += vehicle_b.path[pos_b_f + i]->demand - vehicle_a.path[pos_a_f + i]->demand;
		out_da += (size_a - i - 1) * dif_len;
		out_db -= (size_b - i - 1) * dif_len;
	}
	if (vehicle_a.load + dif_load > vehicle_a.capacity || vehicle_b.load - dif_load > vehicle_b.capacity) return false;  // 超载
	if (out_da + out_db > 0) return false;                                                                               // 不合适
	// Node* node{nullptr};                                                                                        // 开始移动
	for (u32 i{0}; i <= len_t; i++) {
		std::swap(vehicle_a.path[pos_a_f + i], vehicle_b.path[pos_b_f + i]);
		//	node = vehicle_a.path[pos_a_f + i];
		//	vehicle_a.path[pos_a_f + i] = vehicle_b.path[pos_b_f + i];
		//	vehicle_b.path[pos_b_f + i] = node;
	}
	vehicle_a.load += dif_load, vehicle_b.load -= dif_load;
	return true;
}

bool CHK::strCross(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_a_t, const u32 pos_b_f, const u32 pos_b_t, double& out_da, double& out_db) {
	return true;
}

bool CHK::strSwapR(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_b_f, const u32 len_t, double& out_da, double& out_db) {
	u32 pos_a_t{pos_a_f + len_t}, pos_b_t{pos_b_f + len_t}, size_a = vehicle_a.path.size() - 1 - pos_a_f, size_b = vehicle_b.path.size() - 1 - pos_b_f;
	out_da = size_a * (vehicle_b.path[pos_b_t]->dists[vehicle_a.path[pos_a_f - 1]->seq].dist - vehicle_a.path[pos_a_f]->dists[vehicle_a.path[pos_a_f - 1]->seq].dist);
	out_da += (size_a - 1 - len_t) * (vehicle_b.path[pos_b_f]->dists[vehicle_a.path[pos_a_t + 1]->seq].dist - vehicle_a.path[pos_a_t]->dists[vehicle_a.path[pos_a_t + 1]->seq].dist);
	out_db = size_b * (vehicle_a.path[pos_a_t]->dists[vehicle_b.path[pos_b_f - 1]->seq].dist - vehicle_b.path[pos_b_f]->dists[vehicle_b.path[pos_b_f - 1]->seq].dist);
	out_db += (size_b - 1 - len_t) * (vehicle_a.path[pos_a_f]->dists[vehicle_b.path[pos_b_t + 1]->seq].dist - vehicle_b.path[pos_b_t]->dists[vehicle_b.path[pos_b_t + 1]->seq].dist);
	int dif_load = vehicle_b.path[pos_b_f]->demand - vehicle_a.path[pos_a_t]->demand;
	for (u32 i{0}; i < len_t; i++) {
		dif_load += vehicle_b.path[pos_b_t - i]->demand - vehicle_a.path[pos_a_f + i]->demand;
		out_da += (size_a - i - 1) * (vehicle_b.path[pos_b_t - i]->dists[vehicle_b.path[pos_b_t - i - 1]->seq].dist - vehicle_a.path[pos_a_f + i]->dists[vehicle_a.path[pos_a_f + i + 1]->seq].dist);
		out_db += (size_b - i - 1) * (vehicle_a.path[pos_a_t - i]->dists[vehicle_a.path[pos_a_t - i - 1]->seq].dist - vehicle_b.path[pos_b_f + i]->dists[vehicle_b.path[pos_b_f + i + 1]->seq].dist);
	}
	if (vehicle_a.load + dif_load > vehicle_a.capacity || vehicle_b.load - dif_load > vehicle_b.capacity) return false;  // 超载
	if (out_da + out_db > 0) return false;                                                                               // 不合适
	for (u32 i{0}; i <= len_t; i++) {                                                                                    // 开始移动
		std::swap(vehicle_a.path[pos_a_f + i], vehicle_b.path[pos_b_t - i]);
	}
	vehicle_a.load += dif_load, vehicle_b.load -= dif_load;
	return true;
}

bool CHK::strMove(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, const u32 pos, double& out_d) {
	double dif_h{}, dif_t{}, out_h{-vehicle.cumlength}, out_t{}, tmp{};
	u32 size = vehicle.path.size() - 2;
	if (pos_t < pos) {
		if (pos - pos_t == 1) {  // 特殊情况，只更新后插
			// 这里把out_h 当作out_t
			for (u32 i{1}; i < pos_f; i++) {  // 0 ~ pos_f
				dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				out_h += dif_h;
			}
			dif_h += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t + 1]->seq].dist;
			out_h += dif_h;
			dif_h += vehicle.path[pos]->dists[vehicle.path[pos_f]->seq].dist;
			out_h += dif_h;
			for (u32 i{pos_f}; i < pos_t; i++) {  // pos_f ~ pos_t
				dif_h += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				out_h += dif_h;
			}
			if (pos != size) {  // 到终点
				dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos + 1]->seq].dist;
				out_h += dif_h;
				for (u32 i{pos + 1}; i < size; i++) {  // pos ~ end()
					dif_h += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
					out_h += dif_h;
				}
			}
			if (out_h > 0) return false;
			out_d = out_h;
			std::rotate(vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1, vehicle.path.begin() + pos + 1);
			return true;
		} else {
			for (u32 i{1}; i < pos_f; i++) {  // 0 ~ pos_f
				dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				out_h += dif_h;
			}
			dif_h += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t + 1]->seq].dist;
			out_h += dif_h;
			for (u32 i{pos_t + 2}; i < pos; i++) {  // pos_t ~ pos
				dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				out_h += dif_h;
			}
			dif_t = dif_h + vehicle.path[pos - 1]->dists[vehicle.path[pos]->seq].dist;
			out_t = out_h + dif_t;
			dif_t += vehicle.path[pos]->dists[vehicle.path[pos_f]->seq].dist;
			out_t += dif_t;
			dif_h += vehicle.path[pos - 1]->dists[vehicle.path[pos_f]->seq].dist;
			out_h += dif_h;
			for (u32 i{pos_f}; i < pos_t; i++) {  // pos_f ~ pos_t
				tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_h += dif_h, out_t += dif_t;
			}
			dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos]->seq].dist;
			out_h += dif_h;
			if (pos != size) {  // 到终点
				dif_h += vehicle.path[pos]->dists[vehicle.path[pos + 1]->seq].dist;
				out_h += dif_h;
				dif_t += vehicle.path[pos_t]->dists[vehicle.path[pos + 1]->seq].dist;
				out_t += dif_t;
				for (u32 i{pos + 1}; i < size; i++) {  // pos ~ end()
					tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
					dif_h += tmp, dif_t += tmp;
					out_h += dif_h, out_t += dif_t;
				}
			}
		}
		if (out_h < out_t) {  // 前插
			out_d = out_h;
			if ((out_d = out_h) > 0) return false;
			std::rotate(vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1, vehicle.path.begin() + pos);
		} else {  // 后插
			out_d = out_t;
			if ((out_d = out_t) > 0) return false;
			std::rotate(vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1, vehicle.path.begin() + pos + 1);
		}
		return true;
	} else {
		if (pos_f - pos == 1) {  // 特殊情况，只更新前插
			for (u32 i{1}; i < pos; i++) {
				dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				out_h += dif_h;
			}
			dif_h += vehicle.path[pos - 1]->dists[vehicle.path[pos_f]->seq].dist;
			out_h += dif_h;
			for (u32 i{pos_f}; i < pos_t; i++) {  // pos_f ~ pos_t
				dif_h += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				out_h += dif_h;
			}
			dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos]->seq].dist;
			out_h += dif_h;
			if (pos_t != size) {  // 到终点
				dif_h += vehicle.path[pos]->dists[vehicle.path[pos_t + 1]->seq].dist;
				out_h += dif_h;
				for (u32 i{pos_t + 1}; i < size; i++) {  // pos_t ~ end()
					dif_h += vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
					out_h += dif_h;
				}
			}
			if (out_h > 0) return false;
			out_d = out_h;
			std::rotate(vehicle.path.begin() + pos, vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1);
			return true;
		} else {
			for (u32 i{1}; i < pos; i++) {  // 0 ~ pos
				dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				out_h += dif_h;
			}
			dif_t = dif_h + vehicle.path[pos]->dists[vehicle.path[pos - 1]->seq].dist;
			out_t = out_h + dif_t;
			dif_t += vehicle.path[pos]->dists[vehicle.path[pos_f]->seq].dist;
			out_t += dif_t;
			dif_h += vehicle.path[pos - 1]->dists[vehicle.path[pos_f]->seq].dist;
			out_h += dif_h;
			for (u32 i{pos_f}; i < pos_t; i++) {  // pos_f ~ pos_t
				tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_h += dif_h, out_t += dif_t;
			}
			dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos]->seq].dist;
			out_h += dif_h;
			dif_h += vehicle.path[pos]->dists[vehicle.path[pos + 1]->seq].dist;
			out_h += dif_h;
			dif_t += vehicle.path[pos_t]->dists[vehicle.path[pos + 1]->seq].dist;
			out_t += dif_t;
			for (u32 i{pos + 2}; i < pos_f; i++) {  // pos ~ pos_f
				tmp = vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_h += dif_h, out_t += dif_t;
			}
			if (pos_t != size) {  // 到终点
				tmp = vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t + 1]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_h += dif_h, out_t += dif_t;
				for (u32 i{pos_t + 1}; i < size; i++) {  // pos_f ~ end()
					tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
					dif_h += tmp, dif_t += tmp;
					out_h += dif_h, out_t += dif_t;
				}
			}
			if (out_h < out_t) {  // 前插
				out_d = out_h;
				if (out_d > 0) return false;
				std::rotate(vehicle.path.begin() + pos, vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1);
			} else {  // 后插
				out_d = out_t;
				if (out_d > 0) return false;
				std::rotate(vehicle.path.begin() + pos + 1, vehicle.path.begin() + pos_f, vehicle.path.begin() + pos_t + 1);
			}
			return true;
		}
	}
}

bool CHK::strMove(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a_f, const u32 pos_a_t, const u32 pos_b, double& out_da, double& out_db) {
	u32 dif_load{};
	for (u32 i{pos_a_f}; i <= pos_a_t; i++) {
		dif_load += vehicle_a.path[i]->demand;
	}
	if (vehicle_b.load + dif_load > vehicle_b.capacity) return false;  // 超载
	out_da = -vehicle_a.cumlength, out_db = -vehicle_b.cumlength;
	double difa{}, difb_h{}, difb_t{}, out_db_h{}, out_db_t{}, tmp{};
	u32 size_a = vehicle_a.path.size() - 2, size_b = vehicle_b.path.size() - 2;
	// 计算vehicle_a
	for (u32 i{1}; i < pos_a_f; i++) {  // 0 ~ pos_a_f
		difa += vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		out_da += difa;
	}
	if (pos_a_t != size_a) {  // pos_a_t 在末尾
		difa += vehicle_a.path[pos_a_f - 1]->dists[vehicle_a.path[pos_a_t + 1]->seq].dist;
		out_da += difa;
		for (u32 i{pos_a_t + 1}; i < size_a; i++) {  // pos_a_t ~ end()
			difa += vehicle_a.path[i]->dists[vehicle_a.path[i + 1]->seq].dist;
			out_da += difa;
		}
	}
	// 计算vehicle_b
	for (u32 i{1}; i < pos_b; i++) {  // 0 ~ pos_b
		difb_h += vehicle_b.path[i - 1]->dists[vehicle_b.path[i]->seq].dist;
		out_db_h += difb_h;
	}
	difb_t = vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist + difb_h;
	out_db_t = difb_t + out_db_h;
	difb_h += vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a_f]->seq].dist;
	out_db_h += difb_h;
	difb_t += vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a_f]->seq].dist;
	out_db_t += difb_t;
	for (u32 i{pos_a_f}; i < pos_a_t; i++) {  // pos_a_f ~ pos_a_t
		tmp = vehicle_a.path[i]->dists[vehicle_a.path[i + 1]->seq].dist;
		difb_h += tmp, difb_t += tmp;
		out_db_h += difb_h, out_db_t += difb_t;
	}
	difb_h += vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a_t]->seq].dist;
	out_db_h += difb_h;
	if (pos_b != size_b) {  // pos_b 在末尾
		difb_h += vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist;
		out_db_h += difb_h;
		difb_t += vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a_t]->seq].dist;
		out_db_t += difb_t;
		for (u32 i{pos_b + 1}; i < size_b; i++) {  // pos_b ~ end()
			tmp = vehicle_b.path[i]->dists[vehicle_b.path[i + 1]->seq].dist;
			difb_h += tmp, difb_t += tmp;
			out_db_h += difb_h, out_db_t += difb_t;
		}
	}
	if (out_db_h < out_db_t) {  // 前插
		out_db += out_db_h;
		if (out_da + out_db > 0) return false;
		vehicle_b.path.insert(vehicle_b.path.begin() + pos_b, vehicle_a.path.begin() + pos_a_f, vehicle_a.path.begin() + pos_a_t + 1);
	} else {  // 后插
		out_db += out_db_t;
		if (out_da + out_db > 0) return false;
		vehicle_b.path.insert(vehicle_b.path.begin() + pos_b + 1, vehicle_a.path.begin() + pos_a_f, vehicle_a.path.begin() + pos_a_t + 1);
	}
	vehicle_a.load -= dif_load, vehicle_b.load += dif_load;
	vehicle_a.path.erase(vehicle_a.path.begin() + pos_a_f, vehicle_a.path.begin() + pos_a_t + 1);
	return true;
}

bool CHK::PESwap(Vehicle& vehicle, const u32 pos_f, const u32 pos_t, double& out_d) {
	double dif_h{}, dif_t{}, out_d_h{}, out_d_t{}, tmp{};
	out_d = -vehicle.cumlength;
	u32 size = vehicle.path.size() - 2;
	if (pos_f < pos_t) {
		for (u32 i{1}; i < pos_f; i++) {
			dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
			out_d_h += dif_h;
		}
		dif_t = dif_h, out_d_t = out_d_h;
		// 前边
		dif_h += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t - 1]->seq].dist;
		out_d_h += dif_h;
		dif_h += vehicle.path[pos_t - 1]->dists[vehicle.path[pos_t]->seq].dist;
		out_d_h += dif_h;
		dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos_f + 1]->seq].dist;
		out_d_h += dif_h;
		// 后边
		dif_t += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t]->seq].dist;
		out_d_t += dif_t;
		dif_t += vehicle.path[pos_t]->dists[vehicle.path[pos_t + 1]->seq].dist;
		out_d_t += dif_t;
		dif_t += vehicle.path[pos_t + 1]->dists[vehicle.path[pos_f + 1]->seq].dist;
		out_d_t += dif_t;
		for (u32 i{pos_f + 2}; i < pos_t - 1; i++) {
			tmp = vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
			dif_h += tmp, dif_t += tmp;
			out_d_h += dif_h, out_d_t += dif_t;
		}
		// 前边
		if (pos_t - pos_f != 2) {  // 不相邻
			dif_h += vehicle.path[pos_t - 2]->dists[vehicle.path[pos_f]->seq].dist;
			out_d_h += dif_h;
		}
		dif_h += vehicle.path[pos_f]->dists[vehicle.path[pos_t + 1]->seq].dist;
		out_d_h += dif_h;
		// 后边
		if (pos_t - pos_f != 2) {  // 不相邻
			dif_t += vehicle.path[pos_t - 2]->dists[vehicle.path[pos_t - 1]->seq].dist;
			out_d_t += dif_t;
		}
		dif_t += vehicle.path[pos_t - 1]->dists[vehicle.path[pos_f]->seq].dist;
		out_d_t += dif_t;
		if (pos_t != size - 1) {  // 末尾
			// 前边
			dif_h += vehicle.path[pos_t + 1]->dists[vehicle.path[pos_t + 2]->seq].dist;
			out_d_h += dif_h;
			// 后边
			dif_t += vehicle.path[pos_f]->dists[vehicle.path[pos_t + 2]->seq].dist;
			out_d_t += dif_t;
			for (u32 i{pos_t + 2}; i < size; i++) {
				tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_d_h += dif_h, out_d_t += dif_t;
			}
		}
		if (out_d_h < out_d_t) {
			out_d += out_d_h;
			// if (out_d > 0) return false;
			std::swap(vehicle.path[pos_f], vehicle.path[pos_t]);
			vehicle.path.emplace(vehicle.path.begin() + pos_f, vehicle.path[pos_t - 1]);
			vehicle.path.erase(vehicle.path.begin() + pos_t);
		} else {
			out_d += out_d_t;
			// if (out_d > 0) return false;
			std::swap(vehicle.path[pos_f], vehicle.path[pos_t]);
			vehicle.path.emplace(vehicle.path.begin() + pos_f + 1, vehicle.path[pos_t + 1]);
			vehicle.path.erase(vehicle.path.begin() + pos_t + 2);
		}
	} else {
		for (u32 i{1}; i < pos_t - 1; i++) {
			dif_h += vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
			out_d_h += dif_h;
		}
		dif_t = dif_h, out_d_t = out_d_h;
		// 前边
		dif_h += vehicle.path[pos_t - 2]->dists[vehicle.path[pos_f]->seq].dist;
		out_d_h += dif_h;
		dif_h += vehicle.path[pos_f]->dists[vehicle.path[pos_t + 1]->seq].dist;
		out_d_h += dif_h;
		if (pos_f - pos_t != 2) {  // 不相邻
			dif_h += vehicle.path[pos_t + 1]->dists[vehicle.path[pos_t + 2]->seq].dist;
			out_d_h += dif_h;
		}
		// 后边
		dif_t += vehicle.path[pos_t - 2]->dists[vehicle.path[pos_t - 1]->seq].dist;
		out_d_t += dif_t;
		dif_t += vehicle.path[pos_t - 1]->dists[vehicle.path[pos_f]->seq].dist;
		out_d_t += dif_t;
		if (pos_f - pos_t != 2) {  // 不相邻
			dif_t += vehicle.path[pos_f]->dists[vehicle.path[pos_t + 2]->seq].dist;
			out_d_t += dif_t;
		}
		for (u32 i{pos_t + 3}; i < pos_f; i++) {
			tmp = vehicle.path[i - 1]->dists[vehicle.path[i]->seq].dist;
			dif_h += tmp, dif_t += tmp;
			out_d_h += dif_h, out_d_t += dif_t;
		}
		// 前边
		dif_h += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t - 1]->seq].dist;
		out_d_h += dif_h;
		dif_h += vehicle.path[pos_t - 1]->dists[vehicle.path[pos_t]->seq].dist;
		out_d_h += dif_h;
		// 后边
		if (pos_f - pos_t != 2) {  // 不相邻
			dif_t += vehicle.path[pos_f - 1]->dists[vehicle.path[pos_t]->seq].dist;
			out_d_t += dif_t;
		} else {
			dif_t += vehicle.path[pos_f]->dists[vehicle.path[pos_t]->seq].dist;
			out_d_t += dif_t;
		}
		dif_t += vehicle.path[pos_t]->dists[vehicle.path[pos_t + 1]->seq].dist;
		out_d_t += dif_t;
		if (pos_f != size) {  // 末尾
			// 前边
			dif_h += vehicle.path[pos_t]->dists[vehicle.path[pos_f + 1]->seq].dist;
			out_d_h += dif_h;
			// 后边
			dif_t += vehicle.path[pos_t + 1]->dists[vehicle.path[pos_f + 1]->seq].dist;
			out_d_t += dif_t;
			for (u32 i{pos_f + 1}; i < size; i++) {
				tmp = vehicle.path[i]->dists[vehicle.path[i + 1]->seq].dist;
				dif_h += tmp, dif_t += tmp;
				out_d_h += dif_h, out_d_t += dif_t;
			}
		}
		if (out_d_h < out_d_t) {
			out_d += out_d_h;
			// if (out_d > 0) return false;
			std::swap(vehicle.path[pos_f], vehicle.path[pos_t]);
			vehicle.path.emplace(vehicle.path.begin() + pos_f, vehicle.path[pos_t - 1]);
			vehicle.path.erase(vehicle.path.begin() + pos_t - 1);
		} else {
			out_d += out_d_t;
			// if (out_d > 0) return false;
			std::swap(vehicle.path[pos_f], vehicle.path[pos_t]);
			vehicle.path.emplace(vehicle.path.begin() + pos_f + 1, vehicle.path[pos_t + 1]);
			vehicle.path.erase(vehicle.path.begin() + pos_t + 1);
		}
	}
	return true;
}

bool CHK::PESwap(Vehicle& vehicle_a, Vehicle& vehicle_b, const u32 pos_a, const u32 pos_b, double& out_da, double& out_db) {
	int dif_load = vehicle_b.path[pos_b]->demand - vehicle_a.path[pos_a]->demand;
	bool flag_h{(vehicle_a.load + dif_load + vehicle_b.path[pos_b - 1]->demand > vehicle_a.capacity) || (vehicle_b.load - dif_load - vehicle_b.path[pos_b - 1]->demand > vehicle_b.capacity)};
	bool flag_t{(vehicle_a.load + dif_load + vehicle_b.path[pos_b + 1]->demand > vehicle_a.capacity) || (vehicle_b.load - dif_load - vehicle_b.path[pos_b + 1]->demand > vehicle_b.capacity)};
	if (flag_h && flag_t) return false;
	u32 size_a = vehicle_a.path.size() - 2, size_b = vehicle_b.path.size() - 2;
	// 先交换
	double dif_a{(vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist - vehicle_a.path[pos_a - 1]->dists[vehicle_a.path[pos_a]->seq].dist) * (size_a - pos_a + 1)};
	double dif_b{(vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist) * (size_b - pos_b + 1)};
	dif_a += (vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_a.path[pos_a]->dists[vehicle_a.path[pos_a + 1]->seq].dist) * (size_a - pos_a);
	dif_b += (vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist) * (size_b - pos_b);
	if (flag_h) {  // 后插
		// A
		out_da = (size_a - pos_a) * (vehicle_b.path[pos_b + 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist);
		for (u32 i{1}; i <= pos_a - 1; i++) {
			out_da += vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		}
		out_da += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist;
		// B
		out_db = (size_b - pos_b - 1) * (vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 2]->seq].dist - vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b + 1]->dists[vehicle_b.path[pos_b + 2]->seq].dist);
		for (u32 i{0}; i < pos_b - 1; i++) {
			out_db -= vehicle_b.path[i]->dists[vehicle_b.path[i + 1]->seq].dist;
		}
		out_db -= vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist + vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist;
		out_da += dif_a, out_db += dif_b;
		if (out_da + out_db > 0) return false;
		//  交换
		vehicle_a.load += dif_load + vehicle_b.path[pos_b + 1]->demand;
		vehicle_b.load -= dif_load + vehicle_b.path[pos_b + 1]->demand;
		std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);
		vehicle_a.path.emplace(vehicle_a.path.begin() + pos_a + 1, vehicle_b.path[pos_b + 1]);
		vehicle_b.path.erase(vehicle_b.path.begin() + pos_b + 1);
		return true;
	}
	if (flag_t) {  // 前插
		// A
		out_da = (size_a - pos_a + 1) * (vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a - 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a - 1]->seq].dist);
		for (u32 i{1}; i <= pos_a - 1; i++) {
			out_da += vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		}
		out_da += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b - 1]->seq].dist;
		// B
		out_db = (size_b - pos_b + 1) * (vehicle_b.path[pos_b - 2]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 2]->dists[vehicle_b.path[pos_b - 1]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist);
		for (u32 i{1}; i <= pos_b - 1; i++) {
			out_db -= vehicle_b.path[i - 1]->dists[vehicle_b.path[i]->seq].dist;
		}
		// out_db -= vehicle_b.path[pos_b - 2]->dists[vehicle_b.path[pos_b - 1]->seq].dist;
		out_da += dif_a, out_db += dif_b;
		if (out_da + out_db > 0) return false;
		//  交换
		vehicle_a.load += dif_load + vehicle_b.path[pos_b - 1]->demand;
		vehicle_b.load -= dif_load + vehicle_b.path[pos_b - 1]->demand;
		std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);
		vehicle_a.path.emplace(vehicle_a.path.begin() + pos_a, vehicle_b.path[pos_b - 1]);
		vehicle_b.path.erase(vehicle_b.path.begin() + pos_b - 1);
		return true;
	}
	// 全计算
	// A
	double tmp{};
	double out_a_h{(size_a - pos_a + 1) * (vehicle_b.path[pos_b - 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a - 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a - 1]->seq].dist)};
	double out_a_t{(size_a - pos_a) * (vehicle_b.path[pos_b + 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a + 1]->seq].dist - vehicle_b.path[pos_b]->dists[vehicle_a.path[pos_a + 1]->seq].dist)};
	for (u32 i{1}; i <= pos_a - 1; i++) {
		tmp = vehicle_a.path[i - 1]->dists[vehicle_a.path[i]->seq].dist;
		out_a_t += tmp, out_a_h += tmp;
	}
	out_a_h += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b - 1]->seq].dist;
	out_a_t += vehicle_a.path[pos_a - 1]->dists[vehicle_b.path[pos_b]->seq].dist + vehicle_b.path[pos_b]->dists[vehicle_b.path[pos_b + 1]->seq].dist;
	// B
	double out_b_h{(size_b - pos_b + 1) * (vehicle_b.path[pos_b - 2]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b - 2]->dists[vehicle_b.path[pos_b - 1]->seq].dist - vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist)};
	double out_b_t{(size_b - pos_b - 1) * (vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 2]->seq].dist - vehicle_b.path[pos_b + 1]->dists[vehicle_a.path[pos_a]->seq].dist - vehicle_b.path[pos_b + 1]->dists[vehicle_b.path[pos_b + 2]->seq].dist)};
	for (u32 i{1}; i <= pos_b - 1; i++) {
		tmp = vehicle_b.path[i - 1]->dists[vehicle_b.path[i]->seq].dist;
		out_b_t -= tmp, out_b_h -= tmp;
	}
	// out_b_h -= vehicle_b.path[pos_b - 2]->dists[vehicle_b.path[pos_b - 1]->seq].dist;
	out_b_t -= vehicle_b.path[pos_b - 1]->dists[vehicle_a.path[pos_a]->seq].dist + vehicle_a.path[pos_a]->dists[vehicle_b.path[pos_b + 1]->seq].dist;
	if (out_a_h + out_b_h < out_a_t + out_b_t) {  // 前插
		out_da = dif_a + out_a_h, out_db = dif_b + out_b_h;
		if (out_da + out_db > 0) return false;
		//  交换
		vehicle_a.load += dif_load + vehicle_b.path[pos_b - 1]->demand;
		vehicle_b.load -= dif_load + vehicle_b.path[pos_b - 1]->demand;
		std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);
		vehicle_a.path.emplace(vehicle_a.path.begin() + pos_a, vehicle_b.path[pos_b - 1]);
		vehicle_b.path.erase(vehicle_b.path.begin() + pos_b - 1);
	} else {  // 后插
		out_da = dif_a + out_a_t, out_db = dif_b + out_b_t;
		if (out_da + out_db > 0) return false;
		//  交换
		vehicle_a.load += dif_load + vehicle_b.path[pos_b + 1]->demand;
		vehicle_b.load -= dif_load + vehicle_b.path[pos_b + 1]->demand;
		std::swap(vehicle_a.path[pos_a], vehicle_b.path[pos_b]);
		vehicle_a.path.emplace(vehicle_a.path.begin() + pos_a + 1, vehicle_b.path[pos_b + 1]);
		vehicle_b.path.erase(vehicle_b.path.begin() + pos_b + 1);
	}
	return true;
}

double PER::cumlength(std::vector<Node*>& path) {
	double cumlength{};
	for (u64 j{0}, n{path.size() - 2}; j < n; j++) {
		cumlength += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
	}
	return cumlength;
}

bool PER::insert(Vehicle& vehicle, Node* node, u32 ctrl) {
	if (vehicle.load + node->demand > vehicle.capacity) return false;
	vehicle.load += node->demand;
	vehicle.path.emplace(vehicle.path.end() - 1, node);
	std::vector<Node*> path{vehicle.path};
	double minlength{1000000000.0};
	u32 size = path.size();
	if (path[size - 3]->end >= path[size - 2]->end)
		minlength = PER::cumlength(path);
	for (u32 i = path.size() - 3; i > 0; i--) {
		std::swap(path[i], path[i + 1]);
		if (path[i]->end >= path[i + 1]->end) {
			double tmp = PER::cumlength(path);
			if (tmp < minlength) {
				vehicle.path = path;
				minlength = tmp;
			}
		}
	}
	return true;
}

bool PER::insert(Solution& sol, Node* node, u32 ctrl) {
	u32 best_seq{}, seq{0};
	std::vector<Node*> best_path{};
	double minlength{1000000000.0};
	for (auto& ve : sol.solution) {
		seq++;
		if (ve.load + node->demand > ve.capacity && !(ctrl & FORCE)) continue;
		ve.path.emplace(ve.path.begin() + 1, node);
		u32 size = ve.path.size() - 1;
		double tmp{};
		if (size <= 2) {  // 空的
			tmp = PER::cumlength(ve.path) - ve.cumlength;
			if (tmp < minlength) {
				best_path = ve.path;
				best_seq = seq - 1;
				minlength = tmp;
			}
		} else {  // 非空
			if (ve.path[1]->end >= ve.path[2]->end) {
				tmp = PER::cumlength(ve.path) - ve.cumlength;
				if (tmp < minlength) {
					best_path = ve.path;
					best_seq = seq - 1;
					minlength = tmp;
				}
			}
			for (u32 i{2}; i < size; i++) {
				std::swap(ve.path[i], ve.path[i - 1]);
				if (ve.path[i - 1]->end >= ve.path[i]->end && ve.path[i]->end >= ve.path[i + 1]->end) {
					tmp = PER::cumlength(ve.path) - ve.cumlength;
					if (tmp < minlength) {
						best_path = ve.path;
						best_seq = seq - 1;
						minlength = tmp;
					}
				}
			}
		}
		ve.path.erase(ve.path.end() - 2);
	}
	if (best_path.empty()) return false;
	sol.solution[best_seq].path = best_path;
	sol.solution[best_seq].cumlength += minlength;
	sol.solution[best_seq].load += node->demand;
	return true;
}

void PER::EjecChain(Solution& sol, u32 k, u32 epoch) {
	u32 size_s = sol.solution.size();
	if (k >= size_s) k = size_s;
	if (k == 0 || k == 1) return;
	std::vector<u32> select_s(size_s, 0);
	std::iota(select_s.begin() + 1, select_s.end(), 1);
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::shuffle(select_s.begin(), select_s.end(), gen);
	select_s.resize(k);
	select_s.emplace_back(select_s.front());
	for (u32 i{0}; i < k; i++) {
		u32 range_a = sol.solution[select_s[i]].path.size() - 2;
		u32 range_b = sol.solution[select_s[i + 1]].path.size() - 2;
		int n = epoch;
		while (n--) {
			u32 index_a{gen() % range_a + 1};
			u32 index_b{gen() % range_b + 1};
			int difload = sol.solution[select_s[i]].path[index_a]->demand - sol.solution[select_s[i + 1]].path[index_b]->demand;
			if (sol.solution[select_s[i]].load - difload > sol.solution[select_s[i]].capacity || sol.solution[select_s[i + 1]].load + difload > sol.solution[select_s[i + 1]].capacity)
				continue;
			sol.solution[select_s[i]].load -= difload;
			sol.solution[select_s[i + 1]].load += difload;
			sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
			sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
			std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
			sol.solution[select_s[i + 1]].path_cumlength(1);
			sol.solution[select_s[i]].path_cumlength(1);
			sol.alltardiness = priority(sol);
			// break;
		}
	}
	sol.update();
}

void PER::RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch) {
	// ruin
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	u32 size = maxnode.size();
	std::uniform_int_distribution<> disceate(0, size - 1);
	std::vector<Node*> near;
	// double allength{sol.allength};
	if (sol.solution.size() > sol.maxvehicle) {  // 移除最短
		u32 reducevehicle = sol.solution.size() - sol.maxvehicle;
		std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
		for (u32 i{0}; i < reducevehicle; i++) {
			near.insert(near.end(), sol.solution.back().path.begin() + 1, sol.solution.back().path.end() - 1);
			sol.solution.pop_back();
		}
		sol.update_hash();
	} else {
		u32 index = maxnode[disceate(gen)]->seq;
		u32 rnode = sol.shash[index];
		u32 locate{CHK::find(sol.solution[rnode].path, index, 10)};
		near.reserve(size * k + 1);
		for (u32 i{0}; i < k * size; i++) {
			near.emplace_back(sol.solution[rnode].path[locate]->distsort[i].toNode);
		}
		// 移除node
		for (u32 i{0}, n = near.size(); i < n; i++) {
			index = sol.shash[near[i]->seq];
			locate = CHK::find(sol.solution[index].path, near[i], 11);
			sol.solution[index].load -= sol.solution[index].path[locate]->demand;       // load
			sol.solution[index].path.erase(sol.solution[index].path.begin() + locate);  // path
			                                                                            // sol.shash.erase(n->seq);                                                    // hash
		}
		std::for_each(sol.solution.begin(), sol.solution.end(), [](Vehicle& v) { v.path_cumlength(1); });
		sol.update_hash();
		// std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
	}
	// 插入
	Solution s{sol}, best_sol{sol};
	best_sol.valid = 0;
	bool error{0}, flag{0};
	double flength{1000000000.0};
	while (epoch) {
		std::shuffle(near.begin(), near.end(), gen);  // 打乱
		for (auto& n : near) {
			if (!PER::insert(s, n, 0)) {
				error = 1;
				break;
			}
		}
		epoch--;
		if (error) {  // 失败
			if (epoch > 0) {
				s = sol;
				error = 0;
				continue;
			}
			std::sort(near.begin(), near.end(), [](Node* a, Node* b) { return a->demand > b->demand; });  // 排序
			for (auto& n : near) {
				std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.load > b.load; });
				std::vector<Node*> ve{sol.solution.back().path};
				sol.solution.back().load += n->demand;
				ve.emplace(ve.begin() + 1, n);
				u32 size = ve.size() - 1;
				double tmp{}, best{1000000000.0};
				if (ve[1]->end >= ve[2]->end) {
					tmp = PER::cumlength(ve);
					sol.solution.back().path = ve;
					sol.solution.back().cumlength = tmp;
					best = tmp;
				}
				for (u32 i{2}; i < size; i++) {
					std::swap(ve[i], ve[i - 1]);
					if (ve[i - 1]->end >= ve[i]->end && ve[i]->end >= ve[i + 1]->end) {
						tmp = PER::cumlength(ve);
						if (tmp < best) {
							sol.solution.back().path = ve;
							sol.solution.back().cumlength = tmp;
							best = tmp;
						}
					}
				}
			}
			sol.alltardiness = priority(sol);
			sol.update();
			flag = 0;
			break;
		}
		s.alltardiness = priority(s);
		s.update();
		if (s.allobj < flength) {
			flength = s.allobj;
			flag = 1;
			best_sol = s;
			best_sol.valid = 1;
		}
		s = sol;
		error = 0;
	}
	if (flag)
		sol = best_sol;
	sol.update_seq();
	sol.update_hash(1);
}

void PER::RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch, u32 rule) {
	// ruin
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	u32 size = maxnode.size();
	std::uniform_int_distribution<> disceate(0, size - 1);
	std::vector<Node*> near;
	// double allength{sol.allength};
	if (sol.solution.size() > sol.maxvehicle || rule & 1) {  // 移除最短
		u32 reducevehicle = sol.solution.size() - sol.maxvehicle;
		std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
		for (u32 i{0}; i < reducevehicle; i++) {
			near.insert(near.end(), sol.solution.back().path.begin() + 1, sol.solution.back().path.end() - 1);
			sol.solution.pop_back();
		}
		sol.update_hash();
	} else {
		u32 index = maxnode[disceate(gen)]->seq;
		u32 rnode = sol.shash[index];
		u32 locate{CHK::find(sol.solution[rnode].path, index, 10)};
		near.reserve(size * k + 1);
		for (u32 i{0}; i < k * size; i++) {
			near.emplace_back(sol.solution[rnode].path[locate]->distsort[i].toNode);
		}
		// 移除node
		for (u32 i{0}, n = near.size(); i < n; i++) {
			index = sol.shash[near[i]->seq];
			locate = CHK::find(sol.solution[index].path, near[i], 11);
			sol.solution[index].load -= sol.solution[index].path[locate]->demand;       // load
			sol.solution[index].path.erase(sol.solution[index].path.begin() + locate);  // path
			                                                                            // sol.shash.erase(n->seq);                                                    // hash
		}
		std::for_each(sol.solution.begin(), sol.solution.end(), [](Vehicle& v) { v.path_cumlength(1); });
		sol.update_hash();
		// std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
	}
	Solution s{sol}, best_sol{sol};
	best_sol.valid = 0;
	// 插入
	bool error{0}, flag{0};
	double flength{1000000000.0}, ilength{1000000000.0};
	while (epoch) {
		std::shuffle(near.begin(), near.end(), gen);  // 打乱
		if (rule & 1 << 1) {                          // 随机
			for (auto& n : near) {
				if (!PER::insert(s, n, 0)) {
					error = 1;
					PER::insert(s, n, 1);
				}
			}
		} else {  // 后悔
			      // todo
		}
		epoch--;
		if (error) {  // 失败
			s.alltardiness = priority(s);
			s.update();
			s.valid = false;
			if (best_sol.valid == 0 && s.allobj < ilength) {
				ilength = s.allobj;
				flag = 1;
				best_sol = s;
				best_sol.valid = 0;
			}
			s = sol;
			error = 0;
			continue;
		}
		s.alltardiness = priority(s);
		s.update();
		if (s.allobj < flength) {
			flength = s.allobj;
			flag = 1;
			best_sol = s;
			best_sol.valid = 1;
		}
		s = sol;
		error = 0;
	}
	if (flag)
		sol = best_sol;
	else {
		throw "per failed";
	}
	sol.update_seq();
	sol.update_hash(1);
}

void PER::EjecChain(Solution& sol, u32 k, u32 epoch, u32 rule) {
	u32 size_s = sol.solution.size();
	if (k >= size_s) k = size_s;
	if (k == 0 || k == 1) return;
	std::vector<u32> select_s(size_s, 0);
	std::iota(select_s.begin() + 1, select_s.end(), 1);
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::shuffle(select_s.begin(), select_s.end(), gen);
	select_s.resize(k);
	select_s.emplace_back(select_s.front());
	for (u32 i{0}; i < k; i++) {
		u32 range_a = sol.solution[select_s[i]].path.size() - 2;
		u32 range_b = sol.solution[select_s[i + 1]].path.size() - 2;
		int n = epoch;
		while (n--) {
			u32 index_a{gen() % range_a + 1};
			u32 index_b{gen() % range_b + 1};
			int difload = sol.solution[select_s[i]].path[index_a]->demand - sol.solution[select_s[i + 1]].path[index_b]->demand;
			if (rule & FORCE) {
				sol.solution[select_s[i]].load -= difload;
				sol.solution[select_s[i + 1]].load += difload;
				sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
				sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
				std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
				sol.solution[select_s[i + 1]].path_cumlength(1);
				sol.solution[select_s[i]].path_cumlength(1);
				sol.alltardiness = priority(sol);
				// break;
			}
			//} else if (sol.solution[select_s[i]].load - difload <= sol.solution[select_s[i]].capacity && sol.solution[select_s[i + 1]].load + difload <= sol.solution[select_s[i + 1]].capacity) {
			else {
				if (!sol.solution[select_s[i]].path[index_a - 1]->isdepot && sol.solution[select_s[i]].path[index_a - 1]->end > sol.solution[select_s[i + 1]].path[index_b]->end)
					continue;
				if (!sol.solution[select_s[i]].path[index_a + 1]->isdepot && sol.solution[select_s[i + 1]].path[index_b]->end > sol.solution[select_s[i]].path[index_a + 1]->end)
					continue;
				if (!sol.solution[select_s[i + 1]].path[index_b - 1]->isdepot && sol.solution[select_s[i + 1]].path[index_b - 1]->end > sol.solution[select_s[i]].path[index_a]->end)
					continue;
				if (!sol.solution[select_s[i + 1]].path[index_b + 1]->isdepot && sol.solution[select_s[i]].path[index_a]->end > sol.solution[select_s[i + 1]].path[index_b + 1]->end)
					continue;
				sol.solution[select_s[i]].load -= difload;
				sol.solution[select_s[i + 1]].load += difload;
				sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
				sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
				std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
				sol.solution[select_s[i + 1]].path_cumlength(1);
				sol.solution[select_s[i]].path_cumlength(1);
				sol.alltardiness = priority(sol);
				// break;
			}
		}
	}
	sol.update();
}

bool OPS::onepointmove(Solution& s, Vehicle& r, const u32 a, const u32 b, u32 ctrl) {
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd0{0}, flag{0};
	if (ctrl) cmd0 += FORCE;
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r_front_path{}, r_back_path{}, r_front_limit{}, r_back_limit{}, s_front_tardiness{}, s_back_tardiness{};
	r.precheck(cmd0);
	if (cmd0 & LOADS) return false;
	if (cmd0 & LENGTH)
		flag = 1;
	if (a < b) {  // a在b前面
		ALG::rotate(r.path.begin() + a, r.path.begin() + b, -1);
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					r.cumlength = r_front_path;
					r.length = r_front_limit;
					return true;
				} else {
					ALG::rotate(r.path.begin() + a, r.path.begin() + b, 1);
					return false;
				}
			}
		} else {  // 可行
			s_front_tardiness = priority(s);
			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		std::swap(r.path[b], r.path[b - 1]);
		if (r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			s_back_tardiness = priority(s);
			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			// 插入前面更优
			// location = false;
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明前插入不可行，需要还原
				ALG::rotate(r.path.begin() + a, r.path.begin() + b + 1, 1);
				return false;  // 返回不可行
			}
			// 可行，撤销交换
			std::swap(r.path[b], r.path[b - 1]);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
			s.alltardiness = s_front_tardiness;
			return true;
		} else {
			// 插入后面更优
			// location = true;
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明后插入不可行，需要还原
				ALG::rotate(r.path.begin() + a, r.path.begin() + b + 1, 1);
				return false;  // 返回不可行
			}
			// 可行
			r.cumlength = r_back_path;
			r.length = r_back_limit;
			s.alltardiness = s_back_tardiness;
			return true;
		}
	} else {  // a在b后面
		ALG::rotate(r.path.begin() + b, r.path.begin() + a + 1, 1);
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					r.cumlength = r_front_path;
					r.length = r_front_limit;
					return true;
				} else {
					ALG::rotate(r.path.begin() + b, r.path.begin() + a + 1, -1);
					return false;
				}
			}
		} else {  // 可行
			s_front_tardiness = priority(s);
			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		std::swap(r.path[b], r.path[b + 1]);
		if (r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			s_back_tardiness = priority(s);
			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			// 插入前面更优
			// location = false;
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明前插入不可行，需要还原
				ALG::rotate(r.path.begin() + b + 1, r.path.begin() + a + 1, -1);
				return false;  // 返回不可行
			}
			// 可行，撤销交换
			std::swap(r.path[b], r.path[b + 1]);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
			s.alltardiness = s_front_tardiness;
			return true;
		} else {
			// 插入后面更优
			// location = true;
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明后插入不可行，需要还原
				ALG::rotate(r.path.begin() + b + 1, r.path.begin() + a + 1, -1);
				return false;  // 返回不可行
			}
			// 可行
			r.cumlength = r_back_path;
			r.length = r_back_limit;
			s.alltardiness = s_back_tardiness;
			return true;
		}
	}
}

bool OPS::onepointmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	Node* temp{r1.path[a]};  // 保存要插入的节点
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_front_path{}, r2_back_path{}, r2_front_limit{}, r2_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	// bool location{};         // 保存插入的位置
	// 不在同路径
	r1.precheck(cmd1), r2.precheck(cmd2);
	if (cmd1 & LOADS || cmd2 & LOADS)
		flag = 3;
	else if (cmd1 & LENGTH || cmd2 & LENGTH)
		flag = 1;
	r1.path.erase(r1.path.begin() + a);          // 从r1中删除要插入的节点
	r2.path.emplace(r2.path.begin() + b, temp);  // 在r2中插入节点
	if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_front_path, r2_front_limit, cmd2)) {                 // 不可行
		if (flag == 3) {                                                                                                    // load
			if (r2.load - r1.load + temp->demand < 0) {
				r2.cumlength = r2_front_path;
				r2.length = r2_front_limit;
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
				r1.load -= temp->demand;
				r2.load += temp->demand;
				s.alltardiness = priority(s);
				return true;
			} else {
				r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
				r2.path.erase(r2.path.begin() + b);          // 还原r2路径
				return false;                                // 返回不可行
			}
		} else if (flag == 1) {  // 路径长度限制
			if (r2_front_limit < r2.Limit) {
				saving0 = r2_front_limit - r2.length;
			}
		}
	} else {  // 可行
		s_front_tardiness = priority(s);
		saving0 = r1_newpath + r2_front_path - r1.cumlength - r2.cumlength;
		saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
	}
	std::swap(r2.path[b], r2.path[b + 1]);  // 交换r2中插入的节点和后面的节点
	if (!r2.evaluate(r2_back_path, r2_back_limit, cmd2)) {
		if (flag == 1) {
			if (r2_back_limit < r2.Limit) {
				saving1 = r2_back_limit - r2.length;
			}
		}
	} else {
		s_back_tardiness = priority(s);
		saving1 = r1_newpath + r2_back_path - r1.cumlength - r2.cumlength;
		saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
	}
	if (saving0 < saving1) {
		// 插入前面更优
		// location = false;
		saving = static_cast<i64>(saving0 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {            // 如果节省距离大于0，说明前插入不可行，需要还原
			r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
			r2.path.erase(r2.path.begin() + b + 1);      // 还原r2路径
			return false;                                // 返回不可行
		}
		// 可行，撤销交换
		std::swap(r2.path[b], r2.path[b + 1]);
		r2.cumlength = r2_front_path;
		r2.length = r2_front_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= temp->demand;
		r2.load += temp->demand;
		if (s_front_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_front_tardiness;
		return true;
	} else {
		// 插入后面更优
		// location = true;
		saving = static_cast<i64>(saving1 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {            // 如果节省距离大于0，说明后插入不可行，需要还原
			r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
			r2.path.erase(r2.path.begin() + b + 1);      // 还原r2路径
			return false;                                // 返回不可行
		}
		// 可行
		r2.cumlength = r2_back_path;
		r2.length = r2_back_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= temp->demand;
		r2.load += temp->demand;
		if (s_back_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_back_tardiness;
		return true;
	}
}

bool OPS::reverse(Solution& s, Vehicle& r, const u32 f, const u32 t, u32 ctrl) {
	i64 saving{};
	double r_path{}, r_path_limit{}, s_tardiness{-1.0};
	// bool improved{false};
	u32 cmd0{}, flag{};
	if (ctrl) cmd0 += FORCE;
	r.precheck(cmd0);
	if (cmd0 & LOADS)
		return false;
	else if (cmd0 & LENGTH)
		flag = 1;
	// 倒转路径中从f到t之间的元素
	std::reverse(r.path.begin() + f, r.path.begin() + t);
	// 如果倒转后的路径评估成功
	if (!r.evaluate(r_path, r_path_limit, cmd0)) {  // 不可行
		if (flag == 1) {                            // 长度
			// if (r_path_limit < r.length) {
			saving = static_cast<i64>((r_path_limit - r.length) * 1000);
			// improved = true;
			//}
		}
	} else {  // 可行
		s_tardiness = priority(s);
		saving = static_cast<i64>(v_aim(r_path - r.cumlength, s_tardiness - s.alltardiness) * 1000);
		// improved = true;
	}
	if (cmd0 & FORCE || saving < 0) {
		r.cumlength = r_path;
		r.length = r_path_limit;
		if (s_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_tardiness;
		return true;
	}
	std::reverse(r.path.begin() + f, r.path.begin() + t);
	return false;
}

bool OPS::swapmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};                    // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	int difload = r1.path[a]->demand - r2.path[b]->demand;
	if (r1.seq == r2.seq) {  // 同车
		r1.precheck(cmd1);
		if (cmd1 & LOADS)
			return false;
		else if (cmd1 & LENGTH)
			flag = 1;
		std::swap(r1.path[a], r1.path[b]);  // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
			if (flag == 1) {                                // 长度
				// if (r1_newlimit < r1.length) {
				saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				// improved = true;
				//}
			}
		} else {  // 可行
			s_tardiness = priority(s);
			saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			// improved = true;
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
			return true;
		}
		std::swap(r1.path[a], r1.path[b]);     // 恢复原始路径
		return false;                          // 返回交换失败
	} else {                                   // 不同车
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			flag = 3;
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			// if (difload > 0) {
			//	if (r2.load - r1.load + difload >= 0) return false;
			// } else {
			//	if (r2.load - r1.load + difload <= 0) return false;
			// }
			if (difload > 0) {
				if (r2.load + difload > r2.capacity) return false;
			} else {
				if (r1.load - difload > r1.capacity) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);
			r1.update_allength();
			r2.update_allength();
			s.alltardiness = priority(s);
			return true;
		}
		std::swap(r1.path[a], r2.path[b]);     // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
			s_tardiness = priority(s);
			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);  // 恢复原始路径
		return false;                       // 返回交换失败
	}
}
bool OPS::oropt(Solution& s, Vehicle& r, const u32 f, const u32 t, const u32 len, u32 ctrl) {
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd0{0}, flag{0};
	if (ctrl) cmd0 += FORCE;
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r_front_path{}, r_back_path{}, r_front_limit{}, r_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	r.precheck(cmd0);
	if (cmd0 & LOADS)
		return false;
	else if (cmd0 & LENGTH)
		flag = 1;
	// bool location{};
	if (f < t) {                                                         // f在t前面
		if (t - f == len) {                                              // 相邻
			ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, 1);  // 将路径中f到f1的部分旋转
			// r_back = r.path_cumlength();
			//  如果路径评估成功，则计算节约值
			if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 不可行
				if (flag == 1) {                                 // 长度
					if (r_back_limit < r.length) {
						saving1 = r_back_limit - r.length;
					}
				}
			} else {  // 可行
				s_back_tardiness = priority(s);
				saving1 = r_back_path - r.cumlength;
				saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
			}
			// 如果节约值大于0，则表示路径优化失败
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {
				// location = false;
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, -1);  //  还原
				return false;
			}
			// location = 1;
			// saving = saving1;
			r.cumlength = r_back_path;
			r.length = r_back_limit;
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
			return true;
		}
		// 向左旋转r的路径
		ALG::rotate(r.path.begin() + f, r.path.begin() + t, -len);
		// r_front = r.path_cumlength();
		//  前插
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					saving0 = r_front_limit - r.length;
				}
			}
		} else {  // 可行
			s_front_tardiness = priority(s);
			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		ALG::rotate(r.path.begin() + t - len, r.path.begin() + t + 1, 1);
		// r_back = r.path_cumlength();
		//  后插
		if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			if (flag == 1) {                                 // 长度
				if (r_back_limit < r.length) {
					saving1 = r_back_limit - r.length;
				}
			}
		} else {  // 可行
			s_back_tardiness = priority(s);
			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			saving = static_cast<int>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 前插不可行
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, len);
				return false;
			}
			ALG::rotate(r.path.begin() + t - len, r.path.begin() + t + 1, -1);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
			return true;
		} else {
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 后插不可行
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, len);
				return false;
			}
			r.cumlength = r_back_path;
			r.length = r_back_limit;
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
			return true;
		}
	} else {               // f在t后面
		if (f - t == 1) {  // 相邻
			ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, -1);
			// r_front = r.path_cumlength();
			if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
				if (flag == 1) {                                   // 长度
					if (r_front_limit < r.length) {
						saving0 = r_front_limit - r.length;
					}
				}
			} else {  // 可行
				s_front_tardiness = priority(s);
				saving0 = r_front_path - r.cumlength;
				saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
			}
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {
				// location = false;
				ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, 1);  //  还原
				return false;
			}
			// location = 1;
			// saving = saving1;
			r.cumlength = r_front_path;
			r.length = r_front_limit;
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
			return true;
		}
		// 向右旋转r的路径
		ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, len);
		// r_front = r.path_cumlength();
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					saving0 = r_front_limit - r.length;
				}
			}
		} else {  // 可行
			s_front_tardiness = priority(s);
			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		ALG::rotate(r.path.begin() + t, r.path.begin() + t + len + 1, 1);
		// r_back = r.path_cumlength();
		if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			if (flag == 1) {                                 // 长度
				if (r_back_limit < r.length) {
					saving1 = r_back_limit - r.length;
				}
			}
		} else {  // 可行
			s_back_tardiness = priority(s);
			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			saving = static_cast<int>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 前插不可行
				ALG::rotate(r.path.begin() + t + 1, r.path.begin() + f + len, -len);
				return false;
			}
			ALG::rotate(r.path.begin() + t, r.path.begin() + t + len + 1, -1);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
			return true;
		} else {
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 后插不可行
				ALG::rotate(r.path.begin() + t + 1, r.path.begin() + f + len, -len);
				return false;
			}
			r.cumlength = r_back_path;
			r.length = r_back_limit;
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
			return true;
		}
	}
}

bool OPS::oropt(Solution& s, Vehicle& r1, Vehicle& r2, const u32 f, const u32 t, const u32 len, u32 ctrl) {
	const u32 f1{f + len}, t1{t + len};
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd1{0}, cmd2{0}, flag{0}, difload{};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_front_path{}, r2_back_path{}, r2_front_limit{}, r2_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	// bool location{};
	//  从r1中提取要移动的节点
	std::vector<Node*> temp(r1.path.begin() + f, r1.path.begin() + f1);
	r1.precheck(cmd1), r2.precheck(cmd2);
	if (cmd1 & LOADS || cmd2 & LOADS)
		flag = 3;
	else if (cmd1 & LENGTH || cmd2 & LENGTH)
		flag = 1;
	if (len == 2) {
		difload += temp[0]->demand + temp[1]->demand;
	} else if (len == 3) {
		difload += temp[0]->demand + temp[1]->demand + temp[2]->demand;
	} else if (len == 4) {
		difload += temp[0]->demand + temp[1]->demand + temp[2]->demand + temp[3]->demand;
	}
	// 从r1中移除要移动的节点
	r1.path.erase(r1.path.begin() + f, r1.path.begin() + f1);
	// 将节点插入到r2的指定位置
	r2.path.insert(r2.path.begin() + t, temp.begin(), temp.end());
	// 计算r1的路径长度
	if (flag == 3) {  // load
		if (r2.load - r1.load + difload < 0) {
			r1.update_allength();
			r2.update_allength();
			r1.load -= difload;
			r2.load += difload;
			s.alltardiness = priority(s);
			return true;
		} else {
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t, r2.path.begin() + t1);
			return false;  // 返回不可行
		}
	}
	if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_front_path, r2_front_limit, cmd2)) {  // 不可行
		if (flag == 1) {                                                                                     // 路径长度限制
			if (r2_front_limit < r2.length) {
				saving0 = r2_front_limit - r2.length;
			}
		}
	} else {  // 可行
		s_front_tardiness = priority(s);
		saving0 = r1_newpath + r2_front_path - r1.cumlength - r2.cumlength;
		saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
	}
	// 向右旋转r2的路径
	ALG::rotate(r2.path.begin() + t, r2.path.begin() + t + len + 1, 1);
	// 计算r2的路径长度后
	// r2_back = r2.path_cumlength();
	// 评估将节点插入r2的后面的情况
	if (!r2.evaluate(r2_back_path, r2_back_limit, cmd2)) {
		if (flag == 1) {  // 长度限制
			if (r2_back_limit < r2.length) {
				saving1 = r2_back_limit - r2.length;
			}
		}
	} else {  // 可行正常
		s_back_tardiness = priority(s);
		saving1 = r1_newpath + r2_back_path - r1.cumlength - r2.cumlength;
		saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
	}
	if (saving0 < saving1) {
		// 插入前面更优
		// location = false;
		saving = static_cast<i64>(saving0 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {  // 如果节省距离大于0，说明前插入不可行，需要还原
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t + 1, r2.path.begin() + t1 + 1);
			return false;  // 返回不可行
		}
		// 可行，撤销交换
		ALG::rotate(r2.path.begin() + t, r2.path.begin() + t + len + 1, -1);
		r2.cumlength = r2_front_path;
		r2.length = r2_front_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= difload;
		r2.load += difload;
		if (s_front_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_front_tardiness;
		return true;
	} else {
		// 插入后面更优
		// location = true;
		saving = static_cast<i64>(saving1 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {  // 如果节省距离大于0，说明后插入不可行，需要还原
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t + 1, r2.path.begin() + t1 + 1);
			return false;  // 返回不可行
		}
		// 可行
		r2.cumlength = r2_back_path;
		r2.length = r2_back_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= difload;
		r2.load += difload;
		if (s_back_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_back_tardiness;
		return true;
	}
}

/// @brief 判断两个车辆的路径是否可以交换其中的三个节点，计算路径长度变化
/// @param r1
/// @param r2
/// @param a
/// @param b
/// @param saving
/// @return
bool OPS::arcnode(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;               // 是否强制
	i64 saving{};                                 // 保存交换节点后的路径长度
	Node* node{r1.path[a + 1]};                   // 保存需要交换的节点
	int difload = r1.path[a + 1]->demand + r1.path[a]->demand - r2.path[b]->demand;  // 保存交换节点后的负载变化
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	if (r1.seq == r2.seq) {                       // 如果两个车辆的路径相同
		r1.precheck(cmd1);
		if (cmd1 & LENGTH)
			flag = 1;
		if (a < b) {                              // 如果a在b前面
			std::swap(r1.path[a], r1.path[b]);    // 交换节点
			for (auto it{a + 1}; it < b; it++) {  // 调整路径
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
				if (flag == 1) {                                // 长度
					saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				}
			} else {  // 可行
				s_tardiness = priority(s);
				saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			}
			if (cmd1 & FORCE || saving < 0) {
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
				if (s_tardiness < 0.0)
					s.alltardiness = priority(s);
				else
					s.alltardiness = s_tardiness;
				return true;
			}
			std::swap(r1.path[a], r1.path[b]);  // 恢复节点交换前的路径
			for (auto it{b - 1}; it > a; it--) {
				std::swap(r1.path[it], r1.path[it - 1]);
			}
			return false;                       // 返回交换失败
		} else {                                // 如果a在b后面
			std::swap(r1.path[a], r1.path[b]);  // 交换节点
			for (auto it{a}; it > b; it--) {    // 调整路径
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
				if (flag == 1) {                                // 长度
					saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				}
			} else {  // 可行
				s_tardiness = priority(s);
				saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			}
			if (cmd1 & FORCE || saving < 0) {
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
				if (s_tardiness < 0.0)
					s.alltardiness = priority(s);
				else
					s.alltardiness = s_tardiness;
				return true;
			}
			std::swap(r1.path[a + 1], r1.path[b + 1]);  // 恢复节点交换前的路径
			for (auto it{b}; it < a; it++) {
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			return false;  // 返回交换失败
		}
	} else {                                     // 如果两个车辆的路径不同
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			return false;
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			if (difload > 0) {
				if (r2.load - r1.load + difload >= 0) return false;
			} else {
				if (r2.load - r1.load + difload <= 0) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);               // 交换节点
			r1.path.erase(r1.path.begin() + a + 1);          // 调整路径
			r2.path.emplace(r2.path.begin() + b + 1, node);  // 调整路径
			r1.update_allength();
			r2.update_allength();
			s.alltardiness = priority(s);
			return true;
		}
		std::swap(r1.path[a], r2.path[b]);               // 交换节点
		r1.path.erase(r1.path.begin() + a + 1);          // 调整路径
		r2.path.emplace(r2.path.begin() + b + 1, node);  // 调整路径
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
			s_tardiness = priority(s);
			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);  // 恢复节点交换前的路径
		r1.path.emplace(r1.path.begin() + a + 1, node);
		r2.path.erase(r2.path.begin() + b + 1);
		return false;  // 返回交换失败
	}
}

bool OPS::arcswap(Solution& s, Vehicle& r1, Vehicle& r2, float c, const u32 a, const u32 b, u32 ctrl) {
	i64 saving{};
	int difload = r1.path[a]->demand + r1.path[a + 1]->demand - r2.path[b]->demand - r2.path[b + 1]->demand;
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	if (r1.seq == r2.seq) {                         // 同车
		r1.precheck(cmd1);
		if (cmd1 & LOADS)
			return false;
		else if (cmd1 & LENGTH)
			flag = 1;
		std::swap(r1.path[a], r1.path[b]);          // 交换路径中的两个位置
		std::swap(r1.path[a + 1], r1.path[b + 1]);  // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
			if (flag == 1) {                                // 长度
				saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
			}
		} else {  // 可行
			s_tardiness = priority(s);
			saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
			return true;
		}
		std::swap(r1.path[a], r1.path[b]);          // 恢复原始路径
		std::swap(r1.path[a + 1], r1.path[b + 1]);  // 恢复原始路径
		return false;                               // 返回交换失败
	} else {                                        // 不同车
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			flag = 3;
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			if (difload > 0) {
				if (r2.load - r1.load + difload >= 0) return false;
			} else {
				if (r2.load - r1.load + difload <= 0) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);          // 交换路径中的两个位置
			std::swap(r1.path[a + 1], r2.path[b + 1]);  // 交换路径中的两个位置
			r1.update_allength();
			r2.update_allength();
			s.alltardiness = priority(s);
			return true;
		}
		std::swap(r1.path[a], r2.path[b]);                             // 交换路径中的两个位置
		std::swap(r1.path[a + 1], r2.path[b + 1]);                     // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
			s_tardiness = priority(s);
			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);          // 恢复原始路径
		std::swap(r1.path[a + 1], r2.path[b + 1]);  // 恢复原始路径
		return false;                               // 返回交换失败
	}
}