#include "NSearch.hpp"

#include <algorithm>
#include <random>

#include "algorithm.hpp"
#include "operator.hpp"

void ALNS::repair(Solution& solution, std::vector<std::pair<u32, Node*>>& rest) {
	std::sort(rest.begin(), rest.end(), [](const std::pair<u32, Node*>& a, const std::pair<u32, Node*>& b) { return a.first < b.first; });
	for (auto& r : rest) {
		for (auto near = r.second->distsort.begin() + 1; near != r.second->distsort.end(); near++) {                                                               // 查找所在路径
			if (solution.shash.find((*near).to) != solution.shash.end()) {                                                                                         // 邻域是否也被破坏
				if (solution.solution[solution.shash[(*near).to]].path.front()->seq == r.first) {                                                                  // 邻域是否在同厂站
					if (solution.solution[solution.shash[(*near).to]].load + r.second->demand > solution.solution[solution.shash[(*near).to]].capacity) continue;  // 容量已满
					for (u32 i = 1; i < solution.solution[solution.shash[(*near).to]].path.size() - 1; i++) {                                                      // 查找所在路径中的位置
						if (solution.solution[solution.shash[(*near).to]].path[i]->seq == (*near).to) {                                                            // 找到开始插入
							double db = COST::insertb(solution.solution[solution.shash[(*near).to]].path, r.second, i);
							double df = COST::insertf(solution.solution[solution.shash[(*near).to]].path, r.second, i);
							if (db < df) {
								OP::insertb(solution.solution[solution.shash[(*near).to]], r.second, i, 0.0);
							} else {
								OP::insertf(solution.solution[solution.shash[(*near).to]], r.second, i, 0.0);
							}
							solution.shash.emplace(std::make_pair(r.second->seq, solution.shash[(*near).to]));  // 更新查找表
							break;
						}
					}
					break;  // 跳出二层循环
				}
			}
		}
	}
	rest.clear();
}

void ALNS::destory_wst(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest) {
	std::vector<std::pair<u32, double>> cost;  // （路线节点，长度差值）
	for (auto& r : solution.solution) {
		if (r.path.size() == 2) continue;
		u32 num = std::ceil((r.path.size() - 2) * p);  // 每条路线删除的个数
		for (u32 j = 1; j < r.path.size() - 1; j++) {
			/* 0-a-b-c-d-e-0
			 * a：0a + ab - 0b
			 * b: ab + bc - ac
			 * ...
			 */
			double length = r.path[j]->dists[r.path[j + 1]->seq].dist + r.path[j - 1]->dists[r.path[j]->seq].dist - r.path[j - 1]->dists[r.path[j + 1]->seq].dist;
			cost.emplace_back(std::make_pair(j, length));
		}
		if (num > 2) {  // n ? 2
			// std::sort(cost.begin(), cost.end(), [](const std::pair<u32, double>& a, const std::pair<u32, double>& b) { return a.second < b.second; });
			ALG::topK(cost, 0, cost.size() - 1, num, [](const std::pair<u32, double>& a, const std::pair<u32, double>& b) { return a.second < b.second; });
			while (num) {
				// double dif = COST::erase(r.path, cost.back().first);
				rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, cost.back().first, 0.0)));
				solution.shash.erase(rest.back().second->seq);  // 更新查找表
				cost.pop_back();
				num--;
			}
			OP::erase(r);
		} else {
			while (num) {
				int mx = 0;
				for (u32 i = 0; i < cost.size(); i++) {
					mx = cost[mx].second > cost[i].second ? mx : i;
				}
				// double dif = COST::erase(r.path, cost[mx].first);
				rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, cost[mx].first, 0.0)));
				solution.shash.erase(rest.back().second->seq);  // 更新查找表
				std::swap(cost[mx], cost.back());
				cost.pop_back();
				num--;
			}
			OP::erase(r);
		}
		cost.clear();
	}
}

void ALNS::destory_rnd(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest) {
	std::random_device device_seed;
	std::mt19937 gen(device_seed());
	for (auto& r : solution.solution) {
		if (r.path.size() == 2) continue;
		u32 num = std::ceil((r.path.size() - 2) * p);  // 每条路线删除的个数
		std::uniform_int_distribution<> dis(1, r.path.size() - 2);
		for (u32 i = 0; i < num; i++) {
			u32 rd = dis(gen) - i;  // 随机选取
			double dif = COST::erase(r.path, rd);
			rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, rd, dif)));
			solution.shash.erase(rest.back().second->seq);  // 更新查找表
		}
	}
}

void LNS::run(Solution& solution, const std::vector<Node*>& nodes, u32 epoch) {
}

void LS::two(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{i + 1}; j < n; j++) {
			if (solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 2)
				OP::twoSwap(solution.solution[i], solution.solution[j], flag);
		}
	}
}

void LS::one(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{0}; j < n; j++) {
			if (i != j && solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 1) {
				OP::oneMove(solution.solution[i], solution.solution[j], flag);
			}
		}
	}
}

void LS::three(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{0}; j < n; j++) {
			if (i != j && solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 3) {
				OP::threeSwap(solution.solution[i], solution.solution[j], flag);
			}
		}
	}
}

void LS::twoOpt(Solution& solution, bool& flag) {
	double dif{};  // 距离差值
	for (u32 s{0}, size = solution.solution.size(); s < size; s++) {
		if (solution.solution[s].path.size() <= 4) continue;
		for (u32 i{1}, n = solution.solution[s].path.size() - 3; i < n; i++) {
			for (u32 j{i + 2}, m = n + 2; j < m; j++) {
				if (CHK::reverse(solution.solution[s], i, j, dif)) {
					solution.solution[s].cumlength += dif;  // 更新距离
					flag = true;
				}
			}
		}
	}
}

void VNS::relocate(Solution& solution, bool& flag) {
	Node* near{};
	u32 to{};
	for (auto r : solution.solution) {
		if (r.path.size() <= 2) continue;
		for (u32 c{1}, n = r.path.size() - 1; c < n; c++) {
			for (u32 i{0}, m = r.path[c]->distsort.size() * 0.2; i < m; i++) {
				near = r.path[c]->depotsort[i].toNode;
				to = CHK::find(solution.solution[solution.shash[near->seq]].path, near);
				if (to != r.seq) {
				}
			}
		}
	}
}