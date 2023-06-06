#include "NSearch.hpp"

#include <algorithm>
#include <random>

#include "operator.hpp"

void ALNS::repair(Solution& solution, std::vector<std::pair<uint32_t, const Node*>>& rest) {
	std::sort(rest.begin(), rest.end(), [](std::pair<uint32_t, const Node*>& a, std::pair<uint32_t, const Node*>& b) { return a.first < b.first; });
	for (auto& r : rest) {
		uint32_t f{}, l{};                                                                // f 为在哪条路径， l最近的同厂站邻域
		for (uint32_t i = 0; i < r.second->distsort.size(); i++) {                        // 查找所在路径
			if (solution.shash.find(r.second->distsort[i].to) != solution.shash.end()) {  // 邻域是否也被破坏
				if (solution.shash[r.second->distsort[i].to] == r.first) {                // 邻域是否在同厂站
					l = i;
					f = solution.shash[r.second->distsort[i].to];
					break;
				}
			}
		}
		for (uint32_t i = 1; i < solution.solution[f].path.size() - 1; i++) {     // 查找所在路径中的位置
			if (solution.solution[f].path[i]->seq == r.second->distsort[l].to) {  // 找到开始插入
				double db = COST::insertb(solution.solution[f], r.second, i);
				double df = COST::insertf(solution.solution[f], r.second, i);
				if (db < df) {
					OP::insertb(solution.solution[f], r.second, i, db);
				} else {
					OP::insertf(solution.solution[f], r.second, i, df);
				}
				solution.shash.emplace(std::make_pair(r.second->seq, f));  // 更新查找表
				break;
			}
		}
	}
	rest.clear();
}

void ALNS::destory_wst(Solution& solution, const float p, std::vector<std::pair<uint32_t, const Node*>>& rest) {
	std::vector<std::pair<uint32_t, double>> cost;
	for (auto& r : solution.solution) {
		uint32_t num = std::ceil((r.path.size() - 2) * p);
		for (uint32_t j = 1; j < r.path.size() - 1; j++) {
			double length = r.path[j]->dists[r.path[j + 1]->seq].dist + r.path[j - 1]->dists[r.path[j]->seq].dist - r.path[j - 1]->dists[r.path[j + 1]->seq].dist;
			cost.emplace_back(std::make_pair(j, length));
		}
		if ((2 ^ num) > r.path.size()) {  // n^2 ? nlogn
			std::sort(cost.begin(), cost.end(), [](const std::pair<uint32_t, double>& a, const std::pair<uint32_t, double>& b) { return a.second < b.second; });
			while (num) {
				double dif = COST::remove(r, cost.back().first);
				const Node* node = OP::remove(r, cost.back().first, dif);
				rest.emplace_back(std::make_pair(r.path.back()->seq, node));
				solution.shash.erase(rest.back().second->seq);
				cost.pop_back();
				num--;
			}
		} else {
			while (num) {
				int mx = 0;
				for (uint32_t i = 0; i < cost.size(); i++) {
					mx = cost[mx].second > cost[i].second ? mx : i;
				}
				double dif = COST::remove(r, cost[mx].first);
				const Node* node = OP::remove(r, cost[mx].first, dif);
				rest.emplace_back(std::make_pair(r.path.back()->seq, node));
				solution.shash.erase(rest.back().second->seq);
				std::swap(cost[mx], cost.back());
				cost.pop_back();
				num--;
			}
		}

		cost.clear();
	}
}

void ALNS::destory_rnd(Solution& solution, const float p, std::vector<std::pair<uint32_t, const Node*>>& rest) {
	std::random_device device_seed;
	std::mt19937 gen(device_seed());
	for (auto& r : solution.solution) {
		uint32_t num = std::ceil((r.path.size() - 2) * p);
		while (num) {
			std::uniform_int_distribution<> dis(1, r.path.size() - 2);
			uint32_t rd = dis(gen);
			double dif = COST::remove(r, rd);
			const Node* node = OP::remove(r, rd, dif);
			rest.emplace_back(std::make_pair(r.path.back()->seq, node));
			solution.shash.erase(rest.back().second->seq);
			num--;
		}
	}
}