#include "solution.hpp"

#include <algorithm>
#include <set>

Solution cw(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}

Solution greedynear(const std::vector<Node*>& nodes, const std::vector<Node*>& station) {
	Solution solution;
	constexpr int MaxLoad = 200;
	std::vector<std::vector<const Node*>> classfy(station.size());
	std::set<uint32_t> tabu;  // 排除厂站
	for (uint32_t i = 0; i < station.size(); i++) {
		tabu.emplace(station[i]->seq);
	}
	for (auto& node : nodes) {                             // 将每个客户分给距离最近的厂站；
		if (tabu.find(node->seq) != tabu.end()) continue;  // 排除厂站
		for (uint32_t i = 1; i < node->distsort.size(); i++) {
			if (tabu.find(node->distsort[i].to) != tabu.end()) {
				classfy[node->distsort[i].to].emplace_back(node);
				break;
			}
		}
	}
	for (uint32_t i = 0; i < station.size(); i++) {
		Vehicle vehicle(station[i], MaxLoad);
		std::sort(classfy[i].begin(), classfy[i].end(), [i](const Node* a, const Node* b) { return a->dists[i].dist > b->dists[i].dist; });
		std::set<uint32_t> walked, service;  // 每个厂站已服务的节点, 和需要服务的节点
		for (auto& node : classfy[i]) {      // 初始化服务节点
			service.emplace(node->seq);
		}
		while (true) {
			const Node* near{classfy[i].back()};
			if (walked.find(near->seq) == walked.end()) {           // 没有服务过
				while (true) {
					if (!vehicle.move(near)) {                      // 达到最大
						vehicle.path.emplace_back(station[i]);      // 返回厂站
						vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
						solution.add(vehicle);                      // 加入到答案。
						vehicle.clear(station[i]);                  // 清空
						break;
					} else {                                        // 没有达到
						walked.insert(near->seq);
						if (service.size() == walked.size()) {
							vehicle.path.emplace_back(station[i]);      // 返回厂站
							vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
							solution.add(vehicle);                      // 加入到答案。
							vehicle.clear(station[i]);                  // 清空
							break;
						}
						for (auto& n : near->distsort) {
						if (service.find(nodes[n.to]->seq) != service.end() && walked.find(nodes[n.to]->seq) == walked.end()) {  // 在服务列表 且没有服务过
							near = nodes[n.to];                                                                                  // 找到下一个位置
							break;
						}
						}
					}
				}
				classfy[i].pop_back();      // 服务过
			} else {
				classfy[i].pop_back();      // 服务过
			}
			if (classfy[i].empty()) break;  // 服务完了
		}
	}
	for (uint32_t i = 0; i < solution.solution.size(); i++) {
		for (auto& j : solution.solution[i].path) {
			solution.shash.emplace(std::make_pair(j->seq, i));  // 建立hash 查找表
		}
	}
	return solution;  // 返回答案。
}
Solution nassign(const std::vector<Node*>& nodes, const std::vector<Node*>& station) {
	Solution solution;
	constexpr int MaxLoad = 200;
	std::vector<std::vector<const Node*>> classfy(station.size());
	std::set<uint32_t> tabu;  // 排除厂站
	for (uint32_t i = 0; i < station.size(); i++) {
		tabu.emplace(station[i]->seq);
	}
	for (auto& node : nodes) {                             // 将每个客户分给距离最近的厂站；
		if (tabu.find(node->seq) != tabu.end()) continue;  // 排除厂站
		for (uint32_t i = 1; i < node->distsort.size(); i++) {
			if (tabu.find(node->distsort[i].to) != tabu.end()) {
				classfy[node->distsort[i].to].emplace_back(node);
				break;
			}
		}
	}
	for (uint32_t i = 0; i < station.size(); i++) {
		Vehicle vehicle(station[i], MaxLoad);
		std::sort(classfy[i].begin(), classfy[i].end(), [i](const Node* a, const Node* b) { return a->dists[i].dist < b->dists[i].dist; });
		for (auto& node : classfy[i]) {
			if (!vehicle.move(node)) {                      // 达到最大
				vehicle.path.emplace_back(station[i]);      // 返回厂站
				vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
				solution.add(vehicle);                      // 加入到答案。
				vehicle.clear(station[i]);                  // 清空
			}
		}
		if (!vehicle.path.empty()) {                    // 最后的客户
			vehicle.path.emplace_back(station[i]);      // 返回厂站
			vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
			solution.add(vehicle);                      // 加入到答案。
			vehicle.clear(station[i]);                  // 清空
		}
	}
	for (uint32_t i = 0; i < solution.solution.size(); i++) {
		for (auto& j : solution.solution[i].path) {
			solution.shash.emplace(std::make_pair(j->seq, i));  // 建立hash 查找表
		}
	}
	return solution;
}

Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}