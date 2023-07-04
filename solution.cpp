#include "solution.hpp"

#include <algorithm>
#include <set>

#include "algorithm.hpp"

Solution cw(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}

/*
Solution greedynear(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const int maxload) {
	Solution solution;
	// constexpr int MaxLoad = 200;
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
		Vehicle vehicle(station[i], maxload);
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
		for (uint32_t j = 1; j < solution.solution[i].path.size() - 1; j++) {              // 排除厂站
			solution.shash.emplace(std::make_pair(solution.solution[i].path[j]->seq, i));  // 建立hash 查找表
		}
	}
	return solution;  // 返回答案。
}*/

Solution greedynear(std::vector<const Node*>& nodes, const uint32_t depot_num, const uint32_t maxload) {
	Solution solution;
	std::vector<const Node*> depots, custers;
	uint32_t num{0};  // 路线序号
	std::vector<std::vector<const Node*>> classfy(depot_num);
	custers.assign(nodes.begin(), nodes.end() - depot_num);
	depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	std::vector<Node> K;
	K.reserve(depot_num);
	for (auto& i : depots) {
		K.emplace_back(*i);  // K
	}
	ALG::Kmean(nodes, K, 1000, classfy);
	for (uint32_t i = 0; i < depot_num; i++) {  // 将分好类的客户安排给最近厂站
		double min_dist = 1000000.0;
		unsigned int min_index = 10000;
		for (unsigned int j = 0; j < depot_num; ++j) {
			double d = dist(K[i], *depots[j]);
			if (d < min_dist) {
				min_dist = d;
				min_index = j;
			}
		}
		Vehicle vehicle(depots[min_index], maxload, num++);  // 初始路线
		// 按离厂站距离排序
		std::sort(classfy[i].begin(), classfy[i].end(), [&depots, min_index](const Node* a, const Node* b) { return a->dists[depots[min_index]->seq].dist < b->dists[depots[min_index]->seq].dist; });
		for (auto& node : classfy[i]) {
			if (!vehicle.move(node)) {                         // 达到最大
				vehicle.path.emplace_back(depots[min_index]);  // 返回厂站
				vehicle.cumlength = vehicle.path_length();     // 计算路径长度。
				solution.add(vehicle);                         // 加入到答案。
				vehicle.clear(depots[min_index], num++);       // 清空
				vehicle.move(node);                            // 补上
			}
		}
		if (!vehicle.path.empty()) {                       // 最后的客户
			vehicle.path.emplace_back(depots[min_index]);  // 返回厂站
			vehicle.cumlength = vehicle.path_length();     // 计算路径长度。
			solution.add(vehicle);                         // 加入到答案。
			vehicle.clear(depots[min_index], num++);       // 清空
		}
	}
	for (uint32_t i = 0; i < solution.solution.size(); i++) {
		for (uint32_t j = 1; j < solution.solution[i].path.size() - 1; j++) {              // 排除厂站
			solution.shash.emplace(std::make_pair(solution.solution[i].path[j]->seq, i));  // 建立hash 查找表
		}
	}
	return solution;
}

Solution nassign(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const int maxload) {
	Solution solution;
	uint32_t num{0};  // 路线序号
	// constexpr int MaxLoad = 200;
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
		Vehicle vehicle(station[i], maxload, num++);
		std::sort(classfy[i].begin(), classfy[i].end(), [i](const Node* a, const Node* b) { return a->dists[i].dist < b->dists[i].dist; });
		for (auto& node : classfy[i]) {
			if (!vehicle.move(node)) {                      // 达到最大
				vehicle.path.emplace_back(station[i]);      // 返回厂站
				vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
				solution.add(vehicle);                      // 加入到答案。
				vehicle.clear(station[i], num++);           // 清空
				vehicle.move(node);                         // 补上
			}
		}
		if (!vehicle.path.empty()) {                    // 最后的客户
			vehicle.path.emplace_back(station[i]);      // 返回厂站
			vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
			solution.add(vehicle);                      // 加入到答案。
			vehicle.clear(station[i], num++);           // 清空
		}
	}
	for (uint32_t i = 0; i < solution.solution.size(); i++) {
		for (uint32_t j = 1; j < solution.solution[i].path.size() - 1; j++) {              // 排除厂站
			solution.shash.emplace(std::make_pair(solution.solution[i].path[j]->seq, i));  // 建立hash 查找表
		}
	}
	return solution;
}

Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}