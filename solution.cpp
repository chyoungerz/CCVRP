#include "solution.hpp"

#include <algorithm>
#include <vector>
// #include <set>

#include "algorithm.hpp"

Solution cw(std::vector<const Node*>& nodes, const u32 depot_num, const u32 maxload, const u32 routes) {
	Solution solution;
	if (depot_num == 0) {  // 单场站
		std::vector<std::vector<const Node*>> paths(nodes.size());

	} else {  // 多场站
	}
	return solution;
}

/*
Solution greedynear(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const int maxload) {
    Solution solution;
    // constexpr int MaxLoad = 200;
    std::vector<std::vector<const Node*>> classfy(station.size());
    std::set<u32> tabu;  // 排除厂站
    for (u32 i = 0; i < station.size(); i++) {
        tabu.emplace(station[i]->seq);
    }
    for (auto& node : nodes) {                             // 将每个客户分给距离最近的厂站；
        if (tabu.find(node->seq) != tabu.end()) continue;  // 排除厂站
        for (u32 i = 1; i < node->distsort.size(); i++) {
            if (tabu.find(node->distsort[i].to) != tabu.end()) {
                classfy[node->distsort[i].to].emplace_back(node);
                break;
            }
        }
    }
    for (u32 i = 0; i < station.size(); i++) {
        Vehicle vehicle(station[i], maxload);
        std::sort(classfy[i].begin(), classfy[i].end(), [i](const Node* a, const Node* b) { return a->dists[i].dist > b->dists[i].dist; });
        std::set<u32> walked, service;  // 每个厂站已服务的节点, 和需要服务的节点
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
    for (u32 i = 0; i < solution.solution.size(); i++) {
        for (u32 j = 1; j < solution.solution[i].path.size() - 1; j++) {              // 排除厂站
            solution.shash.emplace(std::make_pair(solution.solution[i].path[j]->seq, i));  // 建立hash 查找表
        }
    }
    return solution;  // 返回答案。
}*/

Solution greedynear(std::vector<Node*>& nodes, const u32 depot_num, const u32 maxload) {
	Solution solution;
	std::vector<Node*> depots, custers;
	u32 num{0};  // 路线序号
	std::vector<std::vector<Node*>> classfy(depot_num);
	custers.assign(nodes.begin(), nodes.end() - depot_num);
	depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	std::vector<Node> K;
	K.reserve(depot_num);
	for (auto& i : depots) {
		K.emplace_back(*i);  // K
	}
	// ALG::Kmean(nodes, K, 25, classfy);
	for (u32 i = 0; i < depot_num; i++) {  // 将分好类的客户安排给最近厂站
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
				vehicle.path_cumlength(true);                  // 计算路径长度。
				solution.add(vehicle);                         // 加入到答案。
				vehicle.clear(depots[min_index], num++);       // 清空
				vehicle.move(node);                            // 补上
			}
		}
		if (!vehicle.path.empty()) {                       // 最后的客户
			vehicle.path.emplace_back(depots[min_index]);  // 返回厂站
			vehicle.path_cumlength(true);                  // 计算路径长度。
			solution.add(vehicle);                         // 加入到答案。
			vehicle.clear(depots[min_index], num);         // 清空
		}
	}
	for (u32 i = 0; i < solution.solution.size(); i++) {
		for (u32 j = 1; j < solution.solution[i].path.size() - 1; j++) {                   // 排除厂站
			solution.shash.emplace(solution.solution[i].path[j]->seq, i);                  // 建立hash 查找表
		}
	}
	return solution;
}

Solution nassign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes, u32& ctrl) {
	Solution solution;
	if (depots.size() == 1) {  // 单厂站
		solution.multi = false;
		solution.maxvehicle = routes;
		Node* depot{depots.front()};
		Vehicle vehicle(depot, maxload, 0);  // 初始路线
		// 按离厂站距离排序
		std::sort(customers.begin(), customers.end(), [](const Node* a, const Node* b) {
			if (a->end < b->end)
				return a->demand > b->demand;
			else
				return false;
		});
		solution.add(vehicle, routes);
		u32 maxcust = customers.size();
		for (u32 i{0};; i++) {
			if (i < maxcust)
				solution.solution[i % routes].path.emplace_back(customers[i]);
			else
				break;
		}
		for (auto& v : solution.solution) {
			v.path.emplace_back(depot);
			v.path_cumlength(true);
		}
		/*
		for (auto& node : customers) {
		    if (!vehicle.move(node)) {             // 达到最大
		        vehicle.path.emplace_back(depot);  // 返回厂站
		        vehicle.path_cumlength(true);      // 计算路径长度。
		        solution.add(vehicle);             // 加入到答案。
		        vehicle.clear(depot, 0);           // 清空
		        vehicle.move(node);                // 补上
		    }
		}
		if (!vehicle.path.empty()) {           // 最后的客户
		    vehicle.path.emplace_back(depot);  // 返回厂站
		    vehicle.path_cumlength(true);      // 计算路径长度。
		    solution.add(vehicle);             // 加入到答案。
		    vehicle.clear(depot, 0);           // 清空
		}
		*/
	} else {  // 多厂站(未测试)
		solution.multi = true;
		solution.maxvehicle = routes;
		u32 depot_num = depots.size();
		std::vector<std::vector<Node*>> classfy(depot_num);
		std::vector<Node> K;
		K.reserve(depot_num);
		for (auto& i : depots) {
			K.emplace_back(*i);  // K
		}
		// 多厂站
		ALG::Kmean(customers, depots, K, 25, classfy);
		for (u32 i = 0; i < depot_num; i++) {  // 将分好类的客户安排给最近厂站
			double min_dist = 1000000.0;
			unsigned int min_index = 10000;
			for (unsigned int j = 0; j < depot_num; ++j) {
				double d = dist(K[i], *depots[j]);
				if (d < min_dist) {
					min_dist = d;
					min_index = j;
				}
			}
			Vehicle vehicle(depots[min_index], maxload, 0);  // 初始路线
			// 按离厂站距离排序
			std::sort(classfy[i].begin(), classfy[i].end(), [](const Node* a, const Node* b) {
				if (a->end <= b->end)
					return a->demand < b->demand;
				else
					return false;
			});
			for (auto& node : classfy[i]) {
				if (!vehicle.move(node)) {                         // 达到最大
					vehicle.path.emplace_back(depots[min_index]);  // 返回厂站
					vehicle.path_cumlength(true);                  // 计算路径长度。
					solution.add(vehicle);                         // 加入到答案。
					vehicle.clear(depots[min_index], 0);           // 清空
					vehicle.move(node);                            // 补上
				}
			}
			if (!vehicle.path.empty()) {                       // 最后的客户
				vehicle.path.emplace_back(depots[min_index]);  // 返回厂站
				vehicle.path_cumlength(true);                  // 计算路径长度。
				solution.add(vehicle);                         // 加入到答案。
				vehicle.clear(depots[min_index], 0);           // 清空
			}
		}
	}
	/*if (solution.solution.size() < routes) {               // 加车
	    if (depot_num == 1) {
	        Vehicle vehicle(depots.back(), maxload, num);  // 初始路线
	        while (solution.solution.size() == routes) {
	            solution.add(vehicle);                     // 加入到答案。
	            vehicle.clear(depots.back(), num);         // 清空
	        }
	    } else {
	        Vehicle vehicle(depots.front(), maxload, num);  // 初始路线
	        for (auto& d : depots) {
	            vehicle.clear(d, num);                      // 清空
	            solution.add(vehicle);                      // 加入到答案。
	            if (solution.solution.size() == routes) break;
	        }
	    }
	}
	if (solution.solution.size() > routes) {  // 减少车辆
	    if (depot_num != 1 && !seq.empty()) {
	        std::sort(seq.begin(), seq.end(), [&solution](u32& a, u32& b) -> bool { return solution.solution[a].path.size() > solution.solution[b].path.size(); });
	        bool flag{true};
	        u32 size = solution.solution.size();
	        while (size > routes && flag) {
	            flag = false;
	            for (u32 j{0}, m = seq.size() - 1; j < m; j++) {
	                for (u32 i{1}, n = solution.solution[seq.back()].path.size() - 1; i < n; i++) {
	                    if (solution.solution[seq[j]].load + solution.solution[seq.back()].path[i]->demand <= solution.solution[seq[j]].capacity) {
	                        solution.solution[seq[j]].path.emplace(solution.solution[seq[j]].path.end() - 1, solution.solution[seq.back()].path[i]);
	                        solution.solution[seq[j]].load += solution.solution[seq.back()].path[i]->demand;
	                        solution.solution[seq.back()].path[i] = nullptr;
	                        flag = true;
	                    }
	                }
	                solution.solution[seq.back()].path.erase(std::remove(solution.solution[seq.back()].path.begin(), solution.solution[seq.back()].path.end(), nullptr), solution.solution[seq.back()].path.end());
	                if (solution.solution[seq.back()].path.size() <= 2) {
	                    seq.pop_back();
	                    size--;
	                    break;
	                }
	            }
	        }
	    } else {

	    }
	}
	// 删除空车
	solution.solution.erase(std::remove_if(solution.solution.begin(), solution.solution.end(), [](Vehicle& a) -> bool {if(a.path.size() <= 2) return true; else return false; }), solution.solution.end());
	*/
	// 重建序号
	solution.update_seq();
	solution.alltardiness = 0.0;
	solution.update();
	solution.evaluate(ctrl);
	solution.alltardiness = 0.0;
	// hash
	// for (u32 i = 0; i < solution.solution.size(); i++) {
	//	for (u32 j = 1; j < solution.solution[i].path.size() - 1; j++) {                                          // 排除厂站
	//		solution.shash.emplace(solution.solution[i].path[j]->seq, solution.solution[i].seq);                  // 建立hash 查找表
	//	}
	//}
	return solution;
}

Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}