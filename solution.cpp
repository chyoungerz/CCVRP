#include "solution.hpp"

#include <algorithm>
#include <set>

Solution cw(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}

Solution knn(const std::vector<Node*>& nodes, const std::vector<Node*>& station) {
	Solution solution;
	// uint32_t size = nodes.size();
	std::set<uint32_t> walked;                       // nodes we have already walked through so far.
	for (uint32_t i = 0; i < station.size(); i++) {  // 排除厂站
		walked.insert(station[i]->seq);
	}
	for (uint32_t i = 0; i < station.size(); i++) {
		std::vector<Edge> dist0 = station[i]->distances;
		Vehicle vehicle(station[i], 0.0);
		std::sort(dist0.begin(), dist0.end(), [](Edge a, Edge b) -> bool { return a.distance < b.distance; });
		while (true) {
			uint32_t far{dist0.back().to};
			if (walked.find(far) == walked.end()) {
				if (!vehicle.move(nodes[far])) {    // 达到最大
					vehicle.move(station[i]);       // 返回厂站
					vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
					solution.add(vehicle);          // 加入到答案。
					vehicle.clear(station[i]);      // 清空
					break;
				} else {                            // 没有达到
					walked.insert(far);
					while (true) {
						std::vector<Edge> distnode = nodes[far]->distances;
						std::sort(distnode.begin(), distnode.end(), [](Edge a, Edge b) -> bool { return a.distance > b.distance; });
						far = distnode.back().to;                   // 下一个位置
						while (walked.find(far) != walked.end()) {  // 没访问的位置
							distnode.pop_back();
							far = distnode.back().to;
						}
						if (distnode.empty()) {                  // 走完了
							vehicle.move(station[i]);            // 返回厂站
							vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
							solution.add(vehicle);               // 加入到答案。
							vehicle.clear(station[i]);           // 清空;
							break;
						} else if (!vehicle.move(nodes[far])) {  // 达到最大
							vehicle.move(station[i]);            // 返回厂站
							vehicle.cumlength = vehicle.path_length();  // 计算路径长度。
							solution.add(vehicle);               // 加入到答案。
							vehicle.clear(station[i]);           // 清空;
							break;
						} else {                                 // 没有达到
							walked.insert(far);                  // 记录已经访问过的位置。
						}
					}
					dist0.pop_back();
				}
			} else {
				dist0.pop_back();
			}
			if (dist0.empty()) break;
		}
	}
	return solution;  // 返回答案。
}
Solution NInsertion(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}

Solution SweepA(const std::vector<Node>& nodes, const std::vector<Node>& station) {
	Solution solution;
	return solution;
}