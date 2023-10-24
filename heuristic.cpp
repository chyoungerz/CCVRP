
#include "heuristic.hpp"

#include <cstdint>
#include <random>

#include "NSearch.hpp"

// #include "distribution.hpp"
#include "operator.hpp"
#include "solution.hpp"
// #include ".hpp"

// SA算法
void SA::init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload, uint32_t routes) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	sol = nassign(node, depot_num, maxload, routes);
	depotnum = depot_num;
	nodes = node;
	vehicles = routes;
	// sol.show();
}

void SA::run() {
	// uint32_t customer = nodes.size() - depotnum;
	//  sol.show();
	bool flag{true};
	uint32_t epoch{10};
	uint32_t nodesize = nodes.size();
	uint32_t custer{nodesize - depotnum};
	while (flag) {
		flag = false;
		LS::one(sol, flag);
		LS::two(sol, flag);
		LS::twoOpt(sol, flag);
		LS::three(sol, flag);
	}
	sol.update();
	bestSol = sol;
	while (epoch) {
		flag = true;
		sol.update_hash(true);
		PER::RuinCreate(sol, custer / 10, custer);
		while (flag) {
			flag = false;
			LS::one(sol, flag);
			LS::two(sol, flag);
			LS::twoOpt(sol, flag);
			LS::three(sol, flag);
		}
		sol.update();
		if (sol.allength <= bestSol.allength) {
			bestSol = sol;
			epoch = 10;
		}
	}
	//       sum = 0;
	//       for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//       }
	//       std::cout << sum << std::endl;
}

/*
--------------------SWO start-------------------
*/

void SWO::init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload, uint32_t routes) {
}

void SWO::encode(Solution& sol, std::vector<uint32_t>& code) {
	code.clear();
	for (auto& i : sol.solution) {
		for (uint32_t j{1}, n = i.path.size(); j < n; ++j) {
			code.emplace_back(i.path[j]->seq);
		}
	}
}

void SWO::decode(std::vector<uint32_t>& code, Solution& sol) {}

Solution SWO::cross(Solution& sol1, Solution& sol2, float cr) {
	Solution child;
	return child;
}

void SWO::run(uint32_t min_n, int epoch, float tr, float cr) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	int t{epoch};
	uint32_t num = solutions.size();
	uint32_t custer = nodes.size() - depotnum;
	while (t < epoch) {
		float k{1 - t / (float)epoch};
		if (dis(gen) < tr) {     // 狩猎筑巢
			if (dis(gen) < k) {  // 狩猎(扰动)
				for (auto& sol : solutions) {
					sol.update_hash(true);
					PER::RuinCreate(sol, custer / 10, custer);
					sol.update_hash(false);
				}
			} else {  // 筑巢(优化)
				for (auto& sol : solutions) {
					bool flag{true};
					sol.update_hash(true);
					while (flag) {
						flag = false;
						LS::one(sol, flag);
						LS::two(sol, flag);
						LS::twoOpt(sol, flag);
						LS::three(sol, flag);
					}
					sol.update_hash(false);
				}
			}
		} else {  // 繁衍(交叉)
			std::sort(solutions.begin(), solutions.end(), [](const Solution& a, const Solution& b) { return a.allength >= b.allength; });
			for (uint32_t i{0}, n = solutions.size() * cr; i < n; i++) {
				solutions.emplace_back(SWO::cross(solutions[i], solutions[i + 1], 1 - cr));
			}
		}
		// 种群数量控制保留最好的
		num = min_n + (num - min_n) * k;
		std::sort(solutions.begin(), solutions.end(), [](const Solution& a, const Solution& b) { return a.allength >= b.allength; });
		solutions.resize(num);
		t++;
	}
}
/*
--------------------SWO start-------------------
*/