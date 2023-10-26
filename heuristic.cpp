
#include "heuristic.hpp"

#include <algorithm>
#include <random>
#include <set>
#include <vector>

#include "NSearch.hpp"
// #include "algorithm.hpp"
#include "node.hpp"
#include "operator.hpp"
#include "solution.hpp"

// SA算法
void SA::init(std::vector<const Node*>& node, const u32 depot_num, u32 maxload, u32 routes) {
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
	// u32 customer = nodes.size() - depotnum;
	//  sol.show();
	bool flag{true};
	u32 epoch{10};
	u32 nodesize = nodes.size();
	u32 custer{nodesize - depotnum};
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
--------------------GA start-------------------
*/
void GA::encode(Solution& sol, std::vector<const Node*>& code) {}

void GA::decode(std::vector<const Node*>& code, Solution& sol) {}
/*
--------------------GA end-------------------
*/

/*
--------------------SWO start-------------------
*/

void SWO::init(std::vector<const Node*>& node, const u32 depot_num, u32 maxload, u32 routes) {
}

void SWO::encode(Solution& sol, std::vector<const Node*>& code) {
	std::vector<Vehicle> temp(sol.solution);
	if (!sol.multi) {
		// 单场站
		std::sort(temp.begin(), temp.end(), [](const Vehicle& v1, const Vehicle& v2) { return v1.path.front() < v2.path.front(); });  // 排序，以便消除误差
	} else {
		// 多场站
		std::sort(temp.begin(), temp.end(), [](const Vehicle& v1, const Vehicle& v2) {  // 排序，以便消除误差
			if (v1.depot <= v2.depot)
				return v1.path.front() < v2.path.front();
			else
				return false;
		});
	}
	for (auto& p : temp) {
		code.emplace_back(p.depot);
		for (auto& i : p.path) {
			code.emplace_back(i);
		}
	}
}

void SWO::decode(std::vector<const Node*>& code, Solution& sol) {
	Vehicle v(code.front(), maxload, 0);
	for (u32 i{1}, n = code.size(); i < n; i++) {
		if (!code[i]->isdepot) {
			v.path.emplace_back(code[i]);
		} else {
			sol.add(v);
			v.clear(nodes.front(), 0);  // 下一个车辆
		}
	}
	sol.add(v);  // z最后一个车辆
}

Solution SWO::cross(Solution& sol1, Solution& sol2, float cr) {
	std::random_device rd;
	std::mt19937 gen(rd());
	Solution child_sol;
	u64 num{nodes.size()};
	std::vector<const Node*> code1, code2, child_code;
	code1.reserve(num), code2.reserve(num), child_code.reserve(num);
	encode(sol1, code1), encode(sol2, code2);
	std::vector<u32> range;
	range.reserve(maxvehicles);
	std::set<const Node*> tabu;
	for (u32 i{0}, n = code1.size(); i < n; i++) {
		if (code1[i]->isdepot) {
			range.emplace_back(i);
		}
	}
	range.emplace_back(code1.size());
	std::shuffle(range.begin(), range.end(), gen);
	for (u32 i{0}, n = range.size() * cr; i < n; i++) {
		child_code.emplace_back(code1[range[i]]);
		for (u32 j{range[i] + 1}; !code1[j]->isdepot; j++) {
			child_code.emplace_back(code1[range[j]]);
			tabu.emplace(code1[range[j]]);
		}
	}
	for (auto& i : code2) {
		if (tabu.find(i) == tabu.end()) {
			child_code.emplace_back(i);
		}
	}
	decode(child_code, child_sol);
	std::for_each(solutions.begin(), solutions.end(), [](Solution& sol) {
		sol.remove_void();  // 去除空路线
		sol.evaluate();     // 约束条件
		sol.update(1);      // 计算目标函数
	});
	return child_sol;
}

void SWO::run(u32 min_n, int epoch, float tr, float cr) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	int t{epoch};
	u32 num = solutions.size();
	u32 custer = nodes.size() - depotnum;
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
			std::sort(solutions.begin(), solutions.end(), [](const Solution& a, const Solution& b) { return a.allength <= b.allength; });
			u32 num = solutions.size();
			for (u32 i{0}, n = num * cr; i < n; i++) {
				solutions.emplace_back(SWO::cross(solutions[i], solutions[i + 1], cr / 2));
			}
		}
		// 种群数量控制保留最好的
		num = min_n + (num - min_n) * k;
		std::partial_sort(solutions.begin(), solutions.begin() + num, solutions.end(), [](const Solution& a, const Solution& b) {
			if (a.valid && b.valid)
				return a.allength < b.allength;
			else if (!a.valid)
				return false;
			else
				return true;
		});
		solutions.resize(num);
		bestSol = solutions.front();
		t++;
	}
}
/*
--------------------SWO start-------------------
*/