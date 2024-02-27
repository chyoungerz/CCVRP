
#include "heuristic.hpp"

#include <algorithm>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "NSearch.hpp"
#include "algorithm.hpp"
#include "node.hpp"
#include "operator.hpp"
#include "solution.hpp"

// SA算法
void VN::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	// sol = nassign(node, depot_num, maxload, routes);
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = nassign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.allength = 100000000.0;
}

void VN::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void VN::run() {
	// u32 customer = nodes.size() - depotnum;
	//  sol.show();
	bool improved{1}, flag{0}, change{1};
	int max_epoch{30};
	int epoch{max_epoch};
	float size_near{0.5}, T{1.0}, cold_rate{0.95};
	int vns[7] = {1, 2, 3, 4, 5, 6, 7};
	u32 maxcustomers = customers.size() * 10 > 100 ? customers.size() * 10 : 100;
	u32 stop{maxcustomers}, timelimit{0};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	// sol.evaluate();
	while (stop-- && improved) {
		improved = false;
		VNS::relocate(sol, info.one, size_near, improved);
		VNS::oropt2(sol, info.or2, size_near, improved);
		VNS::arcnode(sol, info.three, size_near, improved);
		VNS::oropt3(sol, info.or3, size_near, improved);
		VNS::oropt4(sol, info.or4, size_near, improved);
		VNS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers;
#ifdef PR
	sol.alltardiness = priority(sol);
#endif
	sol.update();
	// sol.show();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		// std::shuffle(vns, vns + 7, gen);
		//   if (T > 0.2) {
		//	float r = dis(gen);
		//	if (dis(gen) < 0.5) {
		//		// PER::EjecChain(sol, vehicles * T > 2 ? vehicles * T : 2, 10 * T > 1 ? 10 * T : 1, 0);
		//		PER::EjecChain(sol, vehicles * r > 2 ? vehicles * r : 2, 10 * r > 1 ? 10 * r : 1, 0);
		//	} else {
		//		// PER::RuinCreate(sol, T > 0.2 ? T / 2 : 0.1, customers, 10, 2);
		//		PER::RuinCreate(sol, r > 0.2 ? r / 2 : 0.1, customers, 10, 2);
		//	}
		//  } else {
		if (dis(gen) < 0.5) {
			PER::EjecChain(sol, vehicles * (1 - T) > 2 ? vehicles * (1 - T) : 2, 10 * (1 - T) > 1 ? 10 * (1 - T) : 1, 0);
			// PER::EjecChain(sol, vehicles * T > 2 ? vehicles * T : 2, 10 * T > 1 ? 10 * T : 1, 0);
			// PER::EjecChain(sol, vehicles * r > 2 ? vehicles * r : 2, 10 * r > 1 ? 10 * r : 1, 0);
		} else {
			PER::RuinCreate(sol, (1 - T) / 2 > 0.1 ? (1 - T) / 2 : 0.1, customers, 10, 2);
			// PER::RuinCreate(sol, T > 0.2 ? T / 2 : 0.1, customers, 10, 2);
			// PER::RuinCreate(sol, r > 0.2 ? r / 2 : 0.1, customers, 10, 2);
		}
		//}
		for (u32 n{0}; n < 7;) {
			flag = 0;
			if (timelimit > maxcustomers) break;
			switch (vns[n]) {
			case 1:
				while (stop--) {
					VNS::exchange(sol, info.two, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 2:
				while (stop--) {
					VNS::relocate(sol, info.one, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 3:
				while (stop--) {
					VNS::arcnode(sol, info.three, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 4:
				while (stop--) {
					VNS::arcswap(sol, info.arc, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 5:
				while (stop--) {
					VNS::oropt2(sol, info.or2, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 6:
				while (stop--) {
					VNS::oropt3(sol, info.or3, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 7:
				while (stop--) {
					VNS::oropt4(sol, info.or4, size_near, improved);
					if (improved) {
						VNS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			}
			timelimit++;
			if (stop == 0) flag = 0;
			stop = maxcustomers;
			if (flag)
				n = 0;
			else
				n++;
		}
		timelimit = 0;
		stop = maxcustomers;
		sol.update();
		if (sol.valid) {
			if (sol.allobj < bestSol.allobj) {
				bestSol = sol;
			}
			if (T < 0.2 && change) {
				if (bestSol.valid)
					lsbest = bestSol;
				change = 0;
			}
			if (sol.allobj < lsbest.allobj) {
				lsbest = sol;
				epoch = max_epoch;
			} else if (dis(gen) < T) {
				lsbest = sol;
				epoch = max_epoch;
			} else {
				sol = lsbest;
				epoch--;
			}
		} else {
			epoch--;
		}
		T *= cold_rate;
	}
	if (bestSol.allobj > lsbest.allobj || !bestSol.valid) {
		bestSol = std::move(lsbest);
	}
	// bestSol.show();
	//  infos.emplace_back(info);
	//  sol.update();
	//  sol.show();
	//  for (u32 i{0}, n = infos.size(); i < n; i++) {
	// std::cout << "one: " << info.one
	//           << " two: " << info.two
	//           << " three: " << info.three
	//           << " or2: " << info.or2
	//           << " or3: " << info.or3
	//           << " or4: " << info.or4
	//           << " 2-opt: " << info.opt2 << "\n";
	// }
}

void SA::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = assign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
}

void SA::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void SA::run() {
	bool improved{1}, change{1};
	int max_epoch{10};
	int epoch{max_epoch};
	float size_near{0.5}, T{1.0}, cold_rate{0.93};
	u32 maxcustomers = customers.size();
	u32 stop{maxcustomers};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	// sol.evaluate();
	while (stop-- && improved) {
		improved = false;
		VNS::relocate(sol, info.one, size_near, improved);
		VNS::oropt2(sol, info.or2, size_near, improved);
		VNS::arcnode(sol, info.three, size_near, improved);
		VNS::oropt3(sol, info.or3, size_near, improved);
		VNS::oropt4(sol, info.or4, size_near, improved);
		VNS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers * 10;
	sol.alltardiness = priority(sol);
	sol.update();
	// sol.show();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		// std::shuffle(vns, vns + 7, gen);
		if (dis(gen) < 0.5) {
			u32 k = vehicles / 2;
			if (k < 2)
				k = 2;
			else if (k > 4)
				k = 4;
			PER::EjecChain(sol, k, 50);
			// PER::EjecChain(sol, vehicles * T > 2 ? vehicles * T : 2, 10 * T > 1 ? 10 * T : 1, 0);
			// PER::EjecChain(sol, vehicles * r > 2 ? vehicles * r : 2, 10 * r > 1 ? 10 * r : 1, 0);
		} else {
			PER::RuinCreate(sol, 0.3, customers, 10);
			// PER::RuinCreate(sol, T > 0.2 ? T / 2 : 0.1, customers, 10, 2);
			// PER::RuinCreate(sol, r > 0.2 ? r / 2 : 0.1, customers, 10, 2);
		}
		//}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::relocate(sol, info.one, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::oropt2(sol, info.or2, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::arcnode(sol, info.three, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::oropt3(sol, info.or3, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::oropt4(sol, info.or4, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::arcswap(sol, info.arc, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::exchange(sol, info.two, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			VNS::twoopt(sol, info.opt2, improved);
		}
		improved = 1;
		// sol.update();
		if (sol.valid) {
			if (sol.allobj < bestSol.allobj) {
				bestSol = sol;
			}
			if (T < 0.2 && change) {
				if (bestSol.valid)
					lsbest = bestSol;
				change = 0;
			}
			if (sol.allobj < lsbest.allobj) {
				lsbest = sol;
				epoch = max_epoch;
			} else if (dis(gen) < T) {
				lsbest = sol;
				epoch = max_epoch;
			} else {
				sol = lsbest;
				epoch--;
			}
			T *= cold_rate;
		}
		stop--;
		if (stop == 0) break;
	}
	if (bestSol.allobj > lsbest.allobj || !bestSol.valid) {
		bestSol = std::move(lsbest);
	}
}

void VND::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = assign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
}

void VND::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void VND::run() {
	bool improved{1};
	int max_epoch{10};
	int epoch{max_epoch};
	float size_near{0.5};
	u32 maxcustomers = customers.size();
	u32 stop{maxcustomers};
	int vns[4] = {1, 2, 3, 4};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	// sol.evaluate();
	while (stop-- && improved) {
		improved = false;
		VNS::relocate(sol, info.one, size_near, improved);
		VNS::oropt2(sol, info.or2, size_near, improved);
		VNS::arcnode(sol, info.three, size_near, improved);
		VNS::oropt3(sol, info.or3, size_near, improved);
		VNS::oropt4(sol, info.or4, size_near, improved);
		VNS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers * 10;
	sol.alltardiness = priority(sol);
	sol.update();
	// sol.show();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		// std::shuffle(vns, vns + 7, gen);
		for (u32 i{0}; i < 4;) {
			if (vns[i] == 1) {
				SHACK::twoopt(sol, 0.1, 50);
			} else if (vns[i] == 2) {
				SHACK::oropt(sol, 0.1, 50);
			} else if (vns[i] == 3) {
				SHACK::arcnode(sol, 0.1, 50);
			} else if (vns[i] == 4) {
				SHACK::arcswap(sol, 0.1, 50);
			}
			improved = 1;
			while (improved) {
				improved = false;
				VNS::relocate(sol, info.one, size_near, improved);
				VNS::exchange(sol, info.two, size_near, improved);
				VNS::oropt2(sol, info.or2, size_near, improved);
				VNS::arcnode(sol, info.three, size_near, improved);
				VNS::oropt3(sol, info.or3, size_near, improved);
				VNS::oropt4(sol, info.or4, size_near, improved);
				VNS::arcswap(sol, info.arc, improved);
				VNS::twoopt(sol, info.opt2, improved);
				// if (improved) change = 1;
				stop--;
				if (stop == 0) break;
			}
			if (sol.valid) {
				if (sol.allobj < lsbest.allobj) {
					lsbest = sol;
					i = 0;
				} else {
					sol = lsbest;
					i++;
				}
			} else {
				lsbest = sol;
				i++;
			}
			stop = maxcustomers * 10;
		}
		if (lsbest.valid) {
			if (lsbest.allobj < bestSol.allobj) {
				bestSol = lsbest;
				epoch = max_epoch;
			} else {
				lsbest = bestSol;
				epoch--;
			}
		} else {
			lsbest = bestSol;
			epoch--;
		}
	}
}

/*
--------------------GA start-------------------
*/
void GA::encode(Solution& sol, std::vector<Node*>& code) {}

void GA::decode(std::vector<Node*>& code, Solution& sol) {}
/*
--------------------GA end-------------------
*/

/*
--------------------SWO start-------------------
*/

/// @brief 蜘蛛蜂优化算法初始化
/// @param node
/// @param depot_num
/// @param maxload
/// @param routes
void SWO::init(std::vector<Node*>& node, const u32 depot_num, u32 maxload, u32 routes) {
}

/// @brief 编码
/// @param code
/// @param sol
void SWO::encode(Solution& sol, std::vector<Node*>& code) {
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

/// @brief 解码
/// @param code
/// @param sol
void SWO::decode(std::vector<Node*>& code, Solution& sol) {
	Vehicle v(code.front(), maxload, 0);
	for (auto i{code.begin() + 1}; i != code.end(); i++) {
		if (!(*i)->isdepot) {
			v.path.emplace_back(*i);
		} else {
			sol.add(v);
			v.clear(nodes.front(), 0);  // 下一个车辆
		}
	}
	sol.add(v);  // z最后一个车辆
}

/// @brief 在两个解决方案之间执行交叉操作，生成一个新的解决方案。
/// @param sol1 要进行交叉的第一个解决方案。
/// @param sol2 要进行交叉的第二个解决方案。
/// @param cr 交叉率，确定从每个解决方案中选择的节点比例。
/// @return Solution 通过交叉操作生成的新解决方案。
Solution SWO::cross(Solution& sol1, Solution& sol2, float cr) {
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	Solution child_sol;
	u64 num{nodes.size()};
	std::vector<Node*> code1, code2, child_code;
	code1.reserve(num), code2.reserve(num), child_code.reserve(num);
	encode(sol1, code1), encode(sol2, code2);
	std::vector<u32> range;
	range.reserve(maxvehicles);
	std::set<Node*> tabu;
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
		// sol.evaluate();     // 约束条件
		sol.update(1);      // 计算目标函数
	});
	return child_sol;
}

/// @brief SWO主体（包括路径首位不包括厂站）
/// @param min_n 最小种群数
/// @param epoch 迭代次数
/// @param tr 狩猎筑巢 : 繁殖
/// @param cr 交叉率
void SWO::run(u32 min_n, int epoch, float tr, float cr) {
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	int t{epoch};
	u32 num = solutions.size();
	// u32 custer = nodes.size() - depotnum;
	while (t < epoch) {
		float k{1 - t / static_cast<float>(epoch)};
		if (dis(gen) < tr) {     // 狩猎筑巢
			if (dis(gen) < k) {  // 狩猎(扰动)
				for (auto& sol : solutions) {
					sol.update_hash(true);
					// PER::RuinCreate(sol, custer / 10, custer);
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
--------------------SWO end-------------------
*/

/*
-------------------RL start-------------------
*/

class RL {
};

/*
-------------------RL end-------------------
*/
