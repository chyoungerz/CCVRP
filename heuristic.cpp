
#include "heuristic.hpp"

#include "NSearch.hpp"
// #include "distribution.hpp"
#include <algorithm>

#include "operator.hpp"
#include "solution.hpp"
// #include ".hpp"

// SA算法
void SA::init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload, uint32_t routes) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	sol = greedynear(node, depot_num, maxload);
	depotnum = depot_num;
	nodes = node;
	vehicles = routes;
	// sol.show();
}

void SA::run() {
	// uint32_t customer = nodes.size() - depotnum;
	//  sol.show();
	bool flag{true};
	uint32_t epoch{1000};
	while (flag && epoch) {
		flag = false;
		LS::one(sol, flag);
		LS::two(sol, flag);
		LS::twoOpt(sol, flag);
		LS::three(sol, flag);
		epoch--;
	}
	sol.update();
	sol.show();
	//       sum = 0;
	//       for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//       }
	//       std::cout << sum << std::endl;
}