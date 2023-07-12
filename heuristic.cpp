
#include "heuristic.hpp"

#include "NSearch.hpp"
// #include "distribution.hpp"
#include "operator.hpp"
#include "solution.hpp"
// #include ".hpp"

// SA算法
void SA::init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	sol = greedynear(node, depot_num, maxload);
	depotnum = depot_num;
	nodes = node;
	// sol.show();
}

void SA::run() {
	double difa{};
	std::cout << sol.solution[35] << std::endl;
	CHK::reverse(sol.solution[35], 12, 11, difa);
	sol.solution[35].cumlength += difa;
	std::cout << sol.solution[35] << std::endl;
	std::cout << sol.solution[35].path_length() << std::endl;
	//   sum = 0;
	//   for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//   }
	//   std::cout << sum << std::endl;
}