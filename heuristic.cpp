
#include "heuristic.hpp"

#include "NSearch.hpp"
// #include "distribution.hpp"
#include <algorithm>

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
	double difa{}, difb{};
	std::cout << sol.solution[35] << std::endl;
	// std::cout << sol.solution[34] << std::endl;
	CHK::strMove(sol.solution[35], 2, 18, 1, difa);
	// std::rotate(sol.solution[35].path.begin() + 4, sol.solution[35].path.begin() + 6, sol.solution[35].path.begin() + 8);
	sol.solution[35].cumlength += difa;
	sol.solution[34].cumlength += difb;
	std::cout << sol.solution[35] << std::endl;
	// std::cout << sol.solution[34] << std::endl;
	std::cout << sol.solution[35].path_length() << std::endl;
	// std::cout << sol.solution[34].path_length() << std::endl;
	//    sum = 0;
	//    for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//    }
	//    std::cout << sum << std::endl;
}