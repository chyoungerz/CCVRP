
#include "heuristic.hpp"

#include "NSearch.hpp"
// #include "distribution.hpp"
#include "operator.hpp"
#include "solution.hpp"
// #include ".hpp"

// SA算法
void SA::init(std::vector<const Node*>& nodes, const uint32_t depot_num, uint32_t maxload) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	sol = greedynear(nodes, depot_num, maxload);
	// sol.show();
}

void SA::run() {
	// OP::twoOpt(sol);
	double a{}, b{};
	CHK::twoCross(sol.solution[6], sol.solution[13], 4, 3, a, b);
	sol.solution[6].cumlength += a;
	sol.solution[13].cumlength += b;
	std::cout << sol.solution[6].cumlength << std::endl;
	std::cout << sol.solution[13].cumlength << std::endl;
	std::cout << sol.solution[6].path_length() << std::endl;
	std::cout << sol.solution[13].path_length() << std::endl;
	// sol.show();
	//  sum = 0;
	//  for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//  }
	//  std::cout << sum << std::endl;
}