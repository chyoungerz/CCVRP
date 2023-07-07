
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
	sol = bestSol = greedynear(nodes, depot_num, maxload);
	sol.show();
}

void SA::run() {
	// OP::twoOpt(sol);
	OP::twoNSwap(sol);
	sol.update();
	sol.show();
	// sum = 0;
	// for (auto& i : sol.solution) {
	//	sum += i.path.size();
	// }
	// std::cout << sum << std::endl;
}