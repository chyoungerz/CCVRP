
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
	sol.show();
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	OP::oneNMove(sol, nodes, depotnum);
	OP::twoNSwap(sol);
	sol.update();
	sol.show();
	//   sum = 0;
	//   for (auto& i : sol.solution) {
	//	sum += i.path.size();
	//   }
	//   std::cout << sum << std::endl;
}