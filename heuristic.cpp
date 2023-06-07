
#include "heuristic.hpp"

#include "NSearch.hpp"
#include "distribution.hpp"
#include "operator.hpp"
#include "solution.hpp"
#include "utils.hpp"

// SA算法
void SA::init(const std::vector<Node*>& nodes, const std::vector<Node*>& station, const char cmd) {
	if (cmd == 'n')
		sol = bestSol = nassign(nodes, station);
	else if (cmd == 'g')
		sol = bestSol = greedynear(nodes, station);
	else
		;
}

void SA::run() {
	sol.show();
}