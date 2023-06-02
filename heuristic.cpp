
#include "heuristic.hpp"

#include "distribution.hpp"
#include "operator.hpp"
#include "solution.hpp"
#include "utils.hpp"

// SA算法
void SA::init(const std::vector<Node*>& nodes, const std::vector<Node*>& station) {
	bestSol = knn(nodes, station);
}
