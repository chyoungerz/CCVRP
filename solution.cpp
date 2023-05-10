#include "solution.hpp"

#include <set>

Solution cw(const std::vector<Node>& nodes, const Eigen::MatrixXf& dists) {
}

Solution knn(const std::vector<Node>& nodes, const Eigen::MatrixXf& dists) {
	Solution solution;
	Vehicle vehicle;
	uint32_t size = nodes.size();
	std::set<uint32_t> walked;
	Eigen::Vector<float, -1> dist0;
	dist0 = dists.col(0);
	for (int i = 0; i < size; i++) {
	}
	bool flags{true};
	while (!walked.empty()) {
	}
}

Solution NInsertion(const std::vector<Node>& nodes, const Eigen::MatrixXf& dists) {}

Solution SweepA(const std::vector<Node>& nodes, const Eigen::MatrixXf& dists) {}