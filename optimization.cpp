#include "optimization.hpp"

#include <random>

#include "operation.hpp"

bool LS::twoPoint(Solution& solution, uint32_t stop) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> distrib(0, solution.solution.size() - 1);
	while (stop) {
		int a{distrib(gen)}, b{distrib(gen)};
		if (a != b) {
			std::uniform_int_distribution<> distrib_a(1, solution.solution[a].path.size() - 2);
			std::uniform_int_distribution<> distrib_b(1, solution.solution[b].path.size() - 2);
			if (OP::twoswap(solution.solution[a], solution.solution[b], distrib_a(gen), distrib_b(gen))) {
				break;
			}
			stop--;
		} else {
			std::uniform_int_distribution<> distrib_ab(1, solution.solution[a].path.size() - 2);
			if (OP::swaptwo(solution.solution[a], distrib_ab(gen), distrib_ab(gen))) {
				break;
			}
			stop--;
		}
	}
	return true;
}