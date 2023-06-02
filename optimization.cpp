#include "optimization.hpp"

#include <random>

#include "operation.hpp"

bool LS::twoPointMove(Solution& solution, uint32_t stop) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> distrib(0, solution.solution.size() - 1);

	return true;
}