
#include "heuristic.hpp"

#include "distribution.hpp"
#include "optimization.hpp"
#include "utils.hpp"

//ILS算法
void ILS::init(std::vector<Node>& nodes, const unsigned int num, const unsigned int epoch) {
	for (unsigned int i = 0; i < nodes.size(); i++) {
		//vehicles[i];
	}
}

//ALNS算法
void ALNS::init(std::vector<Vehicle>& vehicles, const unsigned int num, const unsigned int epoch) {
	double all_path_length = 0.0;
	for (unsigned int i = 0; i < vehicles.size(); i++) {
		all_path_length += vehicles[i].length;
	}
	//twoOptMove(vehicles[0].path, 1, 2);
}
