#include "operation.hpp"
#define MAXLOAD 200  // 定义最大载货

bool insertback(Vehicle& vehicle, Node& node) {
	if ((vehicle.load + node.demand) > MAXLOAD) return false;                                         // 超重
	vehicle.load += node.demand;
	vehicle.diflength.push_back(vehicle.diflength.back() + node.distances[vehicle.locate].distance);  // from a to b == from b to a
	vehicle.locate = node.seq;
	vehicle.path.push_back(node.seq);
	return true;
}

bool insert(Vehicle& vehicle, const Node& node, const uint32_t pos) {
	if ((vehicle.load + node.demand) > MAXLOAD) return false;  // 超重
}