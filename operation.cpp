#include "operation.hpp"
#define MAXLOAD 200  // 定义最大载货

bool insert(Vehicle& vehicle, const Node* node, const uint32_t pos) {
	if ((vehicle.load + node->demand) > MAXLOAD) return false;                                                                 // 超重
	if (pos >= vehicle.path.size()) return false;                                                                              // 不合法
	double diflength = (vehicle.path.size() - pos - 1) * (vehicle.path[pos]->distances[node->seq].distance + vehicle.path[pos + 1]->distances[node->seq].distance -
														  vehicle.path[pos]->distances[vehicle.path[pos + 1]->seq].distance);  // 计算插入位置之后的时间（距离） future doing
	vehicle.path.insert(vehicle.path.begin() + pos, node);
	for (int i = 0; i <= pos; i++) {                                                                                           // 计算插入的节点
		diflength += vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load += node->demand;
	return true;
}

bool remove(Vehicle& vehicle, const uint32_t pos) {
	if (vehicle.load == 0) return false;                                                                                                                     // 没法删
	if (pos >= vehicle.path.size()) return false;                                                                                                            // 不合法
	double diflength = (vehicle.path.size() - pos - 1) * (vehicle.path[pos - 1]->distances[vehicle.path[pos + 1]->seq].distance -
														  vehicle.path[pos]->distances[pos + 1].distance - vehicle.path[pos - 1]->distances[pos].distance);  // 计算删除位置之后的时间（距离）, future doing
	vehicle.path.erase(vehicle.path.begin() + pos);
	for (int i = 0; i < pos; i++) {                                                                                                                          // 计算插入的节点
		diflength -= vehicle.path[i]->distances[vehicle.path[i + 1]->seq].distance;
	}
	vehicle.cumlength += diflength;  // 更新距离（时间）
	vehicle.load -= vehicle.path[pos]->demand;
}