#include "NSearch.hpp"

#include <algorithm>
#include <random>
#include <set>
#include <vector>

#include "algorithm.hpp"
#include "operator.hpp"

void ALNS::repair(Solution& solution, std::vector<std::pair<u32, Node*>>& rest) {
	std::sort(rest.begin(), rest.end(), [](const std::pair<u32, Node*>& a, const std::pair<u32, Node*>& b) { return a.first < b.first; });
	for (auto& r : rest) {
		for (auto near = r.second->distsort.begin() + 1; near != r.second->distsort.end(); near++) {                                                               // 查找所在路径
			if (solution.shash.find((*near).to) != solution.shash.end()) {                                                                                         // 邻域是否也被破坏
				if (solution.solution[solution.shash[(*near).to]].path.front()->seq == r.first) {                                                                  // 邻域是否在同厂站
					if (solution.solution[solution.shash[(*near).to]].load + r.second->demand > solution.solution[solution.shash[(*near).to]].capacity) continue;  // 容量已满
					for (u32 i = 1; i < solution.solution[solution.shash[(*near).to]].path.size() - 1; i++) {                                                      // 查找所在路径中的位置
						if (solution.solution[solution.shash[(*near).to]].path[i]->seq == (*near).to) {                                                            // 找到开始插入
							double db = COST::insertb(solution.solution[solution.shash[(*near).to]].path, r.second, i);
							double df = COST::insertf(solution.solution[solution.shash[(*near).to]].path, r.second, i);
							if (db < df) {
								OP::insertb(solution.solution[solution.shash[(*near).to]], r.second, i, 0.0);
							} else {
								OP::insertf(solution.solution[solution.shash[(*near).to]], r.second, i, 0.0);
							}
							solution.shash.emplace(std::make_pair(r.second->seq, solution.shash[(*near).to]));  // 更新查找表
							break;
						}
					}
					break;  // 跳出二层循环
				}
			}
		}
	}
	rest.clear();
}

void ALNS::destory_wst(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest) {
	std::vector<std::pair<u32, double>> cost;  // （路线节点，长度差值）
	for (auto& r : solution.solution) {
		if (r.path.size() == 2) continue;
		u32 num = std::ceil((r.path.size() - 2) * p);  // 每条路线删除的个数
		for (u32 j = 1; j < r.path.size() - 1; j++) {
			/* 0-a-b-c-d-e-0
			 * a：0a + ab - 0b
			 * b: ab + bc - ac
			 * ...
			 */
			double length = r.path[j]->dists[r.path[j + 1]->seq].dist + r.path[j - 1]->dists[r.path[j]->seq].dist - r.path[j - 1]->dists[r.path[j + 1]->seq].dist;
			cost.emplace_back(std::make_pair(j, length));
		}
		if (num > 2) {  // n ? 2
			// std::sort(cost.begin(), cost.end(), [](const std::pair<u32, double>& a, const std::pair<u32, double>& b) { return a.second < b.second; });
			ALG::topK(cost, 0, cost.size() - 1, num, [](const std::pair<u32, double>& a, const std::pair<u32, double>& b) { return a.second < b.second; });
			while (num) {
				// double dif = COST::erase(r.path, cost.back().first);
				rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, cost.back().first, 0.0)));
				solution.shash.erase(rest.back().second->seq);  // 更新查找表
				cost.pop_back();
				num--;
			}
			OP::erase(r);
		} else {
			while (num) {
				int mx = 0;
				for (u32 i = 0; i < cost.size(); i++) {
					mx = cost[mx].second > cost[i].second ? mx : i;
				}
				// double dif = COST::erase(r.path, cost[mx].first);
				rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, cost[mx].first, 0.0)));
				solution.shash.erase(rest.back().second->seq);  // 更新查找表
				std::swap(cost[mx], cost.back());
				cost.pop_back();
				num--;
			}
			OP::erase(r);
		}
		cost.clear();
	}
}

void ALNS::destory_rnd(Solution& solution, const float p, std::vector<std::pair<u32, Node*>>& rest) {
	std::random_device device_seed;
	std::mt19937 gen(device_seed());
	for (auto& r : solution.solution) {
		if (r.path.size() == 2) continue;
		u32 num = std::ceil((r.path.size() - 2) * p);  // 每条路线删除的个数
		std::uniform_int_distribution<> dis(1, r.path.size() - 2);
		for (u32 i = 0; i < num; i++) {
			u32 rd = dis(gen) - i;  // 随机选取
			double dif = COST::erase(r.path, rd);
			rest.emplace_back(std::make_pair(r.path.back()->seq, OP::remove(r, rd, dif)));
			solution.shash.erase(rest.back().second->seq);  // 更新查找表
		}
	}
}

void LNS::run(Solution& solution, const std::vector<Node*>& nodes, u32 epoch) {
}

void LS::two(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{i + 1}; j < n; j++) {
			if (solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 2)
				OP::twoSwap(solution.solution[i], solution.solution[j], flag);
		}
	}
}

void LS::one(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{0}; j < n; j++) {
			if (i != j && solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 1) {
				OP::oneMove(solution.solution[i], solution.solution[j], flag);
			}
		}
	}
}

void LS::three(Solution& solution, bool& flag) {
	for (u32 i{0}, n = solution.solution.size(); i < n; i++) {
		for (u32 j{0}; j < n; j++) {
			if (i != j && solution.solution[i].path.size() > 2 && solution.solution[j].path.size() > 3) {
				OP::threeSwap(solution.solution[i], solution.solution[j], flag);
			}
		}
	}
}

void LS::twoOpt(Solution& solution, bool& flag) {
	double dif{};  // 距离差值
	for (u32 s{0}, size = solution.solution.size(); s < size; s++) {
		if (solution.solution[s].path.size() <= 4) continue;
		for (u32 i{1}, n = solution.solution[s].path.size() - 3; i < n; i++) {
			for (u32 j{i + 2}, m = n + 2; j < m; j++) {
				if (CHK::reverse(solution.solution[s], i, j, dif)) {
					solution.solution[s].cumlength += dif;  // 更新距离
					flag = true;
				}
			}
		}
	}
}

void VNS::relocate(Solution& s, bool& flag) {
	Node* near{};
	std::vector<Node*> p;
	u32 tr{}, tp{};  // to_route;to_path;from_route;from_path
	double saving{};
	bool location{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 2) continue;
		p.assign(s.solution[fr].path.begin() + 1, s.solution[fr].path.end() - 1);
		for (u32 fp{0}, n = p.size(); fp < n; fp++) {
			for (u32 i{0}, m = p[fp]->distsort.size() * 0.2; i < m; i++) {
				near = p[fp]->distsort[i].toNode;
				tr = s.shash[near->seq];
				tp = CHK::find(s.solution[tr].path, near);
				if (OPS::onepointmove(s.solution[fr], s.solution[tr], fp, tp, saving, location)) {
					s.shash[p[fp]->seq] = tr;
					flag = 1;
					break;
				}
			}
		}
	}
	s.remove_void();
}

void VNS::twoopt(Solution& s, bool& flag) {
	double saving{};
	for (auto& r : s.solution) {
		if (r.path.size() < 5) continue;
		for (u32 i{1}, n = r.path.size() - 1; i + 2 < n; i++) {
			for (u32 j{i + 2}; j < n; j++) {
				if (OPS::reverse(r, i, j, saving)) flag = 1;
			}
		}
	}
}

void VNS::exchange(Solution& s, bool& flag) {
	Node* near{};
	std::vector<Node*> p;
	u32 tr{}, tp{};  // to_route;to_path;from_route;from_path
	double saving{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 2) continue;
		p.assign(s.solution[fr].path.begin() + 1, s.solution[fr].path.end() - 1);
		for (u32 fp{0}, n = p.size(); fp < n; fp++) {
			for (u32 i{0}, m = p[fp]->distsort.size() * 0.2; i < m; i++) {
				near = p[fp]->distsort[i].toNode;
				tr = s.shash[near->seq];
				tp = CHK::find(s.solution[tr].path, near);
				if (OPS::swapmove(s.solution[fr], s.solution[tr], fp, tp, saving)) {
					s.shash[p[fp]->seq] = tr;
					s.shash[near->seq] = fr;
					flag = 1;
					break;
				}
			}
		}
	}
}

void VNS::oropt2(Solution& s, bool& flag) {
	Node *p1{}, *p2{};
	std::set<u32> tabu;
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	double saving{};
	bool location{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 4) continue;
		u32 index{1};
		size = s.solution[fr].path[index]->depotsort.size() * 0.2;
		while (index < s.solution[fr].path.size() - 2) {
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			if (tabu.find((p1->seq << 10) + p2->seq) != tabu.end()) {
				index += 2;
				continue;
			}
			VNS::neighbor({p1, p2}, size, neighbors);
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				tr = s.shash[neighbors[i]->seq];
				tp = CHK::find(s.solution[tr].path, neighbors[i]->seq);
				fp = CHK::find(s.solution[fr].path, p1);
				if (tr == fr) {
					if (OPS::oropt(s.solution[fr], fp, tp, 2, saving, location)) {
						flag = 1;
						if (fp < tp) {
							tabu.emplace((p1->seq << 10) + p2->seq);
							break;
						}
					} else {
						if (OPS::oropt(s.solution[fr], s.solution[tr], fp, tp, 2, saving, location)) {
							flag = 1;
							break;
						}
					}
				}
				index++;
			}
			tabu.clear();
		}
	}
}

void VNS::arcnode(Solution& s, bool& flag) {
	Node *p1{}, *p2{};
	std::set<u32> tabu;
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	double saving{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 4) continue;
		u32 index{1};
		size = s.solution[fr].path[index]->depotsort.size() * 0.2;
		while (index < s.solution[fr].path.size() - 2) {
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			if (tabu.find((p1->seq << 10) + p2->seq) != tabu.end()) {
				index += 2;
				continue;
			}
			VNS::neighbor({p1, p2}, size, neighbors);
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				tr = s.shash[neighbors[i]->seq];
				tp = CHK::find(s.solution[tr].path, neighbors[i]->seq);
				fp = CHK::find(s.solution[fr].path, p1);
				if (tr == fr) {
					if (OPS::arcnode(s.solution[fr], s.solution[fr], fp, tp, saving)) {
						flag = 1;
						if (fp < tp)
							tabu.emplace((p1->seq << 10) + p2->seq);
						break;
					}
				} else {
					if (OPS::arcnode(s.solution[fr], s.solution[tr], fp, tp, saving)) {
						flag = 1;
						break;
					}
				}
			}
			index++;
		}
		tabu.clear();
	}
}

void VNS::oropt3(Solution& s, bool& flag) {
	Node *p1{}, *p2{}, *p3;
	std::set<u32> tabu;
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	double saving{};
	bool location{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 5) continue;
		u32 index{1};
		size = s.solution[fr].path[index]->depotsort.size() * 0.2;
		while (index < s.solution[fr].path.size() - 3) {
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			p3 = s.solution[fr].path[index + 2];
			if (tabu.find((p1->seq << 20) + (p2->seq << 10) + p3->seq) != tabu.end()) {
				index += 3;
				continue;
			}
			VNS::neighbor({p1, p2, p3}, size, neighbors);
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				tr = s.shash[neighbors[i]->seq];
				tp = CHK::find(s.solution[tr].path, neighbors[i]->seq);
				fp = CHK::find(s.solution[fr].path, p1);
				if (tr == fr) {
					if (OPS::oropt(s.solution[fr], fp, tp, 3, saving, location)) {
						flag = 1;
						if (fp < tp)
							tabu.emplace((p1->seq << 20) + (p2->seq << 10) + p3->seq);
						break;
					}
				} else {
					if (OPS::oropt(s.solution[fr], s.solution[tr], fp, tp, 3, saving, location)) {
						flag = 1;
						break;
					}
				}
			}
			index++;
		}
		tabu.clear();
	}
}

void VNS::oropt4(Solution& s, bool& flag) {
	Node *p1{}, *p2{}, *p3{}, *p4{};
	std::set<u32> tabu;
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	double saving{};
	bool location{};
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		if (s.solution[fr].path.size() <= 6) continue;
		u32 index{1};
		size = s.solution[fr].path[index]->depotsort.size() * 0.2;
		while (index < s.solution[fr].path.size() - 4) {
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			p3 = s.solution[fr].path[index + 2];
			p4 = s.solution[fr].path[index + 3];
			if (tabu.find((p1->seq << 20) + (p2->seq << 10)) != tabu.end() && tabu.find((p3->seq << 10) + p4->seq) != tabu.end()) {
				index += 4;
				continue;
			}
			VNS::neighbor({p1, p2, p3, p4}, size, neighbors);
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				tr = s.shash[neighbors[i]->seq];
				tp = CHK::find(s.solution[tr].path, neighbors[i]->seq);
				fp = CHK::find(s.solution[fr].path, p1);
				if (tr == fr) {
					if (OPS::oropt(s.solution[fr], fp, tp, 4, saving, location)) {
						flag = 1;
						if (fp < tp)
							tabu.emplace((p1->seq << 20) + (p2->seq << 10));
						tabu.emplace((p3->seq << 10) + p4->seq);
						break;
					}

				} else {
					if (OPS::oropt(s.solution[fr], s.solution[tr], fp, tp, 4, saving, location)) {
						flag = 1;
						break;
					}
				}
			}
			index++;
		}
		tabu.clear();
	}
}

/// @brief 计算节点的邻域
/// @param node  输入节点
/// @param size 大小参数
/// @param neighbor  存储邻域的节点
void VNS::neighbor(std::vector<Node*> node, u32 size, std::vector<Node*>& neighbor) {
	std::set<Node*> neighbor_set;
	if (node.size() == 2) {
		Node *n1{node[0]}, *n2{node[1]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode), neighbor_set.emplace(n2->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1 和 n2
		neighbor_set.erase(n1), neighbor_set.erase(n2);
		// 将邻居集合分配给邻居向量
		neighbor.assign(neighbor_set.begin(), neighbor_set.end());
		// 根据距离对邻居向量进行部分排序
		std::partial_sort(neighbor.begin(), neighbor.begin() + size, neighbor.end(), [n1, n2](Node* a, Node* b) {
			return a->dists[n1->seq].dist + a->dists[n2->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist;
		});
		// 调整邻居向量大小为 size
		neighbor.resize(size);
		return;
	}
	if (node.size() == 3) {
		Node *n1{node[0]}, *n2{node[1]}, *n3{node[1]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode);
			neighbor_set.emplace(n2->distsort[i].toNode);
			neighbor_set.emplace(n3->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1、n2 和 n3
		neighbor_set.erase(n1), neighbor_set.erase(n2), neighbor_set.erase(n3);
		// 将邻居集合分配给邻居向量
		neighbor.assign(neighbor_set.begin(), neighbor_set.end());
		// 根据距离对邻居向量进行部分排序
		std::partial_sort(neighbor.begin(), neighbor.begin() + size, neighbor.end(), [n1, n2, n3](Node* a, Node* b) {
			return a->dists[n1->seq].dist + a->dists[n2->seq].dist + a->dists[n3->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist + b->dists[n3->seq].dist;
		});
		// 调整邻居向量大小为 size
		neighbor.resize(size);
		return;
	}
	if (node.size() == 4) {
		Node *n1{node[0]}, *n2{node[1]}, *n3{node[1]}, *n4{node[1]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode), neighbor_set.emplace(n2->distsort[i].toNode);
			neighbor_set.emplace(n3->distsort[i].toNode), neighbor_set.emplace(n4->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1、n2、n3 和 n4
		neighbor_set.erase(n1), neighbor_set.erase(n2), neighbor_set.erase(n3), neighbor_set.erase(n4);
		// 将邻居集合分配给邻居向量
		neighbor.assign(neighbor_set.begin(), neighbor_set.end());
		// 根据距离对邻居向量进行部分排序
		std::partial_sort(neighbor.begin(), neighbor.begin() + size, neighbor.end(), [n1, n2, n3, n4](Node* a, Node* b) {
			return a->dists[n1->seq].dist + a->dists[n2->seq].dist + a->dists[n3->seq].dist + a->dists[n4->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist + b->dists[n3->seq].dist + b->dists[n4->seq].dist;
		});
		// 调整邻居向量大小为 size
		neighbor.resize(size);
		return;
	}
}