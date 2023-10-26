#pragma once

#include <algorithm>
#ifndef _NODE_HPP_
#define _NODE_HPP_

// #include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
// #include <memory>
#include <unordered_map>
#include <vector>

typedef unsigned int u32;
typedef unsigned long long int u64;

class Node;
// 边长度，Node的一部分
struct Edge {
	double dist;      // 长度
	u32 to;           // 指向
	const Node* toNode{nullptr};  // 指向的节点
};

/// @brief 节点
class Node {
  protected:
	double x, y;                  //(x, y)
	const Node* head{nullptr};    // 节点的前一节点
	const Node* tail{nullptr};    // 节点的后一节点

  public:
	std::vector<Edge> dists;     // 该节点到其他节点的距离(数组编号为节点序号)
	std::vector<Edge> distsort;  // 该节点到其他节点的距离按从小到大排序
	u32 seq;                     // 序号，从0开始
	u32 duration;                // 服务时间
	u32 demand;                  // 需求
	u32 start;                   // 开始时间窗
	u32 end;                     // 结束时间窗
	bool isdepot;                // 是否为厂站

	/**********构造函数************/
	//_seq序号， （x.axis, y.axis), start开始时间窗， end结束时间窗, duration 服务时间，demand需求
	Node() : x(0), y(0), seq(0), duration(0), demand(0), start(0), end(0) {}
	Node(const u32 _seq, const int x_axis, const int y_axis, const u32 _duration, const u32 _demand, const u32 _start, const u32 _end) {
		x = x_axis;
		y = y_axis;
		seq = _seq;
		duration = _duration;
		demand = _demand;
		start = _start;
		end = _end;
	}

	// 方便输出
	friend std::ostream& operator<<(std::ostream& _out, const Node _node) {
		_out << _node.seq << " : "
		     << "(" << _node.x << ", " << _node.y << ")"
		     << "\n";
		return _out;
	}
	// Node operator+ (const Node& node_);
	Node& operator=(const Node&) = default;  // 允许赋值
	bool operator!=(const Node& node_) {
		if (this->x == node_.x && this->y == node_.y) return false;
		return true;
	}
	Node& operator+=(const Node& node_) {
		this->x = node_.x + this->x;
		this->y = node_.y + this->y;
		return *this;
	}
	Node& operator/=(const double i) {
		this->x /= i;
		this->y /= i;
		return *this;
	}

	/***********成员函数**********/
	// 计算距离
	double cal_distance(const Node& node) const {
		double result = sqrt(pow((x - node.x), 2) + pow((y - node.y), 2));
		// dists.push_back(result);
		return result;
	}
	void clone(const Node* node) {
		this->x = node->x;
		this->y = node->y;
	}
	// 计算距离（友元）
	friend double dist(const Node& node_x, const Node& node_y) {
		return sqrt((node_x.x - node_y.x) * (node_x.x - node_y.x) + (node_x.y - node_y.y) * (node_x.y - node_y.y));
	}
	// 计算距离（友元）指针
	friend double dist(const Node* node_x, const Node* node_y) {
		return sqrt((node_x->x - node_y->x) * (node_x->x - node_y->x) + (node_x->y - node_y->y) * (node_x->y - node_y->y));
	}
	// 强制连接节点（友元） start -> end
	friend void link(const Node* start, const Node* end) {
		const_cast<Node*>(start)->tail = end;
	}
	// 强制连接节点（友元） start <-> end
	friend void connect(const Node* start, const Node* end) {
		const_cast<Node*>(start)->tail = end;
		const_cast<Node*>(end)->head = start;
	}
	// 交换节点（友元）
	friend void swap(const Node*& a, const Node*& b) {
		const_cast<Node*>(a)->tail = b->tail;
		const_cast<Node*>(b)->head = a->head;
		std::swap(a, b);
	}
};

// 车辆或路线
class Vehicle {
  public:
	std::vector<const Node*> path;  // 走过的路
	// double diflength;                  // 走过的路(时间）差分数组
	double cumlength;                  // 所有节点的长度（时间）之和
	double length;                     // 路径长度（时间）
	u32 capacity;                      // 最大容量
	u32 load;                          // 载重量
	u32 seq;                           // 路线序号
	const Node* depot;                 // 场站

	// 无参构造,默认0
	Vehicle() = delete;
	// loc是位置, max代表车的最大容量
	Vehicle(const Node* loc, const u32 maxload, const u32 seq_) : cumlength(0.0), capacity(maxload), load(0), seq(seq_), depot(loc) {
		path.reserve(100);
	}

	// 检查是否满足约束条件
	bool check(const Node* dest) {
		if (load + dest->demand > capacity)
			return false;
		else
			return true;
	}

	// 移动到dest节点，并计算距离 (往返距离一样)
	bool move(const Node* dest) {
		// double diflength{0.0};
		if (check(dest)) {
			load += dest->demand;
			path.emplace_back(dest);
			return true;
		}
		return false;
	}

	// 路径节点插入（position > 0)
	void emplace(const u32 pos, const Node* node) {
		connect(path[pos], node);      // pos <-> node
		connect(path[pos - 1], node);  // pos - 1 <-> node
		path.emplace(path.begin() + pos, node);
	}

	// 路径节点删除（position > 0)
	const Node* erase(const u32 pos) {
		const Node* node{path[pos]};
		link(path[pos - 1], path[pos + 1]);  // pos - 1 <-> pos + 1
		path.erase(path.begin() + pos);
		return node;
	}

	const Node* remove(const u32 pos) {
		const Node* node{path[pos]};
		path.erase(path.begin() + pos);
		return node;
	}

	/// @brief 计算路径长度
	/// @param update 是否更新，默认否
	/// @return 路径长度
	double path_length(bool update = false) {
		if (path.empty()) return 0.0;
		double _length{0.0};
		for (u64 j{0}, n{path.size() - 1}; j < n; j++) {
			_length += path[j]->dists[path[j + 1]->seq].dist;
		}
		if (update) length = _length;
		return _length;
	}

	/// @brief 计算累计路径长度
	/// @param update 是否更新，默认否
	/// @return 累计路径长度
	double path_cumlength(bool update = false) {
		if (path.empty()) return 0.0;
		double _length{depot->dists[path.front()->seq].dist * path.size()};
		for (u64 j{0}, n{path.size() - 1}; j < n; j++) {
			_length += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
		}
		if (update) cumlength = _length;
		return _length;
	}

	// 连接路径
	void connect_path() {
		for (u64 j{1}, n{path.size() - 2}; j < n; j++) {
			connect(path[j], path[j + 1]);
		}
	}

	/// @brief 清空
	/// @param node0 初始化位置
	/// @param seq_ 路线序号
	void clear(const Node* node0, u32 seq_) {
		load = 0;
		cumlength = 0.0;
		seq = seq_;
		depot = node0;
		path.clear();
	}

	// 方便输出
	friend std::ostream& operator<<(std::ostream& out, const Vehicle& car) {
		out << "length : " << car.cumlength << "  ";
		for (unsigned int j{0}, n = car.path.size() - 1; j < n; j++) {
			out << car.path[j]->seq << "-";
		}
		out << car.path.back()->seq;
		return out;
	}

	Vehicle& operator=(const Vehicle&) = default;  // 允许赋值
};

// 一个解
class Solution {
  public:
	std::vector<Vehicle> solution;                 // 解决方案
	std::unordered_map<u32, u32> shash;            // hash查找表，key为节点序号，value为所在路线
	double allength{0.0};                          // 总路径长度
	double limit{-1.0};                            // 最大路径长度
	bool multi{false};                             // 算法多场站
	bool valid{false};                             // 是否可行

	// 无参初始化
	Solution() {
		allength = 0.0;
		solution.reserve(5);
	}

	// double length() const { return allength; }

	// 添加一条路线（车辆）
	void add(const Vehicle& vehicle) {
		solution.emplace_back(vehicle);
	}

	// 添加多条路线（车辆）
	void add(const Vehicle& vehicle, const u32 num) {
		solution.insert(solution.end(), num, vehicle);
	}

	void show() {
		u32 num{}, routes{};
		for (u32 i = 0, n = solution.size(); i < n; i++) {
			if (solution[i].path.empty()) continue;
			num += solution[i].path.size();
			std::cout << solution[i] << " : " << solution[i].path.size() << std::endl;
			routes++;
		}
		std::cout << "total length: " << allength << "\ttotal customers: " << num << "\ttotal routes: " << routes << std::endl;
	}

	/// @brief 更新累计路径长度
	/// @param update 是否重新计算，1:全部重新计算，0：不重新计算（默认）
	void update(u32 update = 0) {
		allength = 0.0;
		if (update == 0) {
			for (auto& i : solution) {
				allength += i.cumlength;
			}
		} else {
			for (auto& i : solution) {
				allength += i.path_cumlength(true);
			}
		}
	}
	// 更新序号
	void update_seq() {
		for (u32 i{0}, n = solution.size(); i < n; i++) {
			solution[i].seq = i;
		}
	}

	/// @brief 清空哈希表
	/// @param update 是否更新，默认否
	void update_hash(bool update = false) {
		shash.clear();
		if (update) {
			for (u32 s{0}, n = solution.size(); s < n; s++) {
				solution[s].seq = s;
				if (solution[s].path.empty()) continue;
				for (auto& i : solution[s].path) {
					shash.emplace(i->seq, s);
				}
			}
		}
	}

	/// @brief 删除空路径
	void remove_void() {
		solution.erase(std::remove_if(solution.begin(), solution.end(), [](const Vehicle& v) { return v.path.empty(); }), solution.end());
	}

	///@brief 约束目标
	bool evaluate() {
		valid = true;
		for (auto& s : solution) {
			s.load = 0;
			for (u32 i{0}, n = s.path.size() - 1; i < n; i++) {  // 优先级
				s.load += s.path[i]->demand;
				if (s.path[i]->end > s.path[i + 1]->end) {
					valid = false;
				}
			}
			if (s.load > s.capacity) {  // 容量
				valid = false;
			}
		}
		return valid;
	}

	void customer() {
		u32 num{};
		for (u32 i = 0, n = solution.size(); i < n; i++) {
			num += solution[i].path.size() - 2;
		}
		std::cout << "total customers: " << num << std::endl;
	}

	// 删除路径节点
	const Node* erase(const u32 where, const u32 pos) {
		const Node* node{solution[where].remove(pos)};
		if (shash.contains(node->seq)) {
			shash.erase(node->seq);
		}
		return node;
	}
};

/*
class Ant {
  protected:
	unsigned int locate;  // location
	// std::unordered_map<int, int>
	bool end;  // wether to end
  public:
	std::vector<unsigned int> select;
	std::vector<unsigned int> path;  // 走过的路
	unsigned int timenow;            // 当前时间
	double length;                    // 走过的路长度
	// loc 位置（从1开始），sel 全部蚂蚁便于初始化可选路径
	Ant(const unsigned int loc, const unsigned int sel);
	Ant();
	// bool walk(const int node_seq);
	void move(const Eigen::MatrixXf pheromates, const Eigen::MatrixXf& dists);
	void move(const unsigned int loc);
	unsigned int location() const;
	double path_length(const Eigen::MatrixXf& dists);
	void remove_select_path(const unsigned int rm);
	// const std::unordered_map<int, int> walked_path();
	friend std::ostream& operator<<(std::ostream& _out, const Ant& _ant);
	bool end_path() const;
	// friend double dist(const Node& node_x, const Node& node_y,
	// std::map<int, double>& stored);
};

//============================= class Ant start =============================

Ant::Ant(const unsigned int loc, const unsigned int sel) : locate(loc), end(false), timenow(0) {
	for (unsigned int i = 1; i < sel; i++) {  // init select paths
		select.push_back(i);
	}
	path.push_back(loc);
}
Ant::Ant(): locate(0), end(false), timenow(0) {
	path.push_back(0);
}
bool Ant::walk(const int node_seq) {
	if (path.find(node_seq) == path.end()) {
	path.insert(std::pair<int, int>(node_seq, node_seq));
	locate = node_seq;
	return true;
	} else {
		return false;
	}
};
void Ant::move(const Eigen::MatrixXf pheromates, const Eigen::MatrixXf& dists) {
	unsigned int temp = discrete_distribute(select, locate, pheromates, dists); //
	path.push_back(select[temp]);
	locate = select[temp];
	std::swap(select[temp], select.back());
	select.pop_back();  //快速删除
	if (select.empty()) {
		end = true;
	}
}
void Ant::move(const unsigned int loc) {
	path.push_back(loc);
	locate = loc;
	if (select.empty()) {
		end = true;
	}
}
unsigned int Ant::location() const { return locate;}
double Ant::path_length(const Eigen::MatrixXf& dists) {
	for (unsigned int j = 0; j + 1 < path.size(); j++) {
		length += dists(path[j], path[j + 1]);
	}
	return length;
}
void Ant::remove_select_path(const unsigned int rm) {
	std::swap(select[rm], select.back());
	select.pop_back();
	if (select.empty()) {
		end = true;
	}
}

std::vector<int>& Ant::select_path() {
	return select;
}
const std::unordered_map<int, int> Ant::walked_path() {
	return path;
}
std::ostream& operator<< (std::ostream& _out, const Ant& _ant) {
	for (unsigned int j = 0; j < _ant.path.size(); j++) {
		_out << _ant.path[j] << "-";
	}
	return _out;
}
bool Ant::end_path() const {
	return end;
}
//double dist(const Node& node_x, const Node& node_y, std::map<int, double>& stored) {}

//============================= class Ant end =============================
*/
#endif