#pragma once

#ifndef _NODE_HPP_
#define _NODE_HPP_

// #include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
// #include <memory>
#include <unordered_map>
#include <vector>

typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;

class Node;
// 边长度，Node的一部分
struct Edge {
	double dist;      // 长度
	uint32_t to;      // 指向
	const Node* toNode{nullptr};  // 指向的节点
};

class Node {
  protected:
	double x, y;                  //(x, y)
  public:
	std::vector<Edge> dists;      // 该节点到其他节点的距离(数组编号为节点序号)
	std::vector<Edge> distsort;   // 该节点到其他节点的距离按从小到大排序
	uint32_t seq;                 // 序号，从0开始
	uint32_t duration;        // 服务时间
	uint32_t demand;          // 需求
	uint32_t start;           // 开始时间窗
	uint32_t end;             // 结束时间窗

	/**********构造函数************/
	//_seq序号， （x.axis, y.axis), start开始时间窗， end结束时间窗, duration 服务时间，demand需求
	Node() : x(0), y(0), seq(0), duration(0), demand(0), start(0), end(0) {}
	Node(const uint32_t _seq, const int x_axis, const int y_axis, const uint32_t _duration, const uint32_t _demand, const uint32_t _start, const uint32_t _end) {
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
			 << "(" << _node.x << ", " << _node.y << ")" << std::endl;
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
};

// 车辆或路线
class Vehicle {
  public:
	std::vector<const Node*> path;  // 走过的路
	// double diflength;                  // 走过的路(时间）差分数组
	double cumlength;                  // 所有节点的长度（时间）之和
	uint32_t capacity;                 // 最大容量
	uint32_t load;                     // 载重量
	uint32_t seq;                      // 路线序号

	// 无参构造,默认0
	Vehicle() = delete;
	// loc是位置, max代表车的最大容量
	Vehicle(const Node* loc, const uint32_t maxload, const uint32_t seq_) : cumlength(0.0), capacity(maxload), load(0), seq(seq_) {
		path.reserve(100);
		path.emplace_back(loc);
	}

	// 移动到dest节点，并计算距离 (往返距离一样)
	bool move(const Node* dest /*,const Eigen::MatrixXf& dists*/) {
		// double diflength{0.0};
		if ((load + dest->demand) > capacity) return false;
		load += dest->demand;
		path.emplace_back(dest);
		/*for (uint32_t i = 0; i + 1 < path.size(); i++) {
			diflength += path[i]->distances[path[i + 1]->seq].distance;
		}
		cumlength += diflength;*/
		// diflength.push_back(diflength.back() + dists(locate, dest.seq));
		// diflength.push_back(diflength.back() + dest->distances[locate].distance);  // from a to b == from b to a
		return true;
	}

	// 计算路径长度
	double path_length(/*const Eigen::MatrixXf& dists*/) {
		double length{0.0};
		for (uint64_t j{0}, n{path.size() - 2}; j < n; j++) {
			length += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
		}
		return length;
	}

	// 更新路径长度
	void update_length(/*const Eigen::MatrixXf& dists*/) {
		cumlength = 0.0;
		for (uint64_t j{0}, n{path.size() - 2}; j < n; j++) {
			cumlength += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
		}
	}

	// 清空, 并初始化位置
	void clear(const Node* node0, uint32_t seq_) {
		load = 0;
		cumlength = 0.0;
		seq = seq_;
		path.clear();
		path.emplace_back(node0);
	}

	// 方便输出
	friend std::ostream& operator<<(std::ostream& out, const Vehicle& car) {
		out << "length : " << car.cumlength << "  ";
		for (unsigned int j = 0; j < car.path.size(); j++) {
			out << car.path[j]->seq << "-";
		}
		return out;
	}

	Vehicle& operator=(const Vehicle&) = default;  // 允许赋值
};

// 厂站
class Station : public Node {
  public:
	Station() {
		x = 0;
		y = 0;
		seq = 0;
		start = 0;
		end = 0;
	}
	//(x_axis, y_axis) _seq 序号
	Station(const int x_axis, const int y_axis, const uint32_t _seq) {
		x = x_axis;
		y = y_axis;
		seq = _seq;
		start = 0;
		end = 0;
	}
};

// 一个解
class Solution {
  public:
	std::vector<Vehicle> solution;
	// 总路径长度
	double allength{0.0};
	// hash查找表，key为节点序号，value为所在路线
	std::unordered_map<uint32_t, uint32_t> shash;

	// 无参初始化
	Solution() {
		allength = 0.0;
		solution.reserve(5);
	}

	// 添加一条路线（车辆）
	void add(const Vehicle& vehicle) {
		solution.emplace_back(vehicle);
		allength += vehicle.cumlength;
	}

	void show() {
		uint32_t num{};
		uint32_t routes = solution.size();
		for (uint32_t i = 0; i < routes; i++) {
			std::cout << solution[i] << ":" << solution[i].path.size() - 2 << std::endl;
			num += solution[i].path.size() - 2;
		}
		std::cout << "total length: " << allength << "\ttotal customers: " << num << "\ttotal routes: " << routes << std::endl;
	}

	void update() {
		allength = 0.0;
		for (auto& i : solution) {
			i.cumlength = i.path_length();
			allength += i.cumlength;
		}
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