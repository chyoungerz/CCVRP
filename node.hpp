#pragma once

#ifndef _NODE_HPP_
#define _NODE_HPP_

// #include <eigen3/Eigen/Core>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

// 边长度，Node的一部分
struct Edge {
	double distance;  // 长度
	uint32_t to;      // 指向
};

class Node {
  protected:
	int x, y;             //(x, y)
  public:
	std::vector<Edge> distances;  // 该节点到其他节点的距离
	uint32_t seq;                 // 序号，从0开始
	uint32_t duration;        // 服务时间
	uint32_t demand;          // 需求
	uint32_t start;           // 开始时间窗
	uint32_t end;             // 结束时间窗
	//_seq序号， （x.axis, y.axis), start开始时间窗， end结束时间窗, duration 服务时间，demand需求
	Node();
	Node(const uint32_t seq, const int x_axis, const int y_axis, const uint32_t duration, const uint32_t demand, const uint32_t start, const uint32_t end);
	// 方便输出
	friend std::ostream& operator<<(std::ostream& _out, const Node _node);
	// Node operator+ (const Node& node_);
	Node& operator=(const Node&) = default;  // 允许赋值
	// 计算距离
	double cal_distance(const Node& node) const;
	// 计算距离（友元）
	friend double dist(const Node& axis_x, const Node& axis_y);
};

// 车辆或路线
class Vehicle {
  public:
	std::vector<uint32_t> path;        // 走过的路
	std::vector<double> diflength;     // 走过的路(时间）差分数组
	double length;                     // 所有节点的长度（时间）之和
	uint32_t locate;                   // location
	uint32_t load;                     // 载重量
	// 无参构造,默认0
	Vehicle();
	// loc是位置, lengthed 代表之前走过的长度（时间）
	Vehicle(const unsigned int loc, const double lengthed);
	// 移动到dest节点，并计算距离 (往返距离一样)
	bool move(const Node& dest /*,const Eigen::MatrixXf& dists*/);
	// 计算路径长度
	double path_length(/*const Eigen::MatrixXf& dists*/);
	// 清空, 并初始化位置
	void clear(uint32_t seq);
	// 方便输出
	friend std::ostream& operator<<(std::ostream& out, const Vehicle& car);
	Vehicle& operator=(const Vehicle&) = default;  // 允许赋值
};

// 厂站
class Station : public Node {
  public:
	Station();
	//(x_axis, y_axis) _seq 序号
	Station(const int x_axis, const int y_axis, const uint32_t _seq);
};

// 一个解
class Solution {
  public:
	std::vector<Vehicle> solution;
	// 总路径长度
	double allength{0.0};
	// 无参初始化
	Solution();
	// 添加一条路线（车辆）
	void add(const Vehicle& vehicle);
	void show();
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
*/
#endif