#pragma once

#ifndef _NODE_HPP_
#define _NODE_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
// #include <unordered_map>
#include <eigen3/Eigen/Core>
// #include <memory>

/*class Axis {
	protected:
		int x, y; //(x, y)
	public:
		Axis();
		Axis(const int temp_x, const int temp_y);
		void init(const int& temp_x, const int& temp_y);
		void init(const Axis& temp_axis);
		friend std::ostream& operator<< (std::ostream& _out, const Axis& _axis);
		friend std::istream& operator>> (std::istream& _in, Axis& _axis);
		Axis operator+ (const Axis& axis_);
		Axis operator= (const Axis& axis_);
		friend double dist(const Axis& axis_x, const Axis& axis_y);
};*/

class Node {
  protected:
	int x, y;             //(x, y)
  public:
	uint32_t seq;             // 序号，从1开始, 0 为厂站
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
	float cal_distance(const Node& node) const;
	// 计算距离（友元）
	friend float dist(const Node& axis_x, const Node& axis_y);
};

// 车辆或路线
class Vehicle {
  public:
	std::vector<unsigned int> path;    // 走过的路
	float length;                      // 走过的路长度
	uint32_t locate;                   // location
	uint32_t load;                     // 载重量
	bool end{false};                   // wether to end
	// 无参构造,默认0
	Vehicle();
	// loc是位置
	Vehicle(const unsigned int loc);
	// 移动到dest节点，并计算距离
	bool move(const Node& dest, const Eigen::MatrixXf& dists);
	// 计算路径长度
	float path_length(const Eigen::MatrixXf& dists);
	// 方便输出
	friend std::ostream& operator<<(std::ostream& _out, const Vehicle& _car);
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
	float allength{0.0};
	// 无参初始化
	Solution();
	// 添加一条路线（车辆）
	void add(const Vehicle& vehicle);
};

class Ant {
  protected:
	unsigned int locate;  // location
	// std::unordered_map<int, int>
	bool end;  // wether to end
  public:
	std::vector<unsigned int> select;
	std::vector<unsigned int> path;  // 走过的路
	unsigned int timenow;            // 当前时间
	float length;                    // 走过的路长度
	// loc 位置（从1开始），sel 全部蚂蚁便于初始化可选路径
	Ant(const unsigned int loc, const unsigned int sel);
	Ant();
	// bool walk(const int node_seq);
	void move(const Eigen::MatrixXf pheromates, const Eigen::MatrixXf& dists);
	void move(const unsigned int loc);
	unsigned int location() const;
	float path_length(const Eigen::MatrixXf& dists);
	void remove_select_path(const unsigned int rm);
	// const std::unordered_map<int, int> walked_path();
	friend std::ostream& operator<<(std::ostream& _out, const Ant& _ant);
	bool end_path() const;
	// friend double dist(const Node& node_x, const Node& node_y,
	// std::map<int, double>& stored);
};

#endif