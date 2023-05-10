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

class Node;                      // 前向声明
extern std::vector<Node> nodes;  // 节点(全局变量)
extern Eigen::MatrixXf dists;    // 距离矩阵(全局变量)

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
	// 返回序号，从1开始
	unsigned int get_seq() const;
	// 计算距离
	float cal_distance(const Node& node) const;
	// 计算距离（友元）
	friend float dist(const Node& axis_x, const Node& axis_y);
};

// 车辆
class Vehicle {
  public:
	unsigned int locate;               // location
	bool end{false};                   // wether to end
	std::vector<unsigned int> path;    // 走过的路
	std::vector<unsigned int> select;  // 寻找到的路径序列号
	float length;                      // 走过的路长度
	// 禁止无参构造
	Vehicle() = delete;
	// loc是位置，_seq为了初始化select
	Vehicle(const unsigned int loc, const unsigned int _seq);
	void move(const unsigned int dest_seq);
	// 返回当前位置
	unsigned int location() const;
	// 计算路径长度
	float path_length(const Eigen::MatrixXf& dists);
	// 方便输出
	friend std::ostream& operator<<(std::ostream& _out, const Vehicle& _car);
	Vehicle& operator=(const Vehicle&) = default;  // 允许赋值
};

// 厂站
class Station : public Node {
  public:
	Station() = delete;
	Station(const int x_axis, const int y_axis, const unsigned int _seq, const unsigned int _start, const unsigned int _end);
};

// 路线
class Tour {
  public:
	std::vector<Vehicle> Tours;
	// 总路径长度
	float length;
	// 禁止无参构造
	Tour() = delete;
	//
	Tour(const std::vector<Vehicle>);
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