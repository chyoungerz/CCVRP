#pragma once

// #include <algorithm>
#ifndef _NODE_HPP_
#define _NODE_HPP_

// #include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#define FORCE 1
#define ROUTES 2
#define LOADS 4
#define LENGTH 8
#define PRIORITY 16

typedef unsigned int u32;
typedef unsigned long long int u64;
typedef long long int i64;

class Node;
// 边长度，Node的一部分
struct Edge {
	double dist;            // 长度
	Node* toNode{nullptr};  // 指向的节点
	u32 to;                 // 指向
};

struct Info {
	u32 one{};
	u32 two{};
	u32 three{};
	u32 or2{};
	u32 or3{};
	u32 or4{};
	u32 arc{};
	u32 opt2{};
};

// 目标
inline double v_aim(double cumlength_) {
	return cumlength_;
}

/// @brief 节点
class Node {
  protected:
	double x, y;  //(x, y)

  public:
	std::vector<Edge> dists;     // 该节点到其他节点的距离(数组编号为节点序号)
	std::vector<Edge> distsort;  // 该节点到其他节点的距离按从小到大排序(不包括厂站)
	std::vector<Edge> depotsort;  // 该节点到厂站的距离按从小到大排序
	u32 seq;                     // 序号，从0开始
	u32 duration;                // 服务时间
	u32 demand;                  // 需求
	u32 start;                   // 开始时间窗
	u32 end;                     // 结束时间窗(优先级，场站为最高)
	bool isdepot;                // 是否为厂站

	/**********构造函数************/
	//_seq序号， （x.axis, y.axis), start开始时间窗， end结束时间窗, duration 服务时间，demand需求
	Node() : x(0), y(0), seq(0), duration(0), demand(0), start(0), end(0), isdepot(0) {}
	Node(const u32 _seq, const int x_axis, const int y_axis, const u32 _duration, const u32 _demand, const u32 _start, const u32 _end) {
		x = x_axis;
		y = y_axis;
		seq = _seq;
		duration = _duration;
		demand = _demand;
		start = _start;
		end = _end;
		isdepot = false;
	}

	// 方便输出
	friend std::ostream& operator<<(std::ostream& _out, Node _node) {
		_out << _node.seq << " : "
		     << "(" << _node.x << ", " << _node.y << ")"
		     << "\n";
		return _out;
	}
	// Node operator+ (Node& node_);
	Node& operator=(Node&) = default;  // 允许赋值
	bool operator!=(Node& node_) {
		return this->x == node_.x && this->y == node_.y;
	}
	Node& operator+=(Node& node_) {
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
	double cal_distance(Node& node) const {
		double result = sqrt(pow((x - node.x), 2) + pow((y - node.y), 2));
		// dists.push_back(result);
		return result;
	}
	void clone(Node* node) {
		this->x = node->x;
		this->y = node->y;
	}
	void set(const double x_, const double y_) {
		x = x_;
		y = y_;
	}
	// 计算距离（友元）
	friend const double dist(const Node& node_x, const Node& node_y) {
		return sqrt((node_x.x - node_y.x) * (node_x.x - node_y.x) + (node_x.y - node_y.y) * (node_x.y - node_y.y));
	}
	// 计算距离（友元）指针
	friend const double dist(const Node* node_x, const Node* node_y) {
		return sqrt((node_x->x - node_y->x) * (node_x->x - node_y->x) + (node_x->y - node_y->y) * (node_x->y - node_y->y));
	}
	// 计算距离（友元）指针
	friend const double disti(const Node* node_x, const Node* node_y) {
		return std::round(sqrt((node_x->x - node_y->x) * (node_x->x - node_y->x) + (node_x->y - node_y->y) * (node_x->y - node_y->y)));
	}
};

// 车辆或路线
class Vehicle {
  public:
	std::vector<Node*> path;  // 走过的路
	// double diflength;                  // 走过的路(时间）差分数组
	double cumlength;                  // 所有节点的长度（时间）累计和
	double length{};                   // 路径长度（时间）
	double Limit{100000.0};            // 最大路径长度（时间）
	Node* depot{nullptr};              // 场站
	u32 capacity;                      // 最大容量
	u32 load;                          // 载重量
	u32 seq;                           // 路线序号

	// 无参构造,默认0
	Vehicle() = delete;
	// loc是位置, max代表车的最大容量
	Vehicle(Node* loc, const u32 maxload, const u32 seq_) : cumlength(0.0), depot(loc), capacity(maxload), load(0), seq(seq_) {
		path.reserve(100);
		path.emplace_back(loc);
	}
	Vehicle(Node* loc, const u32 maxload) : cumlength(0.0), depot(loc), capacity(maxload), load(0), seq(0) {
		path.reserve(100);
		path.emplace_back(loc);
		path.emplace_back(loc);
	}

	// 检查是否满足约束条件
	bool check(Node* dest) {
		if (load + dest->demand > capacity)
			return false;
		else
			return true;
	}

	// 移动到dest节点，并计算距离 (往返距离一样)
	bool move(Node* dest) {
		// double diflength{0.0};
		if (check(dest)) {
			load += dest->demand;
			path.emplace_back(dest);
			return true;
		}
		return false;
	}

	/// @brief 计算路径长度
	/// @param update 是否更新，默认否
	/// @return 路径长度
	double path_length(bool update = false) {
		if (path.empty()) return 0.0;
		double _length{0.0};
		for (u64 j{0}, n{path.size() - 2}; j < n; j++) {
			_length += path[j]->dists[path[j + 1]->seq].dist;
		}
		if (update) length = _length;
		return _length;
	}

	/// @brief 计算累计路径长度
	/// @param update 是否更新，默认否
	/// @return 累计路径长度
	double path_cumlength(bool update = false) {
		double _length{0.0};
		for (u64 j{0}, n{path.size() - 2}; j < n; j++) {
			_length += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
		}
		if (update) cumlength = _length;
		return _length;
	}

	/// @brief 更新所有路径长度
	void update_allength() {
		length = cumlength = 0.0;
		for (u64 j{0}, n{path.size() - 2}; j < n; j++) {
			length += path[j]->dists[path[j + 1]->seq].dist;
			cumlength += length;
		}
	}

	bool evaluate(double& cumlength_, double& length_, const u32 ctrl) {
		// double length_{};
		u32 load_{};
		for (u32 i{1}, n = path.size() - 1; i < n; i++) {  // 优先级
#ifdef DEBUG
			if (path[i]->isdepot) throw "路径含有场站";
#endif
			length_ += path[i]->dists[path[i - 1]->seq].dist;
			cumlength_ += length_;
			load_ += path[i]->demand;
			// cumlength_ += path[i]->dists[path[i - 1]->seq].dist * (n - i);
		}
		if (!ctrl) {
			if (length_ > Limit || load_ > capacity) return false;
		}
		return true;
	}

	bool valid() {
		if (length > Limit || load > capacity) {
			return false;
		}
		return true;
	}

	void precheck(u32& ctrl) {
		if (length > Limit) ctrl += LENGTH;
		if (load > capacity) ctrl += LOADS;
	}

	/// @brief 清空
	/// @param node0 初始化位置
	/// @param seq_ 路线序号
	void clear(Node* node0, u32 seq_) {
		load = 0;
		cumlength = 0.0;
		seq = seq_;
		depot = node0;
		path.clear();
		path.emplace_back(node0);
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
	double Limit{10000000.0};                      // 最大路径长度
	u32 maxvehicle{};                              // 最大车辆
	bool multi{false};                             // 是否多场站
	bool valid{false};                             // 是否可行

	// 无参初始化
	Solution() {
		allength = 0.0;
		solution.reserve(20);
		shash.reserve(256);
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
			num += solution[i].path.size() - 2;
			std::cout << solution[i] << " : " << solution[i].load << std::endl;
			routes++;
		}
		std::cout << "total length: " << allength << "\ttotal customers: " << num << "\ttotal routes: " << routes << std::endl;
	}

	/// @brief 更新累计路径长度
	/// @param update 是否重新计算，1:全部重新计算，0：不重新计算（默认）
	void update(u32 update = 0) {
		allength = 0.0;
		valid = true;
		if (update == 0) {
			for (auto& i : solution) {
				allength += i.cumlength;
				if (i.load > i.capacity || i.length > i.Limit) valid = false;
			}
		} else {
			for (auto& i : solution) {
				allength += i.path_cumlength(true);
				if (i.load > i.capacity || i.length > i.Limit) valid = false;
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
		bool flag{0};
		if (multi) {  // 多场站
			while (solution.size() != maxvehicle && flag) {
				flag = 0;
			}
			if (solution.size() != maxvehicle && flag) {
				update_seq();
			}
		} else {
			if (solution.size() != maxvehicle) {
				if (solution.size() > maxvehicle) {
					int v = solution.size() - maxvehicle;
					for (auto it = solution.begin(); it != solution.end(); it++) {
						if (it->path.size() <= 2) {
							solution.erase(it);
							v--;
							if (v <= 0)
								break;
						}
					}
				} else {
					solution.resize(maxvehicle, Vehicle(solution.front().depot, 0));
					flag = 1;
				}
			}
			if (flag) {
				update_seq();
				// update_hash(1);
			}
		}
	}

	///@brief 约束目标
	bool evaluate(u32& ctrl) {
		valid = true;
		if (solution.size() > maxvehicle) {
			valid = false;
			ctrl += ROUTES;
		}
		bool bpriority{0}, bloads{0}, blength{0};
		for (auto& s : solution) {
			s.load = 0;
			s.cumlength = 0.0;
			s.length = 0.0;
			for (u32 i{1}, n = s.path.size() - 1; i < n; i++) {  // 优先级
				s.load += s.path[i]->demand;
				s.length += s.path[i - 1]->dists[s.path[i]->seq].dist;
				s.cumlength += s.length;
				if (s.path[i - 1]->end > s.path[i]->end) {
					bpriority = true;
					valid = false;
				}
			}
			if (s.length > Limit) {  // 路径长度
				valid = false;
				blength = true;
			}
			if (s.load > s.capacity) {  // 容量
				valid = false;
				bloads = true;
			}
		}
		if (blength)  // 路径长度
			ctrl += LENGTH;
		if (bloads)  // 容量
			ctrl += LOADS;
		if (bpriority)  // 优先级
			ctrl += PRIORITY;
		return valid;
	}
	void evaluate() {
		valid = true;
		if (solution.size() > maxvehicle) {
			valid = false;
		}
		for (auto& s : solution) {
			if (!s.valid()) {  // 路径长度
				valid = false;
				return;
			}
		}
	}

	void customer() {
		u32 num{};
		for (u32 i = 0, n = solution.size(); i < n; i++) {
			num += solution[i].path.size() - 2;
		}
		std::cout << "total customers: " << num << std::endl;
	}

	void debug(bool verb = false) {
		u32 num{};
		for (auto& s : solution) {
			u32 load_{};
			double length_{}, cumlength_{};
			for (u32 i{1}, n = s.path.size() - 1; i < n; i++) {  // 优先级
				load_ += s.path[i]->demand;
				length_ += s.path[i - 1]->dists[s.path[i]->seq].dist;
				cumlength_ += length_;
				num++;
			}
			if (load_ != s.load || (cumlength_ - s.cumlength) > 0.01) {
				std::cout << s.seq << " load: " << load_ << " " << s.load << "\n";
				std::cout << s.seq << " cumlength: " << cumlength_ << " " << s.cumlength << "\n";
			}
		}
		if (verb)
			std::cout << num << "\n";
	}
	void debug_hash(u32 where) {
		for (auto& s : solution) {
			for (u32 i{1}, n = s.path.size() - 1; i < n; i++) {  // 优先级
				if (shash.contains(s.path[i]->seq)) {
					if (shash[s.path[i]->seq] != s.seq) {
						std::cout << where << " hash error: " << s.path[i]->seq << " in " << s.seq << "\n";
					}
				} else {
					std::cout << where << " hash lost: " << s.path[i]->seq << " in " << s.seq << "\n";
				}
			}
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
    // friend double dist(Node& node_x, Node& node_y,
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
//double dist(Node& node_x, Node& node_y, std::map<int, double>& stored) {}

//============================= class Ant end =============================
*/
#endif