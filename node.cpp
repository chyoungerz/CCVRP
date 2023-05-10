
#include "node.hpp"
#include "distribution.hpp"

//============================= class Axis start =============================
/*
Axis::Axis(): x(0), y(0) {}
Axis::Axis(const int temp_x, const int temp_y): x(temp_x), y(temp_y) {}
void Axis::init(const int& temp_x, const int& temp_y) {
    x = temp_x;
    y = temp_y;
};
void Axis::init(const Axis& temp_axis) {
    x = temp_axis.x;
    y = temp_axis.y;
};
std::ostream& operator<< (std::ostream& _out, const Axis& _axis) {
    _out << "(" << _axis.x << ", " << _axis.y << ")";
    return _out;
};
std::istream& operator>> (std::istream& _in, Axis& _axis) {
    _in >> _axis.x >> _axis.y;
    return _in;
}; 
Axis Axis::operator+ (const Axis& axis_) {
    Axis _axis;
    _axis.x = x + axis_.x, _axis.y = y + axis_.y;
    return _axis;
};
Axis Axis::operator= (const Axis& axis_) {
    Axis _axis;
    _axis.x = axis_.x, _axis.y = axis_.y;
    return _axis;
};
double dist(const Axis& axis_x, const Axis& axis_y) {
    return sqrt(pow((axis_x.x - axis_y.x),2) + pow((axis_x.y - axis_y.y),2));
};
*/
//============================= class Axis end =============================


//============================= class Node start =============================
Node::Node() : x(0), y(0), seq(0), duration(0), demand(0), start(0), end(0) {}
Node::Node(const uint32_t _seq, const int x_axis, const int y_axis, const uint32_t _duration, const uint32_t _demand, const uint32_t _start, const uint32_t _end) {
	x = x_axis;
	y = y_axis;
	seq = _seq;
	duration = _duration;
	demand = _demand;
	start = _start;
	end = _end;
}
std::ostream& operator<< (std::ostream& _out, const Node _node) {
    _out << _node.seq << " : " << "(" << _node.x << ", " << _node.y << ")" << std::endl;
    return  _out;
};
/*Node Node::operator+ (const Node& node_) {
	Node _node;
	_node.x = x + node_.x;
	_node.y = y + node_.y;
	_node.seq = seq + node_.seq;
	_node.start_time = start_time + node_.start_time;
	_node.end_time = end_time + node_.end_time;
	return _node;
};
Node& Node::operator= (const Node& node_) {
	this -> x = node_.x;
	this -> y = node_.y;
	this -> seq = node_.seq;
	this -> start_time = start_time;
	this -> end_time = end_time;
	return *this;
};*/
float Node::cal_distance(const Node& node) const{
    float result = sqrt(pow((x - node.x),2) + pow((y - node.y),2));
    //dists.push_back(result);
    return result;
};
float dist(const Node& node_x, const Node& node_y) {
    return sqrt(pow((node_x.x - node_y.x),2) + pow((node_x.y - node_y.y),2));
};

//============================= class Node end =============================

//============================= class Station public Node start =============================

Station::Station(const int x_axis, const int y_axis, const unsigned int _seq, const unsigned int _start, const unsigned int _end) {
	x = x_axis;
	y = y_axis;
	seq = _seq;
	start = _start;
	end = _end;
} /*
 Station Station::operator= (const Station& station_) {
	 Station _station;
	 _station.x = station_.x;
	 _station.y = station_.y;
	 _station.seq = station_.seq;
	 _station.start_time = station_.start_time;
	 _station.end_time = station_.end_time;
	 _station.port = station_.port;
	 return _station;
 };*/

//============================= class Station public Node end =============================


//============================= class Vehicle public Node start =============================
Vehicle::Vehicle(const unsigned int loc, const unsigned int _sel) : locate(loc), length(0.0) {
	path.reserve(_sel); //提前分配好大小
    select.reserve(_sel);
    path.push_back(loc);
    for (unsigned int i = 1; i <= _sel; i++) { //init select paths
        select.push_back(i);
    }
}
/*bool Ant::walk(const int node_seq) {
    if (path.find(node_seq) == path.end()) {
    path.insert(std::pair<int, int>(node_seq, node_seq));
    locate = node_seq;
    return true;
    } else {
        return false;
    }
};*/


unsigned int Vehicle::location() const{ return locate;}
float Vehicle::path_length(const Eigen::MatrixXf& dists) {
    for (unsigned int j = 0; j + 1 < path.size(); j++) {
        length += dists(path[j], path[j + 1]);
    }
    return length;
}

/*const std::unordered_map<int, int> Ant::walked_path() {
    return path;
}*/
std::ostream& operator<< (std::ostream& _out, const Vehicle& _ant) {
    for (unsigned int j = 0; j < _ant.path.size(); j++) {
        _out << _ant.path[j] << "-";
    }
    return _out;
}
/*Vehicle Vehicle::operator= (const Vehicle& vohicle_) {
	Vehicle _vohicle;
	_vohicle.soc = vohicle_.soc;
	_vohicle.locate = vohicle_.locate;
	_vohicle.length = vohicle_.length;
	_vohicle.path.assign(vohicle_.path.begin(), vohicle_.path.end());
	return _vohicle;
};*/
void Vehicle::move(const unsigned int dest_seq) {
    path.push_back(select[dest_seq]);
    locate = select[dest_seq];
    std::swap(select[dest_seq], select.back());
    select.pop_back();  //快速删除
    if (select.empty()) {
        end = true;
    }
}
//============================= class Vehicle public Node end =============================


//============================= class Ant start =============================

Ant::Ant(const unsigned int loc, const unsigned int sel): locate(loc), end(false), timenow(0) {
    for (unsigned int i = 1; i < sel; i++) { //init select paths
        select.push_back(i);
    }
    path.push_back(loc);
}
Ant::Ant(): locate(0), end(false), timenow(0) {
    path.push_back(0);
}
/*bool Ant::walk(const int node_seq) {
    if (path.find(node_seq) == path.end()) {
    path.insert(std::pair<int, int>(node_seq, node_seq));
    locate = node_seq;
    return true;
    } else {
        return false;
    }
};*/
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
float Ant::path_length(const Eigen::MatrixXf& dists) {
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
/*
std::vector<int>& Ant::select_path() {
    return select;
}
const std::unordered_map<int, int> Ant::walked_path() {
    return path;
}*/
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

