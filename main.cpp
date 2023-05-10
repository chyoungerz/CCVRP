#include <ctime>
#include <vector>

#include "heuristic.hpp"
#include "node.hpp"
#include "utils.hpp"

#define MAXLOAD 200

using namespace std;
int main(int argc, char const *argv[]) {
	const time_t now = time(nullptr);  // 以当前时间设置随机数种子
	string file = "c101";
	if (argc != 2) {
		cerr << "not enought args" << endl;
		cerr << "use default" << endl;
	}
	vector<Node> nodes = read(file);
	if (nodes.empty()) {
		cerr << "file open failed" << endl;
		return 1;
	}
	Eigen::MatrixXf dists = init_distance(nodes);  // 计算距离
	// string result = "data.txt";
	//  create(result, now);
	return 0;
}