#include <ctime>
#include <vector>

#include "heuristic.hpp"
#include "node.hpp"
#include "solution.hpp"
#include "utils.hpp"

using namespace std;
int main(int argc, char const *argv[]) {
	const time_t now = time(nullptr);  // 以当前时间设置随机数种子
	string file, result;
	vector<Node *> nodes, stations;
	if (argc != 3) {
		cerr << "not enought args" << endl;
		cerr << "use default" << endl;
		file = "c101";
		result = "data.txt";
	} else {
		file = argv[1];
		result = argv[2];
	}
	if (nodes.empty()) {
		cerr << "file open failed" << endl;
		return 1;
	}
	init_distance(nodes);                               // 计算客户距离
	stations.assign(nodes.begin(), nodes.begin() + 1);  // 厂站
	Solution sol = knn(nodes, stations);
	// sol.show();
	create(result, now);
	write(result, sol);
	release(nodes);
	return 0;
}
