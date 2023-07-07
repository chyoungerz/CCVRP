#include <ctime>
#include <vector>

#include "fileio.hpp"
#include "heuristic.hpp"
#include "node.hpp"
#include "operator.hpp"

using namespace std;
int main(int argc, char const *argv[]) {
	// const time_t now = time(nullptr);  // 以当前时间设置随机数种子
	string file, result;
	vector<Node *> node;
	vector<const Node *> nodes;
	if (argc != 3) {
		file = "p10";
		result = "data.txt";
		cerr << "no enought args" << endl;
		cerr << "use default: " << file << " " << result << endl;
	} else {
		file = argv[1];
		result = argv[2];
	}
	uint32_t maxload, depot_num;
	node = read(file, maxload, depot_num);
	if (node.empty()) {
		cerr << "file open failed" << endl;
		return 1;
	}
	init_distance(node, depot_num);  // 计算客户距离
	for (uint32_t i = 0; i < node.size(); i++) {
		nodes.emplace_back(static_cast<const Node *>(node[i]));
	}
	node.clear();
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	SA vrp;
	vrp.init(nodes, depot_num, maxload);
	vrp.run();
	nodes.clear();
	// create(result, now);
	// write(result, vrp.bestSol);
	release(node);
	return 0;
}
