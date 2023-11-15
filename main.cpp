// #include <ctime>
#include <vector>

#include "fileio.hpp"
#include "heuristic.hpp"
#include "node.hpp"

using namespace std;
/// @brief
/// @param argv 文件 结果文件 迭代次数 接受准则等
int main(int argc, char const *argv[]) {
	ios::sync_with_stdio(false);
	// const time_t now = time(nullptr);  // 以当前时间设置随机数种子
	string file, result;
	vector<Node *> nodes, depots, customers;
	if (argc != 3) {
		file = "XML100_1113_21.vrp";
		result = "data.txt";
		cerr << "no enought args" << endl;
		cerr << "use default: " << file << " " << result << endl;
	} else {
		file = argv[1];
		result = argv[2];
	}
	u32 maxload{}, depot_num{}, routes{23};
	// nodes = read(file, maxload, depot_num, routes);
	read_vrp(file, maxload, depot_num, routes, nodes);
	if (nodes.empty()) {
		cerr << "file: " << file << " open failed" << endl;
		return 1;
	}
	init_distance(nodes, depot_num, depots, customers);  // 计算客户距离
	SA vrp;
	vrp.init(nodes, depots, customers, depot_num, maxload, routes);
	vrp.run();
	//  depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	// create(result, now);
	// write(result, vrp.bestSol);
	release(nodes);
	return 0;
}
