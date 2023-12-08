// #include <ctime>
#include <chrono>
#include <iostream>
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
	u32 maxload{}, depot_num{}, routes{}, epoch{};
	Solution bestsol;
	bestsol.allength = 1000000000;
	Info infos;
	vector<double> lengths;
	lengths.reserve(10);
	if (argc != 3) {
		file = "P-n23-k8.vrp";
		result = "data.txt";
		routes = 8;
		cerr << "no enought args" << endl;
		cerr << "use default: " << file << endl;
	} else {
		file = argv[1];
		if (file.size() < 6) {
			cerr << "file: " << file << " name error" << endl;
			return 1;
		}
		if (file[file.size() - 6] == 'k') {
			routes = atoi(file.substr(file.size() - 5, 1).c_str());
		} else {
			routes = atoi(file.substr(file.size() - 6, 2).c_str());
		}
		result = file.substr(0, file.size() - 4) + ".txt";
		epoch = atoi(argv[2]);
	}
	// nodes = read(file, maxload, depot_num, routes);
	read_vrp(file, maxload, depot_num, routes, nodes);
	if (nodes.empty()) {
		cerr << "file: " << file << " open failed" << endl;
		return 1;
	}
	init_distance(nodes, depot_num, depots, customers);  // 计算客户距离
	SA vrp;
	vrp.init(nodes, depots, customers, depot_num, maxload, routes);
#ifndef DEBUG
	auto t1{chrono::high_resolution_clock::now()};
	for (u32 i{0}; i < epoch; i++) {
		vrp.run();
		lengths.emplace_back(vrp.bestSol.allength);
		if (bestsol.allength > vrp.bestSol.allength) {
			bestsol = vrp.bestSol;
			infos = vrp.info;
		}
		vrp.reset();
	}
	auto t2{chrono::high_resolution_clock::now()};
	u64 duration = (chrono::duration_cast<chrono::milliseconds>(t2 - t1)).count();
	write(result, bestsol, infos, lengths, duration);
#else
	vrp.run();
	write(result, vrp.bestSol, vrp.info);
#endif

	//   depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾

	release(nodes);
	return 0;
}
