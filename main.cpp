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
	bestsol.allobj = 1000000000;
	if (argc != 3) {
		file = "A-n32-k5-PrU.vrp";
		result = "data.txt";
		routes = 5;
		cerr << "no enought args" << endl;
		cerr << "use default: " << file << endl;
	} else {
		file = argv[1];
		if (file.size() < 10) {
			cerr << "file: " << file << " name error" << endl;
			return 1;
		}
		if (file[file.size() - 10] == 'k') {
			routes = atoi(file.substr(file.size() - 9, 1).c_str());
		} else {
			routes = atoi(file.substr(file.size() - 10, 2).c_str());
		}
		result = "results/" + file.substr(2, file.size() - 6) + ".txt";
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
#ifdef NDEBUG
	Info infos;
	vector<double> lengths, objs, tardiness;
	lengths.reserve(10);
	objs.reserve(10);
	tardiness.reserve(10);
	auto t1{chrono::high_resolution_clock::now()};
	for (u32 i{0}; i < epoch; i++) {
		vrp.run();
		lengths.emplace_back(vrp.bestSol.allength);
		objs.emplace_back(vrp.bestSol.allobj);
		tardiness.emplace_back(vrp.bestSol.alltardiness);
		if (bestsol.allobj > vrp.bestSol.allobj) {
			bestsol = vrp.bestSol;
			infos = vrp.info;
		}
		vrp.reset();
	}
	auto t2{chrono::high_resolution_clock::now()};
	u64 duration = (chrono::duration_cast<chrono::milliseconds>(t2 - t1)).count();
	write(result, bestsol, infos, lengths, objs, tardiness, duration);
#else
	vrp.run();
	vrp.bestSol.show();
#endif

	release(nodes);
	return 0;
}
