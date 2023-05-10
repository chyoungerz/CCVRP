#include <vector>

#include "heuristic.hpp"
#include "node.hpp"
#include "utils.hpp"

using namespace std;
int main(int argc, char const *argv[]) {
	string file = "c101";
	if (argc != 2) {
		cerr << "not enought args" << endl;
		cerr << "use default" << endl;
	}
	bool signal = read(file);
	if (!signal) {
		cerr << "file open failed" << endl;
		return 1;
	}
	init_distance();  // 计算距离
	string result = "data.txt";
	create(result);
	return 0;
}
