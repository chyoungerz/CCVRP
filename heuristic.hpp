#pragma once

#include <vector>
#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class VN {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	std::vector<double> hist;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

class SA {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

class VND {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

// two-phase metaheuristic framework
class TPM {
  public:
	Solution bestSol;     // 最好解
	Solution ithsolBest;  // 第i次迭代最好解

	void init(); /*{
	     ///@brief 后悔插入构造初始解
	     ///@return bestSol;
	     Solution regretinsertion();
	     while (满足终止条件) {
	         ithsolBest = bestSol;
	         run_first_phase();
	         run_second_phrase();
	     }
	 }*/

	void run_first_phase();   /*{
	       ithsolBest = bestSol;
	       PER OP = random<PER>{exchange-based 算子（解） 、 cross-based 算子（解）};
	       int H = random<int>(0, min(ithsolBest.solution.size()/2, 10));
	       for(int H = random<int>(0, min(R/2, 10)); H > 0; H--) {
	           PER::OP(ithsolBest); //扰动算子（解）
	       }
	       LS::OP(ithsolBest); //局部搜索算子
	       if (ithsolBest < bestSol) bestSol = ithsolBest;
	     }*/
	void run_second_phrase(); /*{
	     bool flag = false;
	     Solution sol = ithsolBest;
	     for(int i = 0; i < sol.solution.size(); i++) {
	        for(int j = 0; j < T; j++) {
	            if(sol.solution[i].path.size() < 8) continue;
	            PER OP = random<PER>{3-opt-based 算子（路径）、 4-opt-based 算子（路径）};
	            PER::OP(sol.solution[i]);
	        }
	        OP::twoOpt(sol.solution[i]); //2-opt算子（路径）
	        if(sol < ithsolBest) {
	            ithsolBest = sol;
	            flag = true;
	            break;
	        }
	     }
	     if(flag) {
	        LS::OP(ithsolBest); //局部搜索算子
	        if (ithsolBest < bestSol) bestSol = ithsolBest;
	     }
	 }*/
};

class GA {
  public:
	Solution bestSol;                // 最好解
	Solution sol;                    // 当前解
	std::vector<Node*> nodes;        // 节点
	u32 depotnum;                    // 厂站
	u32 maxvehicles;                 // 最大车辆
	u32 maxload;                     // 最大载重

	void init(std::vector<Node*>& node, const u32 depot_num, u32 max_load, u32 routes);
	// 编码
	void encode(Solution& sol, std::vector<Node*>& code);
	// 解码
	void decode(std::vector<Node*>& code, Solution& sol);
};

class SWO {
  public:
	std::vector<Solution> solutions;  // 解群
	Solution bestSol;                 // 最好解
	std::vector<Node*> nodes;         // 顾客节点
	std::vector<Node*> depots;        // 场站节点
	u32 depotnum;                     // 厂站
	u32 maxvehicles;                  // 最大车辆
	u32 maxload;                      // 最大载重

	// 初始化算法
	void init(std::vector<Node*>& node, const u32 depot_num, u32 max_load, u32 routes);
	// 编码
	void encode(Solution& sol, std::vector<Node*>& code);
	// 解码
	void decode(std::vector<Node*>& code, Solution& sol);
	/// @brief
	/// @param sol1
	/// @param sol2
	/// @param cr
	/// @return Solution
	Solution cross(Solution& sol1, Solution& sol2, float cr);

	/// @brief
	/// @param min_n 最小种群数
	/// @param epoch 迭代次数
	/// @param tr 狩猎和繁衍行为行为控制
	/// @param cr 繁衍交叉概率
	void run(u32 min_n, int epoch, float tr, float cr);
};
#endif