#pragma once

#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class SA {
  public:
	Solution bestSol;  // 最好解
	Solution sol;      // 当前解
	std::vector<const Node*> nodes;  // 节点
	uint32_t depotnum;               // 厂站
	uint32_t vehicles;               // 最大车辆

	/// @brief
	void init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload, uint32_t routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
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

class SWO {
  public:
	std::vector<Solution> solutions;  // 解群
	Solution bestSol;                 // 最好解
	std::vector<const Node*> nodes;   // 节点
	std::vector<uint32_t> codes;      // 编码
	uint32_t depotnum;                // 厂站
	uint32_t vehicles;                // 最大车辆

	// 初始化算法
	void init(std::vector<const Node*>& node, const uint32_t depot_num, uint32_t maxload, uint32_t routes);

	// 编码
	void encode(Solution& sol, std::vector<uint32_t>& code);
	// 解码
	void decode(std::vector<uint32_t>& code, Solution& sol);

	// 交叉
	Solution cross(Solution& sol1, Solution& sol2, float cr);

	/// @brief
	/// @param min_n 最小种群数
	/// @param epoch 迭代次数
	/// @param tr 狩猎和繁衍行为行为控制
	/// @param cr 繁衍交叉概率
	void run(uint32_t min_n, int epoch, float tr, float cr);
};
#endif