#pragma once

#ifndef _ALGORITHM_HPP
#define _ALGORITHM_HPP

#include <random>
#include <vector>

#include "node.hpp"

// #include <eigen3/Eigen/Core>

// 离散分布，interval所有可能的选择的集合， locale当前位置， pheromates信息素，dists距离矩阵
// unsigned int discrete_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const Eigen::MatrixXf& pheromates, const Eigen::MatrixXf& dists);
// 约束分布，interval所有可能的选择的集合, locale当前位置, _timenow当前时间， dists距离矩阵
// unsigned int constrain_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const unsigned int _timenow, const Eigen::MatrixXf& dists);

/// @brief 为搜索提供的一些算法
namespace ALG {

	/// @brief 找出数组最大的k个元素
	/// @tparam T 元素类型
	/// @param vec 数组名称,最大k个元素在末尾
	/// @param low_index 开始位置
	/// @param high_index 结束位置
	/// @param k 最大的个数
	template <typename T>
	inline void topK(std::vector<T>& vec, unsigned int low_index, unsigned int high_index, unsigned int k) {
		if (high_index <= low_index) return;
		unsigned int left{low_index}, right{high_index};
		T pivot{vec[low_index]};
		while (left < right) {
			while (vec[right] > pivot && left < right) right--;
			if (left < right) vec[left++] = vec[right];
			while (vec[left] < pivot && left < right) left++;
			if (left < right) vec[right--] = vec[left];
		}
		vec[left] = pivot;
		if (k == vec.size() - left || k == vec.size() - left - 1) return;
		if (k < vec.size() - left)
			topK(vec, left + 1, high_index, k);
		else
			topK(vec, low_index, left - 1, k);
	}

	/// @brief 找出数组最大的k个元素
	/// @tparam T 元素类型
	/// @param vec 数组名称,最大k个元素在末尾
	/// @param low_index 开始位置
	/// @param high_index 结束位置
	/// @param k 最大的个数
	/// @param cmp 比较函数(<)
	template <typename T, typename F>
	inline void topK(std::vector<T>& vec, unsigned int low_index, unsigned int high_index, unsigned int k, F cmp) {
		if (high_index <= low_index) return;
		unsigned int left{low_index}, right{high_index};
		T pivot{vec[low_index]};
		while (left < right) {
			while (cmp(pivot, vec[right]) && left < right) right--;
			if (left < right) vec[left++] = vec[right];
			while (cmp(vec[left], pivot) && left < right) left++;
			if (left < right) vec[right--] = vec[left];
		}
		vec[left] = pivot;
		if (k == vec.size() - left || k == vec.size() - left - 1) return;
		if (k < vec.size() - left)
			topK(vec, left + 1, high_index, k);
		else
			topK(vec, low_index, left - 1, k);
	}

	/// @brief K聚类算法
	/// @param nodes 节点
	/// @param K K 同时也是初始群
	/// @param epoch_max 最大迭代次数
	/// @param classfy 结果
	/// @return 若超过最大迭代次数则false，否则true
	inline bool Kmean(std::vector<const Node*>& nodes, std::vector<Node>& K, unsigned int epoch_max, std::vector<std::vector<const Node*>>& classfy) {
		// std::vector<Node> K(k);
		uint32_t k = K.size();
		std::vector<const Node*> vec;
		vec.assign(nodes.begin(), nodes.end() - k);
		while (epoch_max--) {
			for (auto& v : vec) {
				double min_dist = 1000000.0;
				unsigned int min_index = 10000;
				for (unsigned int i = 0; i < k; ++i) {
					double d = dist(*v, K[i]);
					if (d < min_dist) {
						min_dist = d;
						min_index = i;
					}
				}
				classfy[min_index].emplace_back(v);
			}
			bool flag = true;
			for (unsigned int i = 0; i < k; ++i) {
				Node sum;
				for (auto& v : classfy[i]) {
					sum += *v;
				}
				if (classfy[i].size() > 0) {
					sum /= classfy[i].size();
				}
				if (sum != K[i]) {
					flag = false;
					K[i] = sum;
				}
			}
			if (flag) break;
			for (auto& j : classfy) {
				j.clear();
			}
		}
		K.clear();
		vec.clear();
		if (epoch_max) return true;
		else
			return false;
	}

}  // namespace ALG

#endif