#pragma once

#include <algorithm>
#ifndef _ALGORITHM_HPP
#define _ALGORITHM_HPP

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
	template <typename T, typename Fn>
	inline void topK(std::vector<T>& vec, unsigned int low_index, unsigned int high_index, unsigned int k, Fn cmp) {
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

	/**
	 * @brief 这个函数使用快速选择算法，用于在一系列元素中找到最大的k个元素
	 * @warning 比较函数必须是（<=）。最大k个元素在末尾。
	 * @tparam _RanIt 随机迭代器类型。
	 * @tparam _Fn 函数对象类型。
	 * @param _first 指向范围中第一个元素的迭代器。
	 * @param _last 指向范围中最后一个元素的迭代器。
	 * @param _k k 的值。
	 * @param _cmp 用于比较元素的函数对象。
	 * @return void
	 */
	template <class _RanIt, class _Fn>
	inline void topk(_RanIt _first, _RanIt _last, unsigned int _k, _Fn _cmp) {
		if (_last <= _first || _k == 0) return;
		if (_k >= _last - _first) return;
		_RanIt _left{_first}, _right{_last};
		// auto _size = std::distance(_first, _last);
		_RanIt _pivot{_first + ((_first, _last) >> 1)};
		while (_left < _right) {
			while (_left < _right && _cmp(*_pivot, *_right)) --_right;
			while (_left < _right && _cmp(*_left, *_pivot)) ++_left;
			if (_left < _right) {
				std::iter_swap(_left, _right);
				--_right, ++_left;
			}
		}
		std::iter_swap(_left, _pivot);
		auto _len = _last - _left;
		if (_k == _len || _k == _len + 1) return;
		if (_k < _len)
			topk(_left + 1, _last, _k, _cmp);
		else
			topk(_first, _left, _k - _len, _cmp);
	}

	/// @brief K聚类算法
	/// @param nodes 节点
	/// @param depots K 同时也是初始群
	/// @param epoch_max 最大迭代次数
	/// @param classfy 结果
	/// @return 若超过最大迭代次数则false，否则true
	inline bool Kmean(std::vector<Node*>& customers, std::vector<Node*>& depots, std::vector<Node>& K, unsigned int epoch_max, std::vector<std::vector<Node*>>& classfy) {
		u32 k = depots.size();
		while (epoch_max--) {
			for (auto& v : customers) {  // 将customers分为k类
				double min_dist = 1000000.0;
				unsigned int min_index = 10000;
				for (unsigned int i{0}; i < k; ++i) {
					double d = dist(*v, K[i]);
					if (d < min_dist) {
						min_dist = d;
						min_index = i;
					}
				}
				classfy[min_index].emplace_back(v);
			}
			bool flag = true;
			for (unsigned int i{0}; i < k; ++i) {
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
		if (epoch_max) return true;
		else
			return false;
	}

	/**
	 * 将范围[first, last)中的元素旋转k个位置。
	 *
	 * @tparam _FwdIt 前向迭代器类型。
	 * @param _first 范围中第一个元素的迭代器。
	 * @param _last 范围中最后一个元素的迭代器。
	 * @param k 旋转元素的位置数。如果为正数，则向右旋转元素。如果为负数，则向左旋转元素。
	 */
	template <class _FwdIt>
	inline void rotate(_FwdIt _first, _FwdIt _last, int k) {
		if (_last <= _first + 1 || k == 0) return;
		if (k > 0) {  // 向右旋转
			_FwdIt _mid{_last - k};
			std::reverse(_first, _mid);
			std::reverse(_mid + 1, _last);
			std::reverse(_first, _last);
		} else {  // 向左旋转
			_FwdIt _mid{_first - k};
			std::reverse(_first, _mid - 1);
			std::reverse(_mid, _last);
			std::reverse(_first, _last);
		}
	}

	/**
	 * @brief 计算DTW距离
	 * @tparam T
	 * @tparam _Dn
	 * @param _r1 路径r1
	 * @param _r2 路径r2
	 * @param _dist 距离函数
	 * @return 距离
	 */
	template <class T, class _Dn>
	inline auto dtw(const std::vector<T>& _r1, const std::vector<T>& _r2, _Dn _dist) -> decltype(_dist) {
		// std::vector<std::vector<double>> _dtw(_r1.size()-1, std::vector<double>(_r2.size()-1, 0.0));
		const unsigned int _r1_size = _r1.size() - 2, _r2_size = _r2.size() - 2;
		if (_r1_size <= 0 || _r2_size <= 0) return;
		auto _dtw = new decltype(_dist)[_r1_size][_r2_size];
		decltype(_dist) _dist_min{};
		_dtw[0][0] = _dist(_r1[1], _r2[1]);
		for (unsigned int i{1}; i < _r1_size; ++i) {
			_dtw[i][0] = _dtw[i - 1][0] + _dist(_r1[i + 1], _r2[1]);
		}
		for (unsigned int i{1}; i < _r2_size; ++i) {
			_dtw[0][i] = _dtw[0][i - 1] + _dist(_r1[1], _r2[i + 1]);
		}
		for (unsigned int i{1}; i < _r1_size; ++i) {
			for (unsigned int j{1}; j < _r2_size; ++j) {
				_dtw[i][j] = std::min({_dtw[i - 1][j], _dtw[i][j - 1], _dtw[i - 1][j - 1]}) + _dist(_r1[i + 1], _r2[j + 1]);
			}
		}
		_dist_min = _dtw[_r1_size - 1][_r2_size - 1];
		delete[] _dtw;
		return _dist_min;
	}

}  // namespace ALG

#endif