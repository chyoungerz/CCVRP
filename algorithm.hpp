#pragma once

#include <algorithm>
#ifndef _ALGORITHM_HPP
#define _ALGORITHM_HPP

#include <array>
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
	template <class _RanIt, typename _Fn>
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
	 * @brief 计算SM距离
	 * @tparam T
	 * @tparam _Dn
	 * @param _r1 路径r1
	 * @param _r2 路径r2
	 * @param _dist 距离函数
	 * @return 距离
	 */
	template <typename T, typename _Dn>
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

	/**
	 * @brief 计算SM距离
	 * @tparam T
	 * @tparam _Dn
	 * @param _r1 路径r1
	 * @param _r2 路径r2
	 * @param _dist 距离函数
	 * @return 距离
	 */
	template <typename T, typename _Dn>
	inline double dtw(const std::vector<T>& _r1, const std::vector<T>& _r2, double** mem_dtw, _Dn _dist) {
		// std::vector<std::vector<double>> _dtw(_r1.size()-1, std::vector<double>(_r2.size()-1, 0.0));
		const unsigned int _r1_size = _r1.size() - 2, _r2_size = _r2.size() - 2;
		if (_r1_size <= 0 || _r2_size <= 0) return 0.0;
		auto _dtw = new decltype(_dist)[_r1_size][_r2_size];
		double _dist_min{};
		if (mem_dtw == nullptr) throw "mem_dtw is nullptr";
		mem_dtw[0][0] = _dist(_r1[1], _r2[1]);
		for (unsigned int i{1}; i < _r1_size; ++i) {
			mem_dtw[i][0] = mem_dtw[i - 1][0] + _dist(_r1[i + 1], _r2[1]);
		}
		for (unsigned int i{1}; i < _r2_size; ++i) {
			mem_dtw[0][i] = mem_dtw[0][i - 1] + _dist(_r1[1], _r2[i + 1]);
		}
		for (unsigned int i{1}; i < _r1_size; ++i) {
			for (unsigned int j{1}; j < _r2_size; ++j) {
				mem_dtw[i][j] = std::min({mem_dtw[i - 1][j], mem_dtw[i][j - 1], mem_dtw[i - 1][j - 1]}) + _dist(_r1[i + 1], _r2[j + 1]);
			}
		}
		_dist_min = mem_dtw[_r1_size - 1][_r2_size - 1];
		return _dist_min;
	}

	// Similarity Measurement
	class SM {
		double* mem_mat{nullptr};
		u32 mem_dtw_r1{};
		u32 mem_dtw_r2{};

	  protected:
		double _dtw_r_(const std::vector<Node*>& _r1, const std::vector<Node*>& _r2) {
			// std::vector<std::vector<double>> _dtw(_r1.size()-1, std::vector<double>(_r2.size()-1, 0.0));
			const u32 _r1_size = _r1.size() - 2, _r2_size = _r2.size() - 2;
			if (_r1_size <= 0 || _r2_size <= 0) throw "path is empty";
			if (mem_mat == nullptr) throw "mem_dtw is nullptr";
			mem_mat[0] = dist(_r1[1], _r2[1]);
			for (unsigned int i{1}; i < _r1_size; ++i) {
				mem_mat[i * _r1_size] = mem_mat[(i - 1) * _r1_size] + dist(_r1[i + 1], _r2[1]);
			}
			for (unsigned int i{1}; i < _r2_size; ++i) {
				mem_mat[i] = mem_mat[i - 1] + dist(_r1[1], _r2[i + 1]);
			}
			for (unsigned int i{1}; i < _r1_size; ++i) {
				for (unsigned int j{1}; j < _r2_size; ++j) {
					mem_mat[i * _r1_size + j] = std::min({mem_mat[(i - 1) * _r1_size + j], mem_mat[i * _r1_size + j - 1], mem_mat[(i - 1) * _r1_size + j - 1]}) + dist(_r1[i + 1], _r2[j + 1]);
				}
			}
			return mem_mat[(_r1_size - 1) * _r1_size + _r2_size - 1];
		}

	  public:
		SM() = default;
		~SM() { delete[] mem_mat; }
		SM(const SM&) = delete;             // 复制构造
		SM(SM&&) = delete;                  // 移动构造
		SM& operator=(const SM&) = delete;  // 复制赋值
		SM& operator=(SM&&) = delete;       // 移动赋值

		double dtw(std::vector<Vehicle>& s1, std::vector<Vehicle>& s2) {
			u32 size = s1.size();
			if (size != s2.size()) throw "r1_size != r2_size";
			u32 index[size];
			double dist_min{};
			bool need_expand{0};
			for (u32 i{0}; i < size; ++i) {
				if (mem_dtw_r1 + 2 < s1[i].path.size()) {
					mem_dtw_r1 = s1[i].path.size();
					need_expand = 1;
				}
				if (mem_dtw_r2 + 2 < s2[i].path.size()) {
					mem_dtw_r2 = s2[i].path.size();
					need_expand = 1;
				}
				index[i] = i;
			}
			if (need_expand) {
				delete[] mem_mat;
				mem_mat = new double[mem_dtw_r1 * mem_dtw_r2];
			}

			for (auto& i : s1) {
				double _dist_min{1000000.0}, temp{};
				u32 index_seq{};
				for (u32 j{0}; j < size; ++j) {
					temp = _dtw_r_(i.path, s2[index[j]].path);
					if (temp < _dist_min) {
						_dist_min = temp;
						index_seq = j;
					}
				}
				dist_min += _dist_min;
				size--;
				index[index_seq] = index[size];
			}
			return dist_min;
		}
	};
}  // namespace ALG

/**
 * @brief
 * @link https://github.com/Reputeless/Xoshiro-cpp/blob/master/XoshiroCpp.hpp @endlink
 */
namespace Xoshiro {

	inline constexpr unsigned long long int DefaultSeed = 0x123456789ULL;

	[[nodiscard]] static constexpr unsigned long long int RotL(const unsigned long long int x, const int s) noexcept {
		return (x << s) | (x >> (64 - s));
	}

	[[nodiscard]] static constexpr unsigned int RotL(const unsigned int x, const int s) noexcept {
		return (x << s) | (x >> (32 - s));
	}
	/**
	 * @brief SplitMix64
	 * Output: 64 bits
	 * Period: 2^64
	 * Footprint: 8 bytes
	 * Original implementation: http://prng.di.unimi.it/splitmix64.c
	 */
	class SplitMix64 {
	  public:
		using state_type = unsigned long long int;
		using result_type = unsigned long long int;

		[[nodiscard]] explicit constexpr SplitMix64(state_type state = DefaultSeed) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			unsigned long long int z = (m_state += 0x9e3779b97f4a7c15);
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
			z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
			return z ^ (z >> 31);
		}

		template <std::size_t N>
		[[nodiscard]] constexpr std::array<unsigned long long int, N> generateSeedSequence() noexcept {
			std::array<unsigned long long int, N> seeds = {};
			for (auto& seed : seeds) {
				seed = operator()();
			}
			return seeds;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const SplitMix64& lhs, const SplitMix64& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const SplitMix64& lhs, const SplitMix64& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};

	/**
	 * @brief xoshiro256**
	 * @return 64 bits
	 * @link http://prng.di.unimi.it/xoshiro256starstar.c @endlink
	 * @version 1.0
	 */
	class Xoshiro256ss {
	  public:
		using state_type = std::array<unsigned long long int, 4>;
		using result_type = unsigned long long int;

		[[nodiscard]] explicit constexpr Xoshiro256ss(unsigned long long int seed = DefaultSeed) noexcept : m_state(SplitMix64{seed}.generateSeedSequence<4>()) {}

		[[nodiscard]] explicit constexpr Xoshiro256ss(state_type state) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			const unsigned long long int result = RotL(m_state[1] * 5, 7) * 9;
			const unsigned long long int t = m_state[1] << 17;
			m_state[2] ^= m_state[0];
			m_state[3] ^= m_state[1];
			m_state[1] ^= m_state[2];
			m_state[0] ^= m_state[3];
			m_state[2] ^= t;
			m_state[3] = RotL(m_state[3], 45);
			return result;
		}

		constexpr void jump() noexcept {
			constexpr unsigned long long int JUMP[] = {0x180ec6d33cfd0aba, 0xd5a61266f0c9392c, 0xa9582618e03fc9aa, 0x39abdc4529b1661c};
			unsigned long long int s0 = 0;
			unsigned long long int s1 = 0;
			unsigned long long int s2 = 0;
			unsigned long long int s3 = 0;

			for (unsigned long long int jump : JUMP) {
				for (int b = 0; b < 64; ++b) {
					if (jump & 0x1ULL << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		constexpr void longJump() noexcept {
			constexpr unsigned long long int LONG_JUMP[] = {0x76e15d3efefdcbbf, 0xc5004e441c522fb3, 0x77710069854ee241, 0x39109bb02acbe635};

			unsigned long long int s0 = 0;
			unsigned long long int s1 = 0;
			unsigned long long int s2 = 0;
			unsigned long long int s3 = 0;

			for (unsigned long long int jump : LONG_JUMP) {
				for (int b = 0; b < 64; ++b) {
					if (jump & 0x1ULL << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const Xoshiro256ss& lhs, const Xoshiro256ss& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const Xoshiro256ss& lhs, const Xoshiro256ss& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};

	/**
	 * xoshiro128**
	 * @return 64 bits
	 * @link http://prng.di.unimi.it/xoshiro128starstar.c @endlink
	 * @version 1.1
	 */
	class Xoshiro128ss {
	  public:
		using state_type = std::array<unsigned int, 4>;
		using result_type = unsigned int;

		[[nodiscard]] explicit constexpr Xoshiro128ss(unsigned long long int seed = DefaultSeed) noexcept : m_state() {
			SplitMix64 splitmix{seed};

			for (auto& state : m_state) {
				state = static_cast<unsigned int>(splitmix());
			}
		}

		[[nodiscard]] explicit constexpr Xoshiro128ss(state_type state) noexcept : m_state(state) {}

		constexpr result_type operator()() noexcept {
			const unsigned int result = RotL(m_state[1] * 5, 7) * 9;
			const unsigned int t = m_state[1] << 9;
			m_state[2] ^= m_state[0];
			m_state[3] ^= m_state[1];
			m_state[1] ^= m_state[2];
			m_state[0] ^= m_state[3];
			m_state[2] ^= t;
			m_state[3] = RotL(m_state[3], 11);
			return result;
		}

		constexpr void jump() noexcept {
			constexpr unsigned int JUMP[] = {0x8764000b, 0xf542d2d3, 0x6fa035c3, 0x77f2db5b};

			unsigned int s0 = 0;
			unsigned int s1 = 0;
			unsigned int s2 = 0;
			unsigned int s3 = 0;

			for (unsigned int jump : JUMP) {
				for (int b = 0; b < 32; ++b) {
					if (jump & 0x1U << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		constexpr void longJump() noexcept {
			constexpr unsigned int LONG_JUMP[] = {0xb523952e, 0x0b6f099f, 0xccf5a0ef, 0x1c580662};

			unsigned int s0 = 0;
			unsigned int s1 = 0;
			unsigned int s2 = 0;
			unsigned int s3 = 0;

			for (unsigned int jump : LONG_JUMP) {
				for (int b = 0; b < 32; ++b) {
					if (jump & 0x1U << b) {
						s0 ^= m_state[0];
						s1 ^= m_state[1];
						s2 ^= m_state[2];
						s3 ^= m_state[3];
					}
					operator()();
				}
			}
			m_state[0] = s0;
			m_state[1] = s1;
			m_state[2] = s2;
			m_state[3] = s3;
		}

		[[nodiscard]] static constexpr result_type min() noexcept {
			return std::numeric_limits<result_type>::lowest();
		}

		[[nodiscard]] static constexpr result_type max() noexcept {
			return std::numeric_limits<result_type>::max();
		}

		[[nodiscard]] constexpr state_type serialize() const noexcept {
			return m_state;
		}

		constexpr void deserialize(state_type state) noexcept {
			m_state = state;
		}

		[[nodiscard]] friend bool operator==(const Xoshiro128ss& lhs, const Xoshiro128ss& rhs) noexcept {
			return (lhs.m_state == rhs.m_state);
		}

		[[nodiscard]] friend bool operator!=(const Xoshiro128ss& lhs, const Xoshiro128ss& rhs) noexcept {
			return (lhs.m_state != rhs.m_state);
		}

	  private:
		state_type m_state;
	};
}  // namespace Xoshiro
#endif