#include "distribution.hpp"

unsigned int discrete_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const Eigen::MatrixXf& pheromates, const Eigen::MatrixXf& dists) {
    std::vector<float> probs;
	std::mt19937 gen(now);
	unsigned int size = interval.size();
    //std::mt11213b gen(std::chrono::high_resolution_clock::now());
    /*float sum(0.0);
    for (int i = 0; i < interval.size(); i++) {
        sum += pheromates(locale, interval[i]);
    }
    for (int i = 0; i < interval.size(); i++) {
        probs.push_back(pheromates(locale, interval[i]) / sum);
    }
    boost::random::uniform_01 gen;
    float temp = gen(rd);
    for (int i = 0; i < probs.size(); i++) {
        if (temp < probs[i] || temp < 0) {
            return interval[i];
        }
        temp = temp - probs[i];
    }*/
    for (unsigned int i = 0; i < size; i++) {
        probs.push_back(pheromates(locale, interval[i]) / dists(locale, interval[i]));
    }
    std::discrete_distribution<> discrete(probs.begin(), probs.end());
    return discrete(gen);
}

unsigned int constrain_distribute(const std::vector<unsigned int>& interval, const unsigned int locale, const unsigned int _timenow, const Eigen::MatrixXf& dists) {
    std::vector<float> probs;
    return 0;
}
