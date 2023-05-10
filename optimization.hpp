#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <string>
#include <vector>

#include "node.hpp"

/*
bool twoStringMove(std::vector<int> &tour, int i, int j, int X, int Y);

bool twoString(std::vector<int> &tour, int X, int Y, bool firstImrpove, int stop_cnt);

double twoStringCostUpdate(std::vector<int> &tour, int i, int j, int X, int Y);

inline bool onePoint(std::vector<int> &tour, bool firstImprove, int stop_cnt) {
    return twoString(tour, 0, 1, firstImprove, stop_cnt);
}

inline bool twoPoint(std::vector<int> &tour, bool firstImprove, int stop_cnt) {
    return twoString(tour, 1, 1, firstImprove, stop_cnt);
}

inline bool threePoint(std::vector<int> &tour, bool firstImprove, int stop_cnt) {
    return twoString(tour, 1, 2, firstImprove, stop_cnt);
}

void twoOptMove(std::vector<int> &tour, int i, int j);

bool twoOpt(std::vector<int> &tour, bool firstImprove, int stop_cnt);

double twoOptCostUpdate(std::vector<int> &tour, int i, int j);

bool AFSrealoc_one(std::vector<int> &tour, bool firstImprove, int termination, int actual_problem_size);
bool AFSrealoc_more_than_one(std::vector<int> &tour, bool firstImprove);

double get_distance(int from, int to);				//returns the distance
double get_evals();									//returns the number of evaluations
bool isValidTour(std::vector<int> tour);
bool is_charging_station(int node);					//returns true if node is a charging station
//Noncapacitated zga for subtours with one AFS
std::vector<int> tsp2evrp_zga_mini(std::vector<int> tspTour);
*/
// 交换路径中i和j位置
bool swapexchange(Tour& tours, const size_t i, const size_t j);

#endif