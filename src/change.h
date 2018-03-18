#ifndef CHANGE_H
#define CHANGE_H
#include <list>
#include <vector>
#include <algorithm>

using namespace std;

//check other lanes for faster progress and whether save to switch, if so, return best lane id
double consider_change(int lane, vector<vector<double>> sensor_fusion, double current_speed, double car_s, int prev_size, double same_lane_dist);


// identify lane changes by other vehicles, return true if found
bool find_lane_change(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s);


#endif /* CHANGE_H*/
