#include <list>
#include <vector>
#include <algorithm>

#include <cstdio>
#include <iostream>
using namespace std;



double consider_change(int lane, vector<vector<double>> sensor_fusion, double current_speed, double car_s, int prev_size, double same_lane_dist)
{

	int rec_lane = lane;
	std::list<int> potential_lanes;

	vector<double> lane_0_speeds;
	vector<double> lane_1_speeds;
	vector<double> lane_2_speeds;

	vector<double> min_speeds;

	//for each car
	cout<<"starting car loop \n";
  	for(int i = 0; i < sensor_fusion.size(); i++)
  	{
  		//log car loop
  		cout<<"car" << i+1 << "from " << sensor_fusion.size() << "\n";
  		float d = sensor_fusion[i][6];
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy *vy);
  		double check_car_s = sensor_fusion[i][5];
  		cout<<"check_car_s " << check_car_s <<   "\n";
  		cout<<"own_s " << car_s <<  "\n";
  		cout<<"check_car_d " << d <<  "\n";
  		// if vehicle is in front and not too far, add its speed to list of speeds for lanes
  		if(  (check_car_s > car_s) && ((check_car_s - car_s) < 65) )
		{
  			//cout<<"found car in front, going " << check_speed << "mph \n";
  			if( (d < 4) && (d >= 0) )
				{
  				//cout<<"car is in left lane \n";
  				lane_0_speeds.push_back(check_speed);}

			if( (d < 8) && (d >= 4) )
				{
				//cout<<"car is in center lane \n";
				lane_1_speeds.push_back(check_speed);}

			if( (d < 12) && (d >= 8) )
				{
				//cout<<"car is in right lane \n";
				lane_2_speeds.push_back(check_speed);}
		}
  	}

	//find min speeds per lane; if no vehicles set to high number
	double lane0m;
	double lane1m;
	double lane2m;

	if (lane_0_speeds.size()>0)
		{lane0m = *std::min_element(lane_0_speeds.begin(),lane_0_speeds.end());;}
	else
		{lane0m = 999;}

	if (lane_1_speeds.size()>0)
		{lane1m = *std::min_element(lane_1_speeds.begin(),lane_1_speeds.end());;}
	else
		{lane1m = 999;}

	if (lane_2_speeds.size()>0)
		{lane2m = *std::min_element(lane_2_speeds.begin(),lane_2_speeds.end());;}
	else
		{lane2m = 999;}

	/* ddebug
	cout<<"lane_0_speeds: \n";
	for (auto const& c : lane_0_speeds)
		std::cout << c << ' ';
	cout<<"\n";
	cout<<"lane_1_speeds: \n";
	for (auto const& c : lane_1_speeds)
		std::cout << c << ' ';
	cout<<"\n";
	cout<<"lane_2_speeds: \n";
	for (auto const& c : lane_2_speeds)
		std::cout << c << ' ';
	cout<<"\n";
	*/

	min_speeds.push_back(lane0m);
	min_speeds.push_back(lane1m);
	min_speeds.push_back(lane2m);
	cout<<"min speeds: \n";
	/* debug

	 cout<<min_speeds";
	for (auto const& c : min_speeds)
		std::cout << c << ' ';
	cout<<"\n";
	 */


  	// compare speeds in adjacent lanes, set recommended lane in case minimum speed of vehicle in front is faster than in current lane
	switch(lane)
	{
	case 0:
		if (min_speeds[0] < min_speeds[1] + 0.5)
			{cout<<"current speed in CL better than my LL, going center \n";
			rec_lane = 1;}
		else if (min_speeds[2] + 1.5 > min_speeds[0]) //if far lane significantly faster try to switch to center lane
		{rec_lane = 1;}
		break;

	case 1:
		/* debug
		cout<<"entered switch case 1 \n";
		cout<< "min_speeds0: " << min_speeds[0] << "\n";
		cout<< "min_speeds1: " << min_speeds[1] << "\n";
		cout<< "min_speeds2: " << min_speeds[2] << "\n";
		cout<< "current_speed: " << current_speed << "\n";
		*/
		if ( (min_speeds[0] > min_speeds[1]+0.5) &&  (min_speeds[0] >= min_speeds[2])) //if both lanes clear prefer left, needs to be improved
		{cout<<"current speed in LL better than my CL, going LL \n";
			rec_lane = 0;}
		if ( (min_speeds[2] > min_speeds[1]+0.5) && (min_speeds[0] < min_speeds[2]))
		{cout<<"current speed in RL better than my CL, going RL \n";
			rec_lane = 2;}

		cout<<"result of switch: " << rec_lane << " \n";
		break;

	case 2:
		if (min_speeds[lane] < min_speeds[1] + 0.5)
			{
			cout<<"current speed my RL slower, going CL \n";
			rec_lane = 1;
			}
		else if (min_speeds[0] + 1.5 >  min_speeds[2]) //if far lane significantly faster try to switch to center lane
		{rec_lane = 1;}
		break;
	}


  	//test feasibility of recommended lane
	if (rec_lane != lane)
	{
		/*
		cout<<"ok, so you want me to switch to " << rec_lane  << " \n";
		*/
		//for recommended lane, project s and compare to self
		//cout<<"current own s " << car_s  << " \n";
		double car_s_x = car_s;
		car_s_x += ((double)prev_size * 0.02 * current_speed); //extrapolate own s position
		//cout<<"extra own s " << car_s_x  << " \n";
		// loop through vehicles
      	for(int i = 0; i < sensor_fusion.size(); i++)
      	{
      		float d = sensor_fusion[i][6];
      			// find vehicles in recommended lane

      			if(d < (2+4*rec_lane+2) && d > (2+4*rec_lane-2))
				{
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx + vy *vy);
					double check_car_s = sensor_fusion[i][5];
					//cout<<"current s of veh " << check_car_s  << " \n";
					double check_car_s_x = check_car_s;
					check_car_s_x += ((double)prev_size * 0.02 * check_speed); //extrapolate veh s position
					//cout<<"extra s of veh " << check_car_s_x  << " \n";

					double dist = car_s - check_car_s;
					double dist_x = car_s_x - check_car_s_x;
					cout<<"	dist: " << dist  << " \n";
					cout<<"	dist extrapolated: " << dist_x  << " \n";
					cout<<"	dist same_lane: " << same_lane_dist  << " \n";

					// check if current distance, extrapolated distance to target lane or distance in own lane too small
					if (  (dist_x > 0 && dist_x < 15 )  ||  (dist_x < 0 && dist_x > -15) || (std::abs(dist) < 15) || (same_lane_dist < 10) ) // car getting close in s-dimension
						{
							// not feasible, so stay in current lane
							cout<<"		not enough space to switch, staying here \n";
							rec_lane = lane;
						}

      			}
      	}
      	// return recommended lane
      	cout<<"recommended lane is " << rec_lane  << " \n";
      	return rec_lane;


	}
	cout<<"recommend to stay in " << rec_lane  << " \n";
	return rec_lane;
}


bool find_lane_change(int lane, vector<vector<double>> sensor_fusion, int prev_size, double car_s)
{
/* find lane changers, if found return true
 * currently also triggers if cars leave our lane, could be improved
  */
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		double s = sensor_fusion[i][5];
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy *vy);
		s += ((double)prev_size * 0.02 * check_speed); //extrapolate veh s position
		if (car_s < s) //if car is in front
		{
			if( (lane == 0) && (d > 3) && (d<5))
				{return true;}
			if( (lane == 1) && ((d > 3) && (d<5)) || ( (d > 7) && (d<9)  ))
				{return true;}
			if( (lane == 2) && (d > 7) && (d<9))
				{return true;}
		}
		return false;
	}
}

