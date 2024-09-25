#ifndef SOLUTION_HEADER
#define SOLUTION_HEADER

#include "Instance.h"
#include "ArtificialNode.h"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <deque>
ILOSTLBEGIN

using namespace std;


class Solution {

public:

	double obj_value;
	int last_delivered_customer_obj2=-1;
	pair<int,int> last_delivered_customers_last_delivery_op ={-1,-1} ;
	double rest_flight_time_drone;//the rest flight time of the drone to its first node. If it is needed (relevant for reopt), then in the truck sortie the starting node is -5
	vector<vector<int>>list_truck_edges;//important for the lower bound. One of these edges will be prohibited
	vector<vector<int>>truck_sorties;//if the truck is waiting, it remains in the same node. Structure: a) n_entries in the sortie, b) current node, c) next node..., d) last node. If the truck is waiting --> only a) and b) (n_entries=1)
	vector<vector<int>>drone_sorties;//if the truck is waiting, structure: a) n_entries in the sortie, b) node of drone delivery 1, c) node of drone delivery 2.... If the truck is travelling, structure: a) n_entries in the sortie, b) node of drone delivery, i.e. n_entries=1
	vector<vector<int>> drone_customers; //drone customers for each stage, -1 if no drone customer in this stage
	vector<vector<int>> list_of_op_customers; // customers for each operation, important for the checking procedures and right accounting of service times, from end to start
	int n_stages;//n of filled stages. 
	string mistake_text = "";
	bool mistake_reported = false;
	int number_of_nodes;
	vector<vector<double>> drone_distances;//shortest part distances for the drone (both ways, to and from the node). Computed after reading the input file
	vector<vector<int>> previous_drone_node;
	vector<vector<double>>truck_distances;
	vector<vector<vector<int>>>previous_truck_node;
	map<int, ArtificialNode> art_node_map;
	map<int,double> service_time_map;
	int total_planned_truck_damages=0;

public:

	/**
	* Constructor for the Solution class to put the solution into this structure.
	*/
	Solution(double obj_value,int n_nodes);
	double get_route_travel_time(bool for_truck, bool sortie_time=false);
    double get_route_tandem_travel_time();
    double get_route_waiting_time(bool for_truck);
    vector<double> get_op_travel_time_vec(bool for_truck);
	vector<double> get_op_waiting_time_vec(bool for_truck);
	vector<double> get_op_travel_and_waiting_time_vec(bool for_truck);
	int get_damaged_trucks_arcs(vector<vector<int>> current_damages);
	int get_drone_riding_truck_ops();

private:

};

#endif