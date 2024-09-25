#ifndef INSTANCE_HEADER
#define INSTANCE_HEADER

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <deque>
#include <math.h>
#include <map>
ILOSTLBEGIN

using namespace std;


class Instance {

public:



	// Number of nodes. Depot is always the first node
	int n_nodes;
	int real_n_nodes;
	int k_arc_damages;
	double s_drone;//speed of drone
	double const_service_time; //service_time of delivery nodes
	bool shortcuts;
	const double max_lim = std::numeric_limits<double>::infinity();
	vector<bool> customers_to_visit; //customers to visit
	vector<int> parking_nodes;
	std::map<int, double> service_time_map;

	vector<vector<double>> euclidean_distance_matrix;
	vector<vector<double>> manhattan_distance_matrix;
	vector<pair<double,double>> coordinates;
	vector<int> customer_nodes;
	vector<vector<int>> priovalues_arcs;//priovalues of the arc for the computation of the lower bound
	vector<vector<int>> arc_status; //initial arc status, 1 known undamaged, -1 known damaged, 0 unknown. If the arc does not exists --> -1.
	vector<vector<int>> arc_status_conservative;

	vector<vector<int>> adjacent_nodes; //list of adjacent node. On the first position --> the number of adjacent nodes

	vector<vector<double>> drone_distances;//shortest path distances for the drone (both ways, to and from the node). Computed after reading the input file
	vector<vector<int>> drone_fromtriple_previousnode;//shortest path distances for the drone (both ways, to and from the node). Computed after reading the input file

	int max_drone_sortie;//length of the max possible drone sortie
	int ins_number;// ID of the instance
	int maxn_stages_long;
	int maxn_stages_short;

public:

	/**
	* Constructor for the Instance class to build the object out of a data file.
	* @method Instance::Instance
	* @param  filename           Name of the data file.
	*/
	Instance(const string& filename, int ins_num, int k_arc_damages, double s_drone_speed, double const_service_time, bool shortcuts);
	Instance();
	void initialize_drone_distances();
	void initialize_customers_bool_and_const_delivery_service_times();



private:

	/**
	* Fill the object data structure from the given file.
	* @method fillFromFile
	* @param  filename     name of the data file
	*/
	void fillFromFile(const string& filename);
	void compute_prio_values_of_arcs();//for the computation of the lower bound
	void compute_maxn_stages();
	
};

#endif

