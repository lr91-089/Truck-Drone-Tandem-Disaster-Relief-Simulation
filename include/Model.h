#ifndef MODEL_HEADER
#define MODEL_HEADER

#include "Instance.h"
#include "Solution.h"
#include "ArtificialNode.h"
#include "Utilities.h"
#include <deque>
#include <set>
#include <unordered_map>
#include <iostream>
#include <map>
#include <math.h>
#include <limits>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <cmath>
#include <sstream>


using namespace std;

struct VectorHasher {
	int operator()(const vector<int>& V) const {
		int hash = V.size();
		for (auto& i : V) {
			hash ^= i + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};

typedef struct DP_tables {
	DP_tables(int _n_nodes, int customer_n_nodes, double max_lim, Instance instance) :
		subsets(std::vector<unordered_map<vector<int>, int, VectorHasher>>(customer_n_nodes + 1)),
		subsets_by_id{},
		distance_metric_naive(vector<vector<double>>(_n_nodes, vector<double>(_n_nodes, -1))),
		dtsp(vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)))),
		dtsp_pred(vector<vector<vector<pair<int, int>>>>(_n_nodes, vector<vector<pair<int, int>>>(_n_nodes, vector<pair<int, int>>(pow(2, customer_n_nodes), {-1, -1} )))),
		dtsp_op(vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)))),
		dtsp_drone(vector<vector<vector<int>>>(_n_nodes, vector<vector<int>>(_n_nodes, vector<int>(pow(2, customer_n_nodes), -1)))),
		dtsp_op_last_delivery(vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)))),
		dtsp_last_delivery_nodes(vector<vector<vector<pair<int, int>>>>(_n_nodes, vector<vector<pair<int, int>>>(_n_nodes, vector<pair<int, int>>(pow(2, customer_n_nodes),{-1,-1})))),
		reopt_drone_distances(instance.drone_distances),
		reopt_drone_fromtriple_previousnode(instance.drone_fromtriple_previousnode),
		service_time_map(instance.service_time_map),
		n_nodes(_n_nodes), 
		artificial_nodes(0),
		coordinates_reopt(instance.coordinates) {}
	vector<unordered_map<vector<int>, int, VectorHasher>> subsets;
	unordered_map<int, std::vector<int>> subsets_by_id;
	//truck distances
	vector<vector<double>> distance_metric_naive;
	vector<vector<vector<double>>> dtsp;
	vector<vector<vector<pair<int, int>>>> dtsp_pred;
	vector<vector<vector<double>>> dtsp_op;
	vector<vector<vector<int>>> dtsp_drone;
	vector<vector<vector<double>>> dtsp_op_last_delivery;
	vector<vector<vector<pair<int, int>>>> dtsp_last_delivery_nodes;
	vector<vector<double>> reopt_drone_distances;//shortest part distances for the drone (both ways, to and from the node). Computed after reading the input file
	vector<vector<int>> reopt_drone_fromtriple_previousnode;
	map<int,double> service_time_map;
	int n_nodes;
	int artificial_nodes;
	map<int,ArtificialNode> artificial_nodes_map;
	vector<pair<double,double>> coordinates_reopt;

	void reset_dp_struct(int _n_nodes, int customer_n_nodes, double max_lim) {
		this->subsets = std::vector<unordered_map<vector<int>, int, VectorHasher>>(customer_n_nodes + 1);
		this->subsets_by_id.clear();
		this->dtsp = vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)));
		this->dtsp_pred = vector<vector<vector<pair<int, int>>>>(_n_nodes, vector<vector<pair<int, int>>>(_n_nodes, vector<pair<int, int>>(pow(2, customer_n_nodes), { -1, -1 })));
		this->dtsp_op = vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)));
		this->dtsp_drone = vector<vector<vector<int>>>(_n_nodes, vector<vector<int>>(_n_nodes, vector<int>(pow(2, customer_n_nodes), -1)));
		this-> dtsp_op_last_delivery = vector<vector<vector<double>>>(_n_nodes, vector<vector<double>>(_n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim)));
		this->dtsp_last_delivery_nodes = vector<vector<vector<pair<int, int>>>>(_n_nodes, vector<vector<pair<int, int>>>(_n_nodes, vector<pair<int, int>>(pow(2, customer_n_nodes),{-1,-1})));
	};
}DP_TABLES;


typedef struct
{
    double cost;
    pair<int,long long int> predecessor;
} dp_value;


typedef struct Ledges { 
	int n_Ledges;//number of Ledges (truck possibly long edges)
	vector<vector<int>>Ledges_from_end_to_start;//for a more convenient ausgabe. Only truck nodes
	vector<double>list_of_Ledges;//list of Ledges: for each Ledge - its length
	vector<int>Ledges_dronenode;//ID of the drone node of the Ledge
	vector<vector<int>>outgoing_Ledges;//n of outgoing Ledges, the list of the numbers of the Ledges
	vector<vector<int>>incoming_Ledges;//n of incoming Ledges, the list of the numbers of the Ledges
	vector<pair<int, int>>replan_nodes_Ledge;
	vector<int> reopt_ct_ledges;
	vector<vector<int>> Ledges_customers;
	vector<vector<int>>nodes_on_Ledge;//the nodes on the Ledge: //node, ledge to which it belongs: only "middle" nodes
	Ledges(int n_nodes) : n_Ledges(0), Ledges_from_end_to_start(vector<vector<int>>(0, vector<int>(0))),
		list_of_Ledges(vector<double>(0)),
		Ledges_dronenode(vector<int>(0)),
		outgoing_Ledges(vector<vector<int>>(n_nodes, vector<int>(0))),
		incoming_Ledges(vector<vector<int>>(n_nodes, vector<int>(0))),
		replan_nodes_Ledge(vector<pair<int, int>>(0)),
		reopt_ct_ledges(vector<int>(0)),
		Ledges_customers(vector<vector<int>>(0, vector<int>(0))),
		nodes_on_Ledge(vector<vector<int>>(n_nodes, vector<int>(0)))
	{};
	void add_Ledge() { n_Ledges = n_Ledges + 1; };
}LEDGES;




class Model
{
public:
	const double max_lim = std::numeric_limits<double>::infinity();
	Instance& instance;
	Solution model_solution;
	int n_stages;
	vector<vector<int>>edge_status;
	bool conservative;
	bool manhattan_truck;
	vector<bool>customers_to_visit;
	vector<vector<int>> arc_status_after_surveillance; //if the arc was already visited in an optimization run
	string mistake_text;
	bool mistake_reported;
	vector<int> customer_set;
	int customer_n_nodes;
	int curr_truck_node;
	int curr_drone_node;
	int curr_drone_customer;
	int last_combined_node;
	int flag_reopt_start_idx = -1;
	double service_time;
	DP_tables dp;
	Ledges Ledges_struct;
	
	

public:
	Model(Instance& instance, vector<vector<int>>edge_status, vector<vector<int>>arc_status_after_surveillance, vector<bool>customers_to_visit,bool manhattan_truck=true, double service_time=0.0,int curr_truck_node=0, int curr_drone_node=0, int _curr_drone_cust=-1, int last_combined_node=0);
	bool check_solution(Solution& solution, vector<vector<int>>& current_real_damages);
	bool check_solution_tsp(Solution& solution, vector<vector<int>>& current_real_damages);
	bool check_solution_new_tsp(Solution& solution, vector<vector<int>>& current_real_damages);
    bool check_solution_survop(Solution &solution, vector<vector<int>> &current_real_damages, double cmax);
    void init_dp_struct( double penalty_factor,bool artificial_node = false);
    void init_distances(double penalty_factor=0.0);
	void init_subsets();
	void init_dp_table(bool artificial_node=false);
	void init_surveillance_dp_table(int drone_surveillance_start_node, set<int> truck_customers);
	void dp_eval(int v, int w, int u, int index1, int index2);
	//void dp_eval_tsp_surveillance(int w, int u, int index1, int index2);
	void dp_make_ops_steiner_thread(int v);
	void dp_make_ops_steiner_last_delivery_thread(int v);
	void dp_make_tsp(int v);
    //void dp_make_tsp_for_surveillance(int v,set<int> truck_customers);
    vector<vector<int>> get_all_subsets(vector<int> &base_set);
    void solve_operations(int settings);
	void conservative_calculation(int settings=1);
	void add_conservative_Loops();
	void write_Ledges();
	void write_solution();
    void reverseAllOperations(std::vector<std::vector<int>> &vec);
    int get_op_weight(const std::vector<int> &operations_vec);
    void write_solution_last_delivery();
    void reset_model(vector<vector<int>> _edge_status, vector<bool> _customers_to_visit, int _curr_truck_node, int _curr_drone_node, int _last_combined_node);
    void reset_model_dp_reopt(vector<vector<int>>_edge_status, vector<bool>_customers_to_visit, int _curr_truck_node, int _curr_drone_node, int _curr_drone_cust, int _last_combined_node);
	virtual vector<int> get_ledges_to_visit();
	vector<int> backtrack_path(int start, int goal, vector<vector<int>> previous_node, vector<int> op);
	bool areEqual(double a, double b);
    bool checkFirstAndLastEqual(const std::vector<std::vector<int>> &operations_vec);
    string write_number(double iteration);
    void backtrack_ledges();
	vector<int>init_customer_set();
	void add_artificial_truck_node(ArtificialNode artificial_node);
	void add_edge_of_artificial_node_to_dp_table(pair<pair<int,int>,double> edge_to_add);
	void add_artificial_drone_node(ArtificialNode artificial_node);
	void add_artificial_drone_node_sichtfeld(ArtificialNode artificial_node, pair<pair<int,int>,double> truck_edge_to_add);
	void add_edge_of_artificial_node_to_drone_reopt_dist(pair<pair<int,int>,double> edge_to_add);
	void add_real_incoming_edges(ArtificialNode artificial_node, bool truck_node);


private:
	
	
};

#endif