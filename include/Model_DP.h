#ifndef MODEL_DP_HEADER
#define MODEL_DP_HEADER

#include "Instance.h"
#include "Model.h"
#include "Model_EULER.h"
#include "Utilities.h"
#include <deque>
#include <set>
#include <unordered_map>
#include <iostream>
#include <map>
#include <math.h>
#include <iterator>


using namespace std;


class Model_DP: public Model {
public:
	int curr_position=0;
	int number_of_nodes;
	vector<vector<double>>d_table_operations_graph;
	vector<vector<tuple<int, int, int>>>prev_d_operations_graph;
	Solution reopt_solution;
	string mistake_text = "";
	bool mistake_reported = false;
	double penalty_factor;

private:
	reoptParams reopt_parameters;

public:
	//NOTE: curr_position - is position of a vehicle AND a drone. Hence, replanning 
	//n_stage : number of the stages, which influence the number of decision variables in the model
	Model_DP(Instance& _instance, vector<vector<int>> _edge_status, vector<vector<int>>_arc_status_after_surveillance, int _curr_position, vector<bool> _customers_to_visit, bool _manhattan_truck, double service_time=0.0, double penalty_factor=0.0);
	void solve(int setting=1);
	void solve_reopt(vector<vector<int>> _edge_status,vector<vector<int>>_arc_status_after_surveillance, vector<bool> _customers_to_visit, reoptParams _reopt_parameters, int simulation_setting);
	void write_solution_dp_reopt();
    void write_solution_dp_reopt_last_delivery();
	void init_reopt_model_struct();
	void add_real_incoming_edges_after_solving();


private:
	double evaluate_artificial_truck_op(int curr_truck_node, int w, int artificial_node, int index, double truck_distance);
	void putsolutionOnScreenDP();
	void read_solution_complete_dp(bool surv = false);
	double evaluate_reopt_distance(int v, int w, vector<vector<double>>& distance);
	vector<int> get_ledges_to_visit();
	void dp_operations_graph(int v);
    void dp_operations_graph_last_delivery(int v);
    void make_operations_graph(int setting = 1);
    void backtrack_ledges_dp(int last_node=0, int simulation_setting=1);
	void backtrack_op_dp2(int u, int w, vector<int> curr_s);
    void backtrack_last_delivery_op_dp2(int u, int w, vector<int> curr_s);
    double get_last_delivery_objective();
	//void adjust_artificial_edges();
	
};




#endif