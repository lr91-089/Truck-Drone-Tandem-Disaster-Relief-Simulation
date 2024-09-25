#ifndef SIMULATION_HEADER
#define SIMULATION_HEADER

#include "Solution.h"
#include "Instance.h"
#include "ArtificialNode.h"
#include "Model_EULER.h"
#include "Model_DP.h"
#include "Model_tsp.h"
#include "Model_op.h"
#include "Model.h"
#include "Utilities.h"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <time.h>
#include <ilcplex/ilocplex.h>
#include <deque>
#include <set>
#include <tuple>
#include <random>
#include <cmath>
#include <chrono>
#include <utility>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <math.h>
#include <cmath>
#include <sstream>



ILOSTLBEGIN

using namespace std;



class Simulation {
public:
	Instance& instance;
	cmd_inputs_struct cmd_inputs;
	int n_iter;//number iterations
	vector<bool> temp_customers_to_visit;//initial customers that have to be visited (all)
	vector<int> current_seed;
	int ranseed;
	mt19937_64 random_engine_generator;
	



	typedef struct TIteration {
		int n_damaged_arcs;//current number of damaged arcs
		vector<vector<int>> current_damages; //filled in in the beginning of the simulation: info on the actually damaged arcs
		vector<vector<int>> truck_arcs_fullinfo; //sets all truck arcs planned in an optimal tour to 1
		vector<vector<int>> truck_arcs_surv; //sets all truck arcs planned in an optimal tour to 1
		vector<vector<int>> truck_arcs_surv_op; //sets all truck arcs planned in an optimal tour to 1
		vector<vector<int>> truck_arcs_reopt; //sets all truck arcs planned in an optimal tour to 1
		double full_info_obj;
		double time_fullinfo;
		double conservative_obj;
		double time_conservative;
		double survfirst_surveillance_tour_cost;
		double survfirst_obj;
		set<int> surveillance_nodes;
		double time_surveillance;
		double survop_surveillance_tour_cost;
		double survop_obj;
		set<int> surveillance_op_nodes;
		double time_surveillance_op;
		double time_reopt;
		double reopt_obj;
		double dyn_cmax;
		bool naive_saved = false;
		string mistake_text = "";
		bool mistake_reported = false;
		int n_replan_surveillance_first;
		int n_replan_surveillance_op_survphase;
		int n_replan_surveillance_op_reoptphase;
		int n_replan_reopt_last_operation;
		int n_replan_survop_reopt_last_operation;
		int tilim = 1800;
		int current_iteration;
		long ranseed;
		sim_results_struc truck_time_used;
		sim_results_struc drone_time_used_without_sorties;
		sim_results_struc drone_sortie_time;
		sim_results_struc truck_wait;
		sim_results_struc drone_wait;
		sim_results_struc truck_drone_tandem_time_used;
		sim_results_struc_int damaged_edges_in_solution;
		sim_results_struc_int operations_num;
		sim_results_struc_int drone_on_truck_operations_num;
		sim_results_struc_vec truck_op_times;
		sim_results_struc_vec drone_op_times;
		sim_results_struc_vec truck_op_wait_times;
		sim_results_struc_vec drone_op_wait_times;
		sim_results_struc_vec drone_op_times_with_wait_time;
		int optimistic_sol_operations=0;
		double optimistic_initial_solution = 0.0;
		//vector<double>delivery_cost = vector<double>(4);
	}TITERATION;

	typedef struct Tlistofarcs {
		int first_node;
		int second_node;
		int prio_value;
	}TLISTOFARCS;

	reoptParams reopt_parameters;
	reoptParams surv_op_reopt_parameters;
	vector<TITERATION> iterations;

public:
	Simulation(Instance& instance, cmd_inputs_struct cmd_inputs);


private:
	void create_current_instance();//create an instance by generating arc damages
	void create_test_instance();//create an instance by generating arc damages
	void init_simulation();
	void run_full_information_model(int simulation_setting);
	void run_surv_first(int simulation_setting);
	void run_surv_op(int simulation_setting);
	void run_reopt(int simulation_setting);
	void run_conservative(int simulation_setting);
	void weighted_sample();
	void cluster_damage();
	bool reoptA_dp(vector<vector<int>>arcs_status_after_surveillance, Solution& delivery_sol, int simulation_setting);
	std::pair<bool, bool> check_replan_reopt(int loc_i, Solution& delivery_solution, vector<bool>& loc_temp_customers_to_visit);
	std::pair<bool, bool> check_replan_reopt_sichtfeld(int loc_i, Solution& delivery_solution, vector<bool>& loc_temp_customers_to_visit);
	bool check_edges(int curr_node_position, Solution& delivery_solution_model, vector<vector<int>> &arcs_status_after_surveillance, vector<bool> &whether_the_node_visited);
	bool check_edges_sichtfeld_drone(int curr_node_position, Solution& delivery_solution_model, vector<vector<int>> &arcs_status_after_surveillance, vector<bool> &whether_the_node_visited);
	double simulate_reopt_truck_walk(int loc_i, Solution& delivery_sol, Solution& delivery_solution_model, vector<vector<int>>& arcs_status_after_surveillance, vector<bool>& loc_temp_customers_to_visit, vector<bool>& whether_the_node_visited, vector<bool>& whether_the_customer_node_visited,reoptParams& loc_reopt_params);
	double simulate_reopt_drone_walk(int loc_i, Solution& delivery_sol, Solution& delivery_solution_model, vector<vector<int>>& arcs_status_after_surveillance, vector<bool>& temp_customers_to_visit, vector<bool>& whether_the_node_visited, vector<bool>& whether_the_customer_node_visited,reoptParams& loc_reopt_params);
	bool surveillance_first_naive(vector<vector<int>>& arc_status_after_surveillance, double& cost_of_surveillance, double& cost_of_delivery, Solution& delivery_sol, Solution& surveillance_solution, int surveillance_first_setting, bool arc_based_tsp); //default value arc_based_tsp=false
	bool surveillance_first_orienteering_variant(vector<vector<int>>& arc_status_after_surveillance, double& cost_of_surveillance, double& cost_of_delivery, Solution& delivery_sol, Solution& surveillance_solution, int simulation_setting, double cmax, double penalty_factor, bool arc_based_tsp); //default value arc_based_tsp=true
	void ausgabe_result_file();
	void ausgabe_tour(Solution curr_solution, string solution_ID, bool checked_and_status);
	void check_solution_tsp(Solution input_solution, bool& result);//just drone sorties
	void check_solution_surveillance(Solution input_solution, bool& result);
    void check_solution_surveillance_op(Solution input_solution, bool &result);
    void ausgabe_tour_surveillance(Solution curr_solution, string solution_ID, bool checked_and_status);
	void update_planned_truck_arcs(int casecode, vector<vector<int>> truck_ledges);
	string create_method_string();
	string write_number(double num, int precision);
	string convert_time_to_str(double duration, int precisionVal);
	std::string vecToString(const std::vector<double>& vec);
	bool areEqual(double a, double b);

};

#endif