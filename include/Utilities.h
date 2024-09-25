#ifndef UTILITIES_HEADER
#define UTILITIES_HEADER

#include <string>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include "ArtificialNode.h"


typedef struct sim_results_struc {
		double fullinfo=0.0;
		double surv=0.0;
		double surv_op=0.0;
		double reopt=0.0;
	}SIM_RESULTS_STRUC;

typedef struct sim_results_struc_vec {
		vector<double> fullinfo=vector<double>();
		vector<double> surv=vector<double>();
		vector<double> surv_op=vector<double>();
		vector<double> reopt=vector<double>();
		vector<double> conservative=vector<double>();
	}SIM_RESULTS_STRUC_VEC;

typedef struct sim_results_struc_int {
		int fullinfo=0;
		int surv=0;
		int surv_op_delivery=0;
		int surv_op_surveillance=0;
		int reopt=0;
	}SIM_RESULTS_STRUC_INT;

typedef struct artificial_truck_node_sichtfeld {
		int art_node=0;
		int to_node=0;
		double distance=0.0;
	}ARTIFICIAL_TRUCK_NODE_SICHTFELD;


typedef struct reoptParams {
	int n_replan_reopt;
	int n_visited_nodes;//n visited nodes in the stage
	int last_combined_node;
	int curr_truck_node;
	int curr_drone_node;
	int curr_drone_cust;
	double drone_duration;
	double truck_duration;
	bool replan_reopt_truck;
	bool replan_reopt_drone;
	bool fix_operation;
	int next_atificial_node_id;
	vector<vector<double>> simulated_truck_distance_traversal;
	ArtificialNode artificial_node;
	reoptParams() :
		n_replan_reopt(-1),
		n_visited_nodes(0),//n visited nodes in the stage
		last_combined_node(0),
		curr_truck_node(0),
		curr_drone_node(0),
		curr_drone_cust(-1),
		drone_duration(0),
		truck_duration(0),
		replan_reopt_truck(true),
		replan_reopt_drone(true),
		fix_operation(false),
		next_atificial_node_id(0),
		simulated_truck_distance_traversal(0),
		artificial_node(ArtificialNode())
		{}
}REOPTPARAMS;

typedef struct cmd_inputs_struct {
	int number_inputs;
	int n_simulations;
	int input_k_arc_damages;
	long ranseed;
	int simulation_settings[6];
	int ins_num;
	double input_clustered_damages;
	double input_required_nodes_proportion;
	double input_howmuch_drone_is_faster;
	double input_parking_nodes;
	double cmax;
	double cmax_ratio;
	double penalty_factor;
	double service_time;
	std::string input_filename;
	std::string path_to_folder;
	std::string path_to_results;
	std::string path_to_data;
	std::string zusatz_ausgabe_filename;
	bool shortcuts;
	bool truck_distance_metric_manhattan;
	int setting_surv_op;
	int setting_full_info;
	int setting_surv_first;
	int setting_reopt;
	int setting_conservative;
	bool setting_debug = false;
	int setting_sichtfeld=0; //0 is off, 1 is uniform, 2 is midpoint, 3 is one fourth 
}CMD_INPUTS_STRUCT;

// Trim from left
inline std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v") {
	s.erase(0, s.find_first_not_of(t));
	return s;
}

// Trim from right
inline std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v") {
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}

// Trim from left & right
inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
	return ltrim(rtrim(s, t), t);
}

//bool to string
inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}



#endif
