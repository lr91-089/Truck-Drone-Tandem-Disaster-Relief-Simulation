#include "Simulation.h"
namespace fs = std::filesystem;

/*

std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[�s]" << std::endl;

*/


Simulation::Simulation(Instance& _instance,cmd_inputs_struct _cmd_inputs) :
	instance(_instance), cmd_inputs(_cmd_inputs),n_iter(cmd_inputs.n_simulations), temp_customers_to_visit(vector<bool>(0))
{
	init_simulation();
	
	fs::create_directory(cmd_inputs.path_to_results);
	for (int curr_iter = 0; curr_iter < n_iter; curr_iter++) {
		iterations.push_back(TITERATION{});
		if (cmd_inputs.ranseed == 0) {
			unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
			iterations.back().ranseed = seed1;
		}
		else {
			//iterations.back().ranseed = cmd_inputs.ranseed+curr_iter+cmd_inputs.ins_num+cmd_inputs.input_k_arc_damages+cmd_inputs.input_clustered_damages;
			iterations.back().ranseed = cmd_inputs.ranseed;
			
		}
		//int input_adjusted_seed = iterations.back().ranseed+curr_iter+cmd_inputs.ins_num+cmd_inputs.input_k_arc_damages+cmd_inputs.input_clustered_damages;
		random_engine_generator.seed(iterations.back().ranseed);
		//iterations.back().ranseed = cmd_inputs.ranseed + ((curr_iter+1)*(cmd_inputs.input_clustered_damages+1+cmd_inputs.input_howmuch_drone_is_faster*100+cmd_inputs.input_k_arc_damages+cmd_inputs.input_required_nodes_proportion*100));
		//iterations.back().ranseed = cmd_inputs.ranseed + ((curr_iter*(cmd_inputs.input_clustered_damages+1)) * (cmd_inputs.input_howmuch_drone_is_faster*100+cmd_inputs.input_k_arc_damages+cmd_inputs.input_required_nodes_proportion*100));
		iterations.back().current_iteration = curr_iter + 1;
		cout << "----------------------------------------------------------------------------------------\n";
		cout << cmd_inputs.ins_num <<" "<<curr_iter+1 << " " << cmd_inputs.input_k_arc_damages<<" "<< cmd_inputs.input_clustered_damages << " " << cmd_inputs.input_required_nodes_proportion <<" "<<cmd_inputs.input_howmuch_drone_is_faster <<" "<<cmd_inputs.cmax << " Shortcuts:" << BoolToString(cmd_inputs.shortcuts)  <<" TruckManhattan:" << BoolToString(cmd_inputs.truck_distance_metric_manhattan) << " " << iterations.back().ranseed << " " << cmd_inputs.input_parking_nodes  << " " <<  cmd_inputs.penalty_factor << cmd_inputs.cmax_ratio << "\n";
		cout << "----------------------------------------------------------------------------------------\n";
		try{
			create_current_instance();
		}
		catch (const std::exception &e) {
			cerr << "ERROR OCCURED!" << e.what() << "\n";	
			iterations.back().mistake_text = e.what();
			ausgabe_result_file();
		};
		iterations.back().truck_arcs_fullinfo = vector<vector<int>>(instance.n_nodes,vector<int>(instance.n_nodes,-1));
		iterations.back().truck_arcs_surv = vector<vector<int>>(instance.n_nodes,vector<int>(instance.n_nodes,-1));
		iterations.back().truck_arcs_surv_op = vector<vector<int>>(instance.n_nodes,vector<int>(instance.n_nodes,-1));
		iterations.back().truck_arcs_reopt = vector<vector<int>>(instance.n_nodes,vector<int>(instance.n_nodes,-1));
		if(cmd_inputs.setting_full_info==3){
			for(int settingx=1;settingx<3;settingx++){
						try{
							//create_test_instance();
							cmd_inputs.setting_full_info = settingx;
							cmd_inputs.setting_surv_first = settingx;
							cmd_inputs.setting_reopt = settingx;
							cmd_inputs.setting_conservative = settingx;
							cmd_inputs.setting_surv_op = settingx;
							run_full_information_model(cmd_inputs.setting_full_info);
							run_surv_first(cmd_inputs.setting_surv_first);
							run_surv_op(cmd_inputs.setting_surv_op);
							run_reopt(cmd_inputs.setting_reopt);
							run_conservative(cmd_inputs.setting_conservative);
						}
						catch (const std::exception &e) {
							cerr << "ERROR OCCURED!" << e.what() << "\n";	
							iterations.back().mistake_text = e.what();
							ausgabe_result_file();
						};
						ausgabe_result_file();
			} 
		}
		else{
			try{
			
			//create_test_instance();
			
			run_full_information_model(cmd_inputs.setting_full_info);
			run_surv_first(cmd_inputs.setting_surv_first);
			run_surv_op(cmd_inputs.setting_surv_op);
			run_reopt(cmd_inputs.setting_reopt);
			run_conservative(cmd_inputs.setting_conservative);
			}
			catch (const std::exception &e) {
				cerr << "ERROR OCCURED!" << e.what() << "\n";	
				iterations.back().mistake_text = e.what();
				ausgabe_result_file();
			};
			ausgabe_result_file();
		} 

	}
}

void Simulation::create_current_instance() {
	iterations.back().current_damages.resize(0);
	//CHECKIING MODUS

	/*
	vector<pair<int, int>> random_edge_removal_vec;
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = loc_i+1; loc_j < instance.n_nodes; loc_j++) {
			if (instance.arc_status[loc_i][loc_j] > -1){
				random_edge_removal_vec.push_back({ loc_i,loc_j });
			}
		}
	}
	int remove_edges = int(round(0.7*random_edge_removal_vec.size()));
	std::shuffle(std::begin(random_edge_removal_vec), std::end(random_edge_removal_vec), random_engine_generator);

	for (int k = 0; k< remove_edges; k++){
			pair<int, int> edge = random_edge_removal_vec[k];
			instance.arc_status[edge.first][edge.second] = -1;
			instance.arc_status[edge.second][edge.first] = -1;
	}
	if(cmd_inputs.shortcuts==false){
		instance.initialize_drone_distances();
	}
	*/

	std::uniform_real_distribution<double> loc_distribution(0.0, 1.0);
	

	iterations.back().n_damaged_arcs = 0;
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		iterations.back().current_damages.push_back(instance.arc_status[loc_i]);
	}
	if ((int)cmd_inputs.input_clustered_damages == 0) {
		//arc damages by number k
		//std::uniform_real_distribution<double> loc_distribution(0.0, 1.0);
		vector<pair<int, int>> random_choice_vec;
		for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
			for (int loc_j = loc_i+1; loc_j < instance.n_nodes; loc_j++) {
				if (instance.arc_status[loc_i][loc_j] > -1){
					random_choice_vec.push_back({ loc_i,loc_j });
				}
			}
		}
		instance.real_n_nodes = instance.n_nodes;
		std::shuffle(std::begin(random_choice_vec), std::end(random_choice_vec), random_engine_generator);
		for (int k = 0; k< cmd_inputs.input_k_arc_damages; k++){
			if (k<(int)random_choice_vec.size()){
				pair<int, int> edge = random_choice_vec[k];
				iterations.back().n_damaged_arcs = iterations.back().n_damaged_arcs + 1;
				iterations.back().current_damages[edge.first][edge.second] = -1;
				iterations.back().current_damages[edge.second][edge.first] = -1;
				if(cmd_inputs.setting_sichtfeld>0) {
					double randnum = loc_distribution(random_engine_generator);
					int a = instance.n_nodes;
					int b = instance.n_nodes+1;
					instance.n_nodes = instance.n_nodes+2;
					int new_size = instance.n_nodes;
					for (int m=0;m<2;m++) {
						for (int i = 0; i < (int)instance.arc_status.size(); i++) {
							instance.arc_status[i].push_back(-1);
							instance.arc_status_conservative[i].push_back(-1);
							instance.adjacent_nodes[i].push_back(-1);
							instance.euclidean_distance_matrix[i].push_back(std::numeric_limits<double>::infinity());
							instance.manhattan_distance_matrix[i].push_back(std::numeric_limits<double>::infinity());
							instance.drone_fromtriple_previousnode[i].push_back(-1);
							iterations.back().current_damages[i].push_back(-1);
						}
						instance.arc_status.push_back(vector<int>(new_size, -1));
						instance.arc_status_conservative.push_back(vector<int>(new_size, -1));
						instance.adjacent_nodes.push_back(vector<int>(new_size, -1));
						instance.euclidean_distance_matrix.push_back(vector<double>(new_size, std::numeric_limits<double>::infinity()));
						instance.manhattan_distance_matrix.push_back(vector<double>(new_size, std::numeric_limits<double>::infinity()));
						instance.drone_fromtriple_previousnode.push_back(vector<int>(new_size, -1));
						iterations.back().current_damages.push_back(vector<int>(new_size, -1));
					}
					//initialize new edges
					instance.arc_status[edge.first][edge.second] = -1;
					instance.arc_status[edge.second][edge.first] = -1;
					//update arc status
					instance.arc_status[edge.first][a] = 1;
					instance.arc_status[a][b] = 1;
					instance.arc_status[b][edge.second] = 1;
					instance.arc_status[a][edge.first] = 1;
					instance.arc_status[b][a] = 1;
					instance.arc_status[edge.second][b] = 1;
					//initialize damages
					iterations.back().current_damages[edge.first][a] = 1;
					iterations.back().current_damages[a][b] = 1;
					iterations.back().current_damages[b][edge.second] = 1;
					iterations.back().current_damages[a][edge.first] = 1;
					iterations.back().current_damages[b][a] = 1;
					iterations.back().current_damages[edge.second][b] = 1;
					//initialize euclidean distances
					double divisor_a = 3;
					double divisor_ab = 3;
					double divisor_b = 3;
					switch (cmd_inputs.setting_sichtfeld) {
						case 1:
							divisor_a = 3;
							divisor_ab = 3;
							divisor_b = 3;
							break;
						case 2:
							divisor_a = 2;
							divisor_ab = 0;
							divisor_b = 2;
							break;
						case 3:
							divisor_a = 4;
							divisor_ab = 2;
							divisor_b = 4;
							break;
					}
					
					instance.euclidean_distance_matrix[edge.first][a] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_a;
					if(divisor_ab>0.0) {
						instance.euclidean_distance_matrix[a][b] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_ab;
					}
					else{
						instance.euclidean_distance_matrix[a][b] = 0.0;
					}
					instance.euclidean_distance_matrix[b][edge.second] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_b;
					instance.euclidean_distance_matrix[a][edge.first] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_a;
					if(divisor_ab>0.0) {
						instance.euclidean_distance_matrix[b][a] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_ab;
					}
					else{
						instance.euclidean_distance_matrix[b][a] = 0.0;
					}
					instance.euclidean_distance_matrix[edge.second][b] = (double)instance.euclidean_distance_matrix[edge.first][edge.second]/divisor_b;
					//coordinates
					double temp_dist = instance.euclidean_distance_matrix[edge.first][edge.second];
					double x_a = (((double)instance.euclidean_distance_matrix[edge.first][a])*instance.coordinates[edge.second].first+((double)instance.euclidean_distance_matrix[a][b]+(double)instance.euclidean_distance_matrix[b][edge.second])*instance.coordinates[edge.first].first)/temp_dist;
					double y_a = (((double)instance.euclidean_distance_matrix[edge.first][a])*instance.coordinates[edge.second].second+((double)instance.euclidean_distance_matrix[a][b]+(double)instance.euclidean_distance_matrix[b][edge.second])*instance.coordinates[edge.first].second)/temp_dist;
					double x_b = (((double)instance.euclidean_distance_matrix[a][b]+(double)instance.euclidean_distance_matrix[b][edge.second])*instance.coordinates[edge.second].first+((double)instance.euclidean_distance_matrix[edge.first][a])*instance.coordinates[edge.first].first)/temp_dist;
					double y_b = (((double)instance.euclidean_distance_matrix[a][b]+(double)instance.euclidean_distance_matrix[b][edge.second])*instance.coordinates[edge.second].second+((double)instance.euclidean_distance_matrix[edge.first][a])*instance.coordinates[edge.first].second)/temp_dist;
					instance.coordinates.push_back({x_a,y_a});
					instance.coordinates.push_back({x_b,y_b});
					//initialize manhattan distances
					instance.manhattan_distance_matrix[edge.first][a] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_a;
					if(divisor_ab>0.0) {
						instance.manhattan_distance_matrix[a][b] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_ab;
					}
					else {
						instance.manhattan_distance_matrix[a][b] = 0.0;
					}
					instance.manhattan_distance_matrix[b][edge.second] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_b;
					instance.manhattan_distance_matrix[a][edge.first] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_a;
					if(divisor_ab>0.0) {
						instance.manhattan_distance_matrix[b][a] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_ab;
					}
					else{
						instance.manhattan_distance_matrix[b][a] = 0.0;
					}
					instance.manhattan_distance_matrix[edge.second][b] = (double)instance.manhattan_distance_matrix[edge.first][edge.second]/divisor_b;
					//predecessors
					iterations.back().current_damages[edge.first][a] = 1;
					iterations.back().current_damages[a][b] = 1;
					iterations.back().current_damages[b][edge.second] = 1;
					iterations.back().current_damages[a][edge.first] = 1;
					iterations.back().current_damages[b][a] = 1;
					iterations.back().current_damages[edge.second][b] = 1;
					switch (cmd_inputs.setting_sichtfeld) {
						// uniform damage probability for all 3 secions
						case 1:
							if(randnum< (double)1/3) {
								//segment 1
								iterations.back().current_damages[edge.first][a] = -1;
								iterations.back().current_damages[a][edge.first] = -1;
							}
							else if(randnum>= (double)1/3 && randnum< (double)2/3) {
								//segment 2
								iterations.back().current_damages[a][b] = -1;
								iterations.back().current_damages[b][a] = -1;
							}
							else{//(randum>=2/3) {
								//segment 3
								iterations.back().current_damages[b][edge.second] = -1;
								iterations.back().current_damages[edge.second][b] = -1;
							};
							break;
						// always damage in the middle section
						case 2:
							iterations.back().current_damages[a][b] = -1;
							iterations.back().current_damages[b][a] = -1;
							break;
						// always damage in the middle section
						case 3:
							iterations.back().current_damages[a][b] = -1;
							iterations.back().current_damages[b][a] = -1;
							break;
					}
				}
			}
			else{
				break;
			}
		}
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
					if ((instance.arc_status[loc_i][loc_j] == 0 && iterations.back().current_damages[loc_i][loc_j] > -1) || loc_i == loc_j) {
						iterations.back().current_damages[loc_i][loc_j] = 1;
						iterations.back().current_damages[loc_j][loc_i] = 1;
				};
			};
		};
		if(cmd_inputs.setting_sichtfeld>0) {
			instance.adjacent_nodes.resize(0);
			vector<int> temp_vector(instance.n_nodes, -1);
			temp_vector.push_back(-1);
			temp_vector[0] = 0;
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				instance.adjacent_nodes.push_back(temp_vector);
			};
			// fill_in the list of adjacent nodes
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
					if (instance.arc_status[loc_i][loc_j] >= 0) {
						instance.adjacent_nodes[loc_i][0] = instance.adjacent_nodes[loc_i][0] + 1;
						int temp_int = instance.adjacent_nodes[loc_i][0];
						instance.adjacent_nodes[loc_i][temp_int] = loc_j;
					};
				}
			}
			instance.initialize_drone_distances();
			instance.initialize_customers_bool_and_const_delivery_service_times();
		}
	}
	else {
		//weighted_sample();
		cluster_damage();
	};
	//set the arc status to unknown
	for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
			for (int loc_j = 1; loc_j < instance.n_nodes; loc_j++) {
				if (instance.arc_status[loc_i][loc_j] > -1){
					instance.arc_status[loc_i][loc_j] = 0;
				}
			}
	}
	//parking nodes
	/*
	vector<int> temp_parking_nodes(0);
	vector<int> random_choice_vec(instance.n_nodes-1);
	std::iota (std::begin(random_choice_vec), std::end(random_choice_vec), 1);
	std::shuffle(std::begin(random_choice_vec), std::end(random_choice_vec), random_engine_generator);
	int parking_number = int(round((instance.n_nodes-1)*cmd_inputs.input_parking_nodes));
	for (int loc_i = 0; loc_i < parking_number; loc_i++) {
		int temp_customer = random_choice_vec[loc_i];
		temp_parking_nodes.push_back(temp_customer);
	}
	//required customers	
	
	random_choice_vec.resize(0);
	for(int loc_k=1;loc_k<instance.n_nodes;loc_k++) {
		if(std::find(temp_parking_nodes.begin(), temp_parking_nodes.end(), loc_k) != temp_parking_nodes.end()) {
			random_choice_vec.push_back(loc_k);
		}
	}
	//add depot node
	temp_parking_nodes.push_back(0);
	sort(temp_parking_nodes.begin(), temp_parking_nodes.end()); 
	instance.parking_nodes = temp_parking_nodes;
	std::shuffle(std::begin(random_choice_vec), std::end(random_choice_vec), random_engine_generator);
	int customer_number = int(round((instance.n_nodes-1)*cmd_inputs.input_required_nodes_proportion));
	*/
	// drone can land on every node
	if(cmd_inputs.input_parking_nodes==1.0) {
		std::vector<int> temp_parking_nodes(instance.n_nodes) ;
		std::iota (std::begin(temp_parking_nodes), std::end(temp_parking_nodes), 0);
		instance.parking_nodes = temp_parking_nodes;
	}
	// drone can only land on customer nodes and depot
	else if(cmd_inputs.input_parking_nodes==cmd_inputs.input_required_nodes_proportion) {
		instance.parking_nodes.resize(0);
		instance.parking_nodes.push_back(0);
		instance.parking_nodes.insert(instance.parking_nodes.end(), instance.customer_nodes.begin(), instance.customer_nodes.end());
	}
	//other cases not defined and sets are just read from file
	//read out customer nodes
	temp_customers_to_visit = instance.customers_to_visit;
};

void Simulation::create_test_instance() {
	iterations.back().current_damages.resize(0);

	std::uniform_real_distribution<double> loc_distribution(0.0, 1.0);

	iterations.back().n_damaged_arcs = 0;
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		iterations.back().current_damages.push_back(instance.arc_status[loc_i]);
	}
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			if ((instance.arc_status[loc_i][loc_j] == 0 && iterations.back().current_damages[loc_i][loc_j] > -1) || loc_i == loc_j) {
				iterations.back().current_damages[loc_i][loc_j] = 1;
				iterations.back().current_damages[loc_j][loc_i] = 1;
			}
		}
	}
	iterations.back().n_damaged_arcs = iterations.back().n_damaged_arcs + 1;
	iterations.back().current_damages[3][4] = -1;
	iterations.back().current_damages[4][3] = -1;
	iterations.back().current_damages[7][6] = -1;
	iterations.back().current_damages[6][7] = -1;

	//required customers
	temp_customers_to_visit = vector<bool>(instance.n_nodes, false);
	temp_customers_to_visit[1] = true;
	temp_customers_to_visit[2] = true;
	instance.customers_to_visit = temp_customers_to_visit;
}
void Simulation::init_simulation()
{
	current_seed.resize(0);
	iterations = vector<TITERATION>(0);

	temp_customers_to_visit.resize(instance.n_nodes, true);
	temp_customers_to_visit[0] = false;

	//Check, whether all initial edges are known. If not --> collect information on them
	for (int loc_i = 0; loc_i < instance.adjacent_nodes[0][0]; loc_i++) {
		if (instance.arc_status[0][instance.adjacent_nodes[0][loc_i + 1]] == 0) {
			instance.arc_status[0][instance.adjacent_nodes[0][loc_i + 1]] = 1;
			instance.arc_status[instance.adjacent_nodes[0][loc_i + 1]][0] = 1;
		};
	};
}
void Simulation::run_full_information_model(int simulation_setting)
{
	if (simulation_setting > 0 && simulation_setting < 3) {
		//find first solution in the setting
		//time(&cmd_inputs.zstart);
		auto zstart = std::chrono::steady_clock::now();
		bool solution_OK = true;
		vector<bool>loc_temp_customers_to_visit = instance.customers_to_visit;
		loc_temp_customers_to_visit[0] = false;
		Solution full_info_sol(0,instance.n_nodes);
		//Model model(instance, cmd_inputs.maxn_stages_long, iterations[loc_iter].current_damages, 0, temp_customers_to_visit, full_info_sol, false);
		Model_DP model_DP(instance, iterations.back().current_damages, iterations.back().current_damages, 0, loc_temp_customers_to_visit, cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time);
		//model_DP.solve();
		model_DP.solve(simulation_setting);
		solution_OK = model_DP.check_solution(model_DP.model_solution, iterations.back().current_damages);
		int casecode = 1; 
		update_planned_truck_arcs(casecode,model_DP.Ledges_struct.Ledges_from_end_to_start);
		full_info_sol = model_DP.model_solution;
		//get truck and drone distances
		iterations.back().truck_time_used.fullinfo = full_info_sol.get_route_travel_time(true);
		iterations.back().drone_time_used_without_sorties.fullinfo = full_info_sol.get_route_travel_time(false);
		iterations.back().drone_sortie_time.fullinfo = full_info_sol.get_route_travel_time(false,true);
		iterations.back().truck_drone_tandem_time_used.fullinfo = full_info_sol.get_route_tandem_travel_time();
		//get truck and drone waiting times
		iterations.back().truck_wait.fullinfo = full_info_sol.get_route_waiting_time(true);
		iterations.back().drone_wait.fullinfo = full_info_sol.get_route_waiting_time(false);
		//cout << "test";
		iterations.back().mistake_reported = full_info_sol.mistake_reported;
		iterations.back().mistake_text = iterations.back().mistake_text+full_info_sol.mistake_text;
		iterations.back().full_info_obj = full_info_sol.obj_value;
		iterations.back().operations_num.fullinfo = (int) full_info_sol.truck_sorties.size();
		iterations.back().drone_on_truck_operations_num.fullinfo = full_info_sol.get_drone_riding_truck_ops();
		//time(&cmd_inputs.zend);
		auto zend = std::chrono::steady_clock::now();
		iterations[0].time_fullinfo = std::chrono::duration_cast<std::chrono::duration<double>>(zend - zstart).count();
		string sol_id = "_";
		switch(simulation_setting) {
			case 1:
				sol_id = sol_id+"ct";
				break;
			case 2:
				sol_id = sol_id+"ld";
				break;
			default:
				break;
		}	
		ausgabe_tour(full_info_sol, "fullinfo"+sol_id, solution_OK);
	};
}

void Simulation::run_surv_first(int simulation_setting)
{
	if (simulation_setting > 0 && simulation_setting < 3) {
		auto zstart = std::chrono::steady_clock::now();
		string output_name;
		Solution survfirst_delivery(0,instance.n_nodes);
		Solution survfirst_surveillance(0, instance.n_nodes);
		vector<vector<int>>arcs_status_surveillancenaive;
		double cost_of_surveillance;	// also initialized in adp, but unused right now
		double cost_of_delivery;		// also initialized in adp, but unused right now
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			arcs_status_surveillancenaive.push_back(instance.arc_status[loc_i]);
		};
		
		//time(&cmd_inputs.zstart);
		bool solution_OK;
		//change if you want to run arc based tsp
		bool arc_based_tsp = false;		
		solution_OK = surveillance_first_naive(arcs_status_surveillancenaive, cost_of_surveillance, cost_of_delivery, survfirst_delivery, survfirst_surveillance, simulation_setting, arc_based_tsp);
		output_name = "survfirst_";
		
		iterations.back().survfirst_surveillance_tour_cost = cost_of_surveillance;
		iterations.back().survfirst_obj = cost_of_delivery;
		//time(&cmd_inputs.zend);
		//iterations.back().time_surveillance = difftime(cmd_inputs.zend, cmd_inputs.zstart);

		auto zend = std::chrono::steady_clock::now();
		iterations.back().time_surveillance = std::chrono::duration_cast<std::chrono::duration<double>>(zend - zstart).count();
		iterations.back().truck_time_used.surv = survfirst_delivery.get_route_travel_time(true);
		iterations.back().drone_time_used_without_sorties.surv = survfirst_delivery.get_route_travel_time(false);
		iterations.back().drone_sortie_time.surv = survfirst_delivery.get_route_travel_time(false,true);
		iterations.back().truck_drone_tandem_time_used.surv = survfirst_delivery.get_route_tandem_travel_time();
		//get truck and drone waiting times
		iterations.back().truck_wait.surv = survfirst_delivery.get_route_waiting_time(true);
		iterations.back().drone_wait.surv = survfirst_delivery.get_route_waiting_time(false);

		iterations.back().operations_num.surv = (int) survfirst_delivery.truck_sorties.size();
		iterations.back().drone_on_truck_operations_num.surv = survfirst_delivery.get_drone_riding_truck_ops();
		//get vectors
		iterations.back().truck_op_times.surv = survfirst_delivery.get_op_travel_time_vec(true);
		iterations.back().drone_op_times.surv = survfirst_delivery.get_op_travel_time_vec(false);

		iterations.back().truck_op_wait_times.surv = survfirst_delivery.get_op_waiting_time_vec(true);
		iterations.back().drone_op_wait_times.surv = survfirst_delivery.get_op_waiting_time_vec(false);

		iterations.back().drone_op_times_with_wait_time.surv = survfirst_delivery.get_op_travel_and_waiting_time_vec(false);

		string sol_id = "_";
		switch(simulation_setting) {
			case 1:
				sol_id = sol_id+"ct";
				break;
			case 2:
				sol_id = sol_id+"ld";
				break;
			default:
				break;
		}	

		

		ausgabe_tour(survfirst_delivery, output_name + "delivery"+sol_id, solution_OK);

		check_solution_surveillance(survfirst_surveillance, solution_OK);
		ausgabe_tour_surveillance(survfirst_surveillance, output_name + "surveillance"+sol_id, solution_OK);
	};
}

void Simulation::run_surv_op(int simulation_setting)
{
	if (simulation_setting > 0 && simulation_setting < 3) {
		auto zstart = std::chrono::steady_clock::now();
		string output_name;
		Solution survop_delivery(0,instance.n_nodes);
		Solution survop_surveillance(0, instance.n_nodes);
		vector<vector<int>>arcs_status_survop;
		double cost_of_surveillance_survop;	// also initialized in adp, but unused right now
		double cost_of_delivery_survop;		// also initialized in adp, but unused right now
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			arcs_status_survop.push_back(instance.arc_status[loc_i]);
		};

		//time(&cmd_inputs.zstart);
		bool solution_OK;

		//change if you want to run naive tsp
		bool arc_based_tsp = true;
		
		solution_OK = surveillance_first_orienteering_variant(arcs_status_survop, cost_of_surveillance_survop, cost_of_delivery_survop, survop_delivery, survop_surveillance, simulation_setting, cmd_inputs.cmax, cmd_inputs.penalty_factor, arc_based_tsp);
		output_name = "survop_";
		
		iterations.back().survop_surveillance_tour_cost = cost_of_surveillance_survop;
		iterations.back().survop_obj = cost_of_delivery_survop;
		//time(&cmd_inputs.zend);
		//iterations.back().time_surveillance = difftime(cmd_inputs.zend, cmd_inputs.zstart);

		auto zend = std::chrono::steady_clock::now();
		iterations.back().time_surveillance_op = std::chrono::duration_cast<std::chrono::duration<double>>(zend - zstart).count();
		iterations.back().truck_time_used.surv_op = survop_delivery.get_route_travel_time(true);
		iterations.back().drone_time_used_without_sorties.surv_op = survop_delivery.get_route_travel_time(false);
		iterations.back().drone_sortie_time.surv_op = survop_delivery.get_route_travel_time(false,true);
		iterations.back().truck_drone_tandem_time_used.surv_op = survop_delivery.get_route_tandem_travel_time();
		iterations.back().truck_wait.surv_op = survop_delivery.get_route_waiting_time(true);
		iterations.back().drone_wait.surv_op = survop_delivery.get_route_waiting_time(false);
		iterations.back().operations_num.surv_op_delivery = (int) survop_delivery.truck_sorties.size();
		iterations.back().drone_on_truck_operations_num.surv_op_delivery = survop_delivery.get_drone_riding_truck_ops();

		//get vectors
		iterations.back().truck_op_times.surv_op = survop_delivery.get_op_travel_time_vec(true);
		iterations.back().drone_op_times.surv_op = survop_delivery.get_op_travel_time_vec(false);

		iterations.back().truck_op_wait_times.surv_op = survop_delivery.get_op_waiting_time_vec(true);
		iterations.back().drone_op_wait_times.surv_op = survop_delivery.get_op_waiting_time_vec(false);

		iterations.back().drone_op_times_with_wait_time.surv_op = survop_delivery.get_op_travel_and_waiting_time_vec(false);


		string sol_id = "_";
		switch(simulation_setting) {
			case 1:
				sol_id = sol_id+"ct";
				break;
			case 2:
				sol_id = sol_id+"ld";
				break;
			default:
				break;
		}	

		

		ausgabe_tour(survop_delivery, output_name + "delivery"+sol_id, solution_OK);

		check_solution_surveillance_op(survop_surveillance, solution_OK);
		ausgabe_tour_surveillance(survop_surveillance, output_name + "surveillance"+sol_id, solution_OK);
	};
}

void Simulation::run_reopt(int simulation_setting)
{
	if (simulation_setting > 0 && simulation_setting < 3) {
		auto zstart = std::chrono::steady_clock::now();
		vector<vector<double>>arc_cost_for_truck = vector<vector<double>>(instance.n_nodes, vector<double>(instance.n_nodes));
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
				arc_cost_for_truck[loc_i][loc_j] = instance.euclidean_distance_matrix[loc_i][loc_j];
			};
		};
		Solution delivery_reopt_sol(0, instance.n_nodes);
		//time(&cmd_inputs.zstart);
		bool solution_OK = false;
		//simulate_delivery_truck_and_drone_conservative(2, instance.arc_status, arc_cost_for_truck, delivery_reopt_sol, cmd_inputs.setting_reopt);
		//solution_OK = reoptA(2, instance.arc_status, delivery_reopt_sol, simulation_setting);
		solution_OK = reoptA_dp(instance.arc_status, delivery_reopt_sol, simulation_setting);


		//time(&cmd_inputs.zend);
		//iterations.back().time_reopt = difftime(cmd_inputs.zend, cmd_inputs.zstart);
		iterations.back().reopt_obj = delivery_reopt_sol.obj_value;
		iterations.back().mistake_reported = delivery_reopt_sol.mistake_reported;
		iterations.back().mistake_text = iterations.back().mistake_text + delivery_reopt_sol.mistake_text;
		auto zend = std::chrono::steady_clock::now();
		iterations.back().time_reopt = std::chrono::duration_cast<std::chrono::duration<double>>(zend - zstart).count();
		iterations.back().truck_time_used.reopt = delivery_reopt_sol.get_route_travel_time(true);
		iterations.back().drone_time_used_without_sorties.reopt = delivery_reopt_sol.get_route_travel_time(false);
		iterations.back().drone_sortie_time.reopt = delivery_reopt_sol.get_route_travel_time(false,true);
		iterations.back().truck_wait.reopt = delivery_reopt_sol.get_route_waiting_time(true);
		iterations.back().drone_wait.reopt = delivery_reopt_sol.get_route_waiting_time(false);
		iterations.back().truck_drone_tandem_time_used.reopt = delivery_reopt_sol.get_route_tandem_travel_time();
		iterations.back().operations_num.reopt = (int) delivery_reopt_sol.truck_sorties.size();
		iterations.back().drone_on_truck_operations_num.reopt = delivery_reopt_sol.get_drone_riding_truck_ops();

		//get vectors
		iterations.back().truck_op_times.reopt = delivery_reopt_sol.get_op_travel_time_vec(true);
		iterations.back().drone_op_times.reopt = delivery_reopt_sol.get_op_travel_time_vec(false);

		iterations.back().truck_op_wait_times.reopt = delivery_reopt_sol.get_op_waiting_time_vec(true);
		iterations.back().drone_op_wait_times.reopt = delivery_reopt_sol.get_op_waiting_time_vec(false);

		iterations.back().drone_op_times_with_wait_time.reopt = delivery_reopt_sol.get_op_travel_and_waiting_time_vec(false);
		

		string sol_id = "_";
		switch(simulation_setting) {
			case 1:
				sol_id = sol_id+"ct";
				break;
			case 2:
				sol_id = sol_id+"ld";
				break;
			default:
				break;
		}	
		sol_id =  "reopt"+sol_id;
		ausgabe_tour(delivery_reopt_sol, sol_id , solution_OK);
	};
}

void Simulation::run_conservative(int simulation_setting)
{
	if (simulation_setting > 0 && simulation_setting < 3) {
		//time(&cmd_inputs.zstart);
		Solution conservative_ub_solution(0, instance.n_nodes);
		vector<bool>loc_temp_customers_to_visit = instance.customers_to_visit;
		loc_temp_customers_to_visit[0] = false;
		auto zstart = std::chrono::steady_clock::now();
		Model_DP Model_DP(instance, instance.arc_status_conservative, instance.arc_status_conservative, 0, loc_temp_customers_to_visit, cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time);
		//Model_DP.solve();
		Model_DP.solve(simulation_setting);
		conservative_ub_solution = Model_DP.model_solution;
		iterations.back().conservative_obj = conservative_ub_solution.obj_value;
		iterations.back().naive_saved = true;
		//time(&cmd_inputs.zend);
		//iterations[0].time_conservative = difftime(cmd_inputs.zend, cmd_inputs.zstart);
		auto zend = std::chrono::steady_clock::now();
		iterations.back().time_conservative = std::chrono::duration_cast<std::chrono::duration<double>>(zend - zstart).count();
		bool solution_OK = Model_DP.check_solution(Model_DP.model_solution, iterations.back().current_damages);
		iterations.back().mistake_reported = Model_DP.model_solution.mistake_reported;
		iterations.back().mistake_text = iterations.back().mistake_text + Model_DP.model_solution.mistake_text;
		string sol_id = "_";
		switch(simulation_setting) {
			case 1:
				sol_id = sol_id+"ct";
				break;
			case 2:
				sol_id = sol_id+"ld";
				break;
			default:
				break;
		}	
		sol_id ="conservative"+sol_id ;
		iterations.back().drone_op_times.conservative = Model_DP.model_solution.get_op_travel_time_vec(false);
		ausgabe_tour(conservative_ub_solution, sol_id, solution_OK);
	}
}

void Simulation::weighted_sample() {
	vector<vector<int>> freq = instance.arc_status;
	int edge_num = 0;
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			switch (freq[loc_i][loc_j]) {
			case 0: {
				freq[loc_i][loc_j] = 1;
				edge_num = edge_num + 1;
				break;
			}
			case 1: {
				freq[loc_i][loc_j] = 1;
				edge_num = edge_num + 1;
				break;
			}
			}
		}
	}
	std::vector<int> cluster_arr(instance.n_nodes);
	std::iota(cluster_arr.begin(), cluster_arr.end(), 0);
	std::shuffle(std::begin(cluster_arr), std::end(cluster_arr), random_engine_generator);
	std::vector<int> cluster_neighbors_arr;
	for(int k=0;k<cmd_inputs.input_clustered_damages;k++){
		cluster_arr.push_back(cluster_arr[k]);
	}
	for (int ith : cluster_arr) {
		for (int num = 1; num <= instance.adjacent_nodes[ith][0]; num++) {
			int node_to_add = instance.adjacent_nodes[ith][num];
			if (node_to_add != 0 && std::find(cluster_neighbors_arr.begin(), cluster_neighbors_arr.end(), node_to_add) == cluster_neighbors_arr.end()) {
				cluster_neighbors_arr.push_back(node_to_add);
			}
		}
	}

	vector<pair<int, int>> random_choice_vec;
	for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = loc_i+1; loc_j < instance.n_nodes; loc_j++) {
			switch (freq[loc_i][loc_j]) {
			case 1: {
				if (std::find(cluster_arr.begin(), cluster_arr.end(), loc_i) != cluster_arr.end() || std::find(cluster_arr.begin(), cluster_arr.end(), loc_j) != cluster_arr.end()) {
					int weight = 3;
					freq[loc_i][loc_j] = weight;
					random_choice_vec.insert(random_choice_vec.end(), weight, { loc_i,loc_j });
				}
				else if ((std::find(cluster_neighbors_arr.begin(), cluster_neighbors_arr.end(), loc_i) != cluster_neighbors_arr.end()) && (std::find(cluster_arr.begin(), cluster_arr.end(), loc_j) == cluster_arr.end())) {
					int weight = 2;
					freq[loc_i][loc_j] = weight;
					random_choice_vec.insert(random_choice_vec.end(), weight, { loc_i,loc_j });
				}
				else {
					random_choice_vec.push_back({ loc_i,loc_j });
				}
				break;
			}
			}
		}
	}
	//arc damages by k arc damages
	for (int loc_k = 0; loc_k < cmd_inputs.input_k_arc_damages; loc_k++) {
		if (random_choice_vec.size() > 0) {
			std::shuffle(std::begin(random_choice_vec), std::end(random_choice_vec), random_engine_generator);
			pair<int, int> edge = random_choice_vec.back();
			int loc_i = edge.first;
			int loc_j = edge.second;
			iterations.back().n_damaged_arcs = iterations.back().n_damaged_arcs + 1;
			iterations.back().current_damages[loc_i][loc_j] = -1;
			iterations.back().current_damages[loc_j][loc_i] = -1;
			pair<int, int> edge_invert = { loc_j, loc_i };
			random_choice_vec.erase(std::remove(random_choice_vec.begin(), random_choice_vec.end(), edge), random_choice_vec.end());
			random_choice_vec.erase(std::remove(random_choice_vec.begin(), random_choice_vec.end(), edge_invert), random_choice_vec.end());
		}
	}
}

void Simulation::cluster_damage()
{
	std::vector<int> cluster_arr(instance.n_nodes-1);
	std::iota(cluster_arr.begin(), cluster_arr.end(), 1);
	std::shuffle(std::begin(cluster_arr), std::end(cluster_arr), random_engine_generator);
	for(int k=0;k<cmd_inputs.input_clustered_damages;k++) {
		int loc_i = cluster_arr[k];
		for(int loc_j=1; loc_j < instance.n_nodes;loc_j++) {
			if(instance.arc_status[loc_i][loc_j]>-1 && loc_i!=loc_j && iterations.back().current_damages[loc_i][loc_j] > -1 && iterations.back().n_damaged_arcs<cmd_inputs.input_k_arc_damages) {
				iterations.back().current_damages[loc_i][loc_j] = -1;
				iterations.back().current_damages[loc_j][loc_i] = -1;
				//cout << loc_i << ", " << loc_j << "\n";
				iterations.back().n_damaged_arcs = iterations.back().n_damaged_arcs + 1;
			}
		}
	}
	vector<pair<int, int>> random_choice_vec;
	for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = loc_i+1; loc_j < instance.n_nodes; loc_j++) {
			if (instance.arc_status[loc_i][loc_j] > -1 && iterations.back().current_damages[loc_i][loc_j] > -1){
				random_choice_vec.push_back({ loc_i,loc_j });
			}
		}
	}
	std::shuffle(std::begin(random_choice_vec), std::end(random_choice_vec), random_engine_generator);
	for (int k = 0; k< cmd_inputs.input_k_arc_damages; k++){
		if (k<(int)random_choice_vec.size() && iterations.back().n_damaged_arcs < cmd_inputs.input_k_arc_damages){
			pair<int, int> edge = random_choice_vec[k];
			iterations.back().n_damaged_arcs = iterations.back().n_damaged_arcs + 1;
			iterations.back().current_damages[edge.first][edge.second] = -1;
			iterations.back().current_damages[edge.second][edge.first] = -1;
			//cout << edge.first<< ", " << edge.second << "\n";
		}
		else{
			break;
		}
	}
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
				if ((instance.arc_status[loc_i][loc_j] == 0 && iterations.back().current_damages[loc_i][loc_j] > -1) || loc_i == loc_j) {
					iterations.back().current_damages[loc_i][loc_j] = 1;
					iterations.back().current_damages[loc_j][loc_i] = 1;
			};
		};
	};
}

bool Simulation::reoptA_dp(vector<vector<int>>arcs_status_after_surveillance, Solution& delivery_sol, int simulation_setting) {
	vector<bool>loc_temp_customers_to_visit = instance.customers_to_visit;
	loc_temp_customers_to_visit[0] = false;
	vector<bool>whether_the_node_visited = vector<bool>(instance.n_nodes);
	whether_the_node_visited[0] = true;
	vector<bool>whether_the_customer_node_visited = vector<bool>(instance.n_nodes);
	whether_the_node_visited[0] = true;
	reopt_parameters = reoptParams();
	reopt_parameters.next_atificial_node_id = instance.n_nodes;

	for (int loc_f = 0; loc_f < instance.n_nodes; loc_f++) {//turn the arc status into optimistic
		for (int loc_r = 0; loc_r < instance.adjacent_nodes[loc_f][0]; loc_r++) {
			int neighbornode_loc = instance.adjacent_nodes[loc_f][loc_r + 1];
			if (arcs_status_after_surveillance[loc_f][neighbornode_loc] == 0) {
				arcs_status_after_surveillance[loc_f][neighbornode_loc] = 1;
			};
		};
	};

	double time_up_to_now = 0;

	//Initialize
	Model_DP euler_model(instance, arcs_status_after_surveillance, arcs_status_after_surveillance ,  reopt_parameters.last_combined_node, loc_temp_customers_to_visit,cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time);
	bool solution_OK = true;
	iterations.back().n_replan_reopt_last_operation = 0;
	int casecode = 3; 
	while (reopt_parameters.replan_reopt_truck == true || reopt_parameters.replan_reopt_drone == true) {
		reopt_parameters.replan_reopt_truck = false;
		reopt_parameters.replan_reopt_drone = false;
		reopt_parameters.n_replan_reopt = reopt_parameters.n_replan_reopt + 1;
		if (reopt_parameters.n_replan_reopt == 0) {
			//euler_model.solve();
			euler_model.solve(simulation_setting);
			solution_OK = euler_model.check_solution(euler_model.model_solution, arcs_status_after_surveillance);
			
			update_planned_truck_arcs(casecode,euler_model.Ledges_struct.Ledges_from_end_to_start);
			iterations.back().damaged_edges_in_solution.reopt = iterations.back().damaged_edges_in_solution.reopt+ euler_model.model_solution.get_damaged_trucks_arcs(iterations.back().current_damages);
			iterations.back().optimistic_sol_operations = (int) euler_model.model_solution.truck_sorties.size();
			iterations.back().optimistic_initial_solution = euler_model.model_solution.obj_value;

		}
		else {
			euler_model.solve_reopt(arcs_status_after_surveillance,arcs_status_after_surveillance, loc_temp_customers_to_visit, reopt_parameters, simulation_setting);
			reopt_parameters.next_atificial_node_id = euler_model.dp.n_nodes + euler_model.dp.artificial_nodes;
			solution_OK = euler_model.check_solution(euler_model.model_solution, arcs_status_after_surveillance);
 			update_planned_truck_arcs(casecode,euler_model.Ledges_struct.Ledges_from_end_to_start);
			iterations.back().damaged_edges_in_solution.reopt = iterations.back().damaged_edges_in_solution.reopt+ euler_model.model_solution.get_damaged_trucks_arcs(iterations.back().current_damages);

		}
		for (int loc_i = 0; loc_i < euler_model.model_solution.n_stages; loc_i++) {
			std::pair<bool, bool> check_replan = check_replan_reopt(loc_i, euler_model.model_solution, loc_temp_customers_to_visit);
			if(cmd_inputs.setting_sichtfeld>0) {
				check_replan = check_replan_reopt_sichtfeld(loc_i, euler_model.model_solution, loc_temp_customers_to_visit);
			}
			double stage_duration = 0;
			reopt_parameters.drone_duration = 0;
			reopt_parameters.truck_duration = 0;
			if (!reopt_parameters.fix_operation) {
				reopt_parameters.last_combined_node = euler_model.model_solution.truck_sorties[loc_i].front();
				//needed for the service time accounting
				euler_model.reopt_solution.list_of_op_customers.push_back(vector<int>(0));
			}
			if (check_replan.first) {
				reopt_parameters.truck_duration = simulate_reopt_truck_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, arcs_status_after_surveillance, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited, reopt_parameters);
				reopt_parameters.drone_duration = simulate_reopt_drone_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, arcs_status_after_surveillance, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited, reopt_parameters);
			}
			else {
				reopt_parameters.drone_duration = simulate_reopt_drone_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, arcs_status_after_surveillance, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited,reopt_parameters);
				reopt_parameters.truck_duration = simulate_reopt_truck_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, arcs_status_after_surveillance, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited,reopt_parameters);
			}
			if((!reopt_parameters.replan_reopt_drone && !reopt_parameters.replan_reopt_truck && loc_i == euler_model.model_solution.n_stages-1 && euler_model.model_solution.last_delivered_customer_obj2>-1 && euler_model.model_solution.last_delivered_customers_last_delivery_op.second ==-1 )) {
				reopt_parameters.drone_duration = 0.0;
			}	
			stage_duration = reopt_parameters.drone_duration;
			if (reopt_parameters.truck_duration > stage_duration) {
				stage_duration = reopt_parameters.truck_duration;
			};
			time_up_to_now = time_up_to_now + stage_duration;
			cout << "Reopt simulation time up to now: " << time_up_to_now << '\n';
			reopt_parameters.fix_operation = false;
			if (reopt_parameters.replan_reopt_drone || reopt_parameters.replan_reopt_truck) {
				if (reopt_parameters.curr_drone_node != reopt_parameters.curr_truck_node) {
					reopt_parameters.fix_operation = true;
				}
				//count replanning in last operation
				if(simulation_setting ==2 && loc_i == euler_model.model_solution.n_stages-1){
					iterations.back().n_replan_reopt_last_operation = iterations.back().n_replan_reopt_last_operation+1;
				}
				break;
			};

		};//for all stages
		//bool visited_all = std::none_of(loc_temp_customers_to_visit.begin(), loc_temp_customers_to_visit.end(), [](bool v) { return v; });
	};

	euler_model.reopt_solution.obj_value = time_up_to_now;
	euler_model.customers_to_visit = instance.customers_to_visit;
	euler_model.reopt_solution.n_stages = euler_model.reopt_solution.truck_sorties.size();
	euler_model.last_combined_node = 0;
	euler_model.curr_drone_node = 0;
	euler_model.curr_truck_node = 0;
	euler_model.reopt_solution.mistake_text = euler_model.mistake_text;
	//euler_model.customer_set = initial_customer_set;
	euler_model.reset_model(arcs_status_after_surveillance, instance.customers_to_visit, 0, 0, 0);
	euler_model.init_dp_struct(1.0,false);
	cout << "Final Check" << '\n';
	solution_OK = euler_model.check_solution(euler_model.reopt_solution, iterations.back().current_damages);
	delivery_sol = euler_model.reopt_solution;
	return solution_OK;

}

double Simulation::simulate_reopt_truck_walk(int loc_i, Solution& delivery_sol, Solution& delivery_solution_model, vector<vector<int>>& arcs_status_after_surveillance, vector<bool>& loc_temp_customers_to_visit, vector<bool>& whether_the_node_visited, vector<bool>& whether_the_customer_node_visited, reoptParams& loc_reopt_params) {
	double truck_duration = 0.0;
	int first_node = delivery_solution_model.truck_sorties[loc_i][0];
	int temp_loc_curr_position = first_node;
	bool breakup_delivery_service = false;
	if (!loc_reopt_params.fix_operation) {
		delivery_sol.truck_sorties.push_back(vector<int>(0));
		//if (first_node < instance.n_nodes) {
	}
	delivery_sol.truck_sorties.back().push_back(first_node);
	if((loc_temp_customers_to_visit[first_node]==true) && (first_node!= delivery_solution_model.drone_customers[loc_i].back()) && (std::find(delivery_solution_model.list_of_op_customers[loc_i].begin(), delivery_solution_model.list_of_op_customers[loc_i].end(), first_node) != delivery_solution_model.list_of_op_customers[loc_i].end())) {
		truck_duration = truck_duration +cmd_inputs.service_time;
		whether_the_customer_node_visited[first_node] = true;
		delivery_sol.list_of_op_customers.back().push_back(first_node);
		loc_temp_customers_to_visit[first_node] = false;
		if (loc_reopt_params.replan_reopt_drone && (truck_duration > loc_reopt_params.drone_duration)) {
			breakup_delivery_service = true;
			ArtificialNode new_artificial_node(loc_reopt_params.next_atificial_node_id,instance.n_nodes, first_node, first_node,cmd_inputs.service_time, truck_duration,  loc_reopt_params.drone_duration, cmd_inputs.service_time, breakup_delivery_service, delivery_solution_model.art_node_map,delivery_solution_model.truck_distances);
			temp_loc_curr_position = loc_reopt_params.next_atificial_node_id;
			truck_duration = loc_reopt_params.drone_duration;
			loc_reopt_params.curr_truck_node = temp_loc_curr_position;
			loc_reopt_params.artificial_node = new_artificial_node;
			return truck_duration;
		}
	}
	if (loc_reopt_params.replan_reopt_drone && areEqual(loc_reopt_params.drone_duration,0.0)) {
		return truck_duration;
	}
	
	for (int loc_j = 1; loc_j < (int)delivery_solution_model.truck_sorties[loc_i].size(); loc_j++) {
		int second_node = delivery_solution_model.truck_sorties[loc_i][loc_j];

		//check next arc
		if(first_node<instance.n_nodes){
			arcs_status_after_surveillance[first_node][second_node] = iterations.back().current_damages[first_node][second_node];
			arcs_status_after_surveillance[second_node][first_node] = iterations.back().current_damages[second_node][first_node];
			if ( (arcs_status_after_surveillance[first_node][second_node] <= -1)) {
				loc_reopt_params.replan_reopt_truck = true;
				break;
			};
		}

		if (loc_reopt_params.replan_reopt_truck) {
			truck_duration = truck_duration + 0;
		}
		else {
			double distance_move = delivery_solution_model.truck_distances[first_node][second_node];
			double traveled_dist = truck_duration + distance_move;
			//check if the other edge damage is discovered during the service of the customer
			//the travel distance should be below the truck duration
			if(loc_reopt_params.replan_reopt_drone && (traveled_dist < loc_reopt_params.drone_duration)) {
				//check if it is a service node and if the truck detects the damage earlier
				if((loc_temp_customers_to_visit[second_node]==true) && (second_node!= delivery_solution_model.drone_customers[loc_i].back()) && (std::find(delivery_solution_model.list_of_op_customers[loc_i].begin(), delivery_solution_model.list_of_op_customers[loc_i].end(), second_node) != delivery_solution_model.list_of_op_customers[loc_i].end())) {
					if(traveled_dist+delivery_solution_model.service_time_map[second_node] > loc_reopt_params.drone_duration) {
						breakup_delivery_service = true;
						traveled_dist = traveled_dist+delivery_solution_model.service_time_map[second_node] ;
					}
				}
			}
			if (loc_reopt_params.replan_reopt_drone && (traveled_dist > loc_reopt_params.drone_duration)) {
				ArtificialNode new_artificial_node(loc_reopt_params.next_atificial_node_id,instance.n_nodes, first_node, second_node,distance_move, traveled_dist,  loc_reopt_params.drone_duration, cmd_inputs.service_time, breakup_delivery_service, delivery_solution_model.art_node_map,delivery_solution_model.truck_distances);
				if (areEqual(new_artificial_node.real_incoming_edge.second,0.0)) {
					temp_loc_curr_position = first_node;
				}
				//no artificial node "needed"
				else if(areEqual(new_artificial_node.real_outgoing_edge.second,0.0)) {
					temp_loc_curr_position = second_node;
					delivery_sol.truck_sorties.back().push_back(second_node);
				}
				//check if already reached second node and begun service time
				else if(breakup_delivery_service) {
					delivery_sol.truck_sorties.back().push_back(second_node);
					new_artificial_node = ArtificialNode(loc_reopt_params.next_atificial_node_id,instance.n_nodes, second_node, second_node,distance_move, traveled_dist,  loc_reopt_params.drone_duration, cmd_inputs.service_time, breakup_delivery_service, delivery_solution_model.art_node_map,delivery_solution_model.truck_distances);
					temp_loc_curr_position = loc_reopt_params.next_atificial_node_id;
					if (second_node < instance.n_nodes) {
						if ((whether_the_customer_node_visited[second_node] == false) && (loc_temp_customers_to_visit[second_node] == true)) {
							if((second_node!= delivery_solution_model.drone_customers[loc_i].back()) && (std::find(delivery_solution_model.list_of_op_customers[loc_i].begin(), delivery_solution_model.list_of_op_customers[loc_i].end(), second_node) != delivery_solution_model.list_of_op_customers[loc_i].end())) {
									//not needed!?
									truck_duration = truck_duration+cmd_inputs.service_time;
									whether_the_customer_node_visited[second_node] = true;
									delivery_sol.list_of_op_customers.back().push_back(second_node);
									loc_temp_customers_to_visit[second_node] = false;
							}
						}
						loc_reopt_params.replan_reopt_truck = check_edges(second_node,delivery_solution_model,arcs_status_after_surveillance, whether_the_node_visited);
					}
				}
				else {
					temp_loc_curr_position = loc_reopt_params.next_atificial_node_id;
				}
				truck_duration = loc_reopt_params.drone_duration;
				loc_reopt_params.curr_truck_node = temp_loc_curr_position;
				loc_reopt_params.artificial_node = new_artificial_node;
				return truck_duration;
			}
			else {
				delivery_sol.truck_sorties.back().push_back(second_node);
				truck_duration = traveled_dist;
			}
		};
		first_node = second_node;
		temp_loc_curr_position = first_node;
		loc_reopt_params.curr_truck_node = temp_loc_curr_position;
		//CASE that the duration is exactly equal, we do need an artificial node to close the operation
		if (loc_reopt_params.replan_reopt_drone && (areEqual(truck_duration, loc_reopt_params.drone_duration)==true)) {
			//artificial waiting node if < drone duration
			//else no artificial node position
			if (second_node < instance.n_nodes) { 
				if (whether_the_node_visited[second_node] == false) {
					loc_reopt_params.replan_reopt_truck = check_edges(second_node,delivery_solution_model,arcs_status_after_surveillance, whether_the_node_visited);
					loc_reopt_params.n_visited_nodes = loc_reopt_params.n_visited_nodes + 1;
					if (loc_reopt_params.replan_reopt_truck || (loc_reopt_params.replan_reopt_drone && areEqual(truck_duration,loc_reopt_params.drone_duration))) {
						break;
					}
				}
			}
			break;
		}
		if (second_node < instance.n_nodes) {
			if ((whether_the_customer_node_visited[second_node] == false) && (loc_temp_customers_to_visit[second_node] == true)) {
				if((second_node!= delivery_solution_model.drone_customers[loc_i].back()) && (std::find(delivery_solution_model.list_of_op_customers[loc_i].begin(), delivery_solution_model.list_of_op_customers[loc_i].end(), second_node) != delivery_solution_model.list_of_op_customers[loc_i].end())) {
						truck_duration = truck_duration+cmd_inputs.service_time;
						whether_the_customer_node_visited[second_node] = true;
						delivery_sol.list_of_op_customers.back().push_back(second_node);
						loc_temp_customers_to_visit[second_node] = false;
				}
			}
			if (whether_the_node_visited[second_node] == false) {
				loc_reopt_params.replan_reopt_truck = check_edges(second_node,delivery_solution_model,arcs_status_after_surveillance, whether_the_node_visited);
				loc_reopt_params.n_visited_nodes = loc_reopt_params.n_visited_nodes + 1;
				if (loc_reopt_params.replan_reopt_truck || (loc_reopt_params.replan_reopt_drone && areEqual(truck_duration,loc_reopt_params.drone_duration))) {
					temp_loc_curr_position = second_node;
					break;
				}
			};
		}
	};
	loc_reopt_params.curr_truck_node = temp_loc_curr_position;
	if (loc_reopt_params.replan_reopt_drone && (truck_duration < loc_reopt_params.drone_duration)) {
		ArtificialNode new_artificial_node(loc_reopt_params.next_atificial_node_id, instance.n_nodes, loc_reopt_params.curr_truck_node,loc_reopt_params.curr_truck_node,0, loc_reopt_params.drone_duration,loc_reopt_params.drone_duration, cmd_inputs.service_time, false, delivery_solution_model.art_node_map, delivery_solution_model.truck_distances);
		new_artificial_node.real_incoming_edge.second = (loc_reopt_params.drone_duration - truck_duration);
		new_artificial_node.real_outgoing_edge.second = 0;
		new_artificial_node.artificial_outgoing_edge.second = std::numeric_limits<double>::infinity();
		loc_reopt_params.curr_truck_node = loc_reopt_params.next_atificial_node_id;
		loc_reopt_params.artificial_node = new_artificial_node;
		truck_duration = loc_reopt_params.drone_duration;
	}
	return truck_duration;
};

double Simulation::simulate_reopt_drone_walk(int loc_i, Solution& delivery_sol, Solution& delivery_solution_model, vector<vector<int>>& arcs_status_after_surveillance, vector<bool>& loc_temp_customers_to_visit, vector<bool>& whether_the_node_visited, vector<bool>& whether_the_customer_node_visited,  reoptParams& loc_reopt_params) {
	double drone_duration = 0.0;
	//int loc_number_drone_nodes = delivery_solution_model.drone_sorties[loc_i].size();
	bool drone_customer_delivered = false;
	int first_node = delivery_solution_model.drone_sorties[loc_i].front();
	bool breakup_delivery_service = false;
	if (loc_reopt_params.replan_reopt_truck && areEqual(loc_reopt_params.truck_duration,0.0)) {
		if (!loc_reopt_params.fix_operation) {
			delivery_sol.drone_sorties.push_back(vector<int>(1, -1));
			delivery_sol.drone_customers.push_back(vector<int>(1, -1));
		}
		return drone_duration;
	}
	if (!loc_reopt_params.fix_operation) {
		delivery_sol.drone_sorties.push_back(vector<int>(0));
		delivery_sol.drone_customers.push_back(vector<int>(1,-1));
	}
	//check if drone stage exist, or reopt operation, which needs fixing. If last delivery objective, then only fix the operatio if it is not the last operation
	if (delivery_solution_model.drone_sorties[loc_i].front() > -1 || loc_reopt_params.fix_operation) {
		//if (first_node < instance.n_nodes) {
		delivery_sol.drone_sorties.back().push_back(first_node);
		//}
		for (int loc_j = 0; loc_j < (int)delivery_solution_model.drone_sorties[loc_i].size(); loc_j++) {
			int temp_second_node = delivery_solution_model.drone_sorties[loc_i][loc_j];
			double distance_move = delivery_solution_model.drone_distances[first_node][temp_second_node];
			double traveled_dist = drone_duration;
			traveled_dist = traveled_dist + distance_move;
			//check if the other edge damage is discovered during the service of the customer
			//the travel distance should be below the truck duration
			if(loc_reopt_params.replan_reopt_truck && (traveled_dist < loc_reopt_params.truck_duration)) {
				//check if it is a service node and if the truck detects the damage earlier
				if((!drone_customer_delivered && temp_second_node == delivery_solution_model.drone_customers[loc_i].back()) && (traveled_dist+delivery_solution_model.service_time_map[temp_second_node] > loc_reopt_params.truck_duration)) {
					breakup_delivery_service = true;
					traveled_dist = traveled_dist+delivery_solution_model.service_time_map[temp_second_node];
				}
			}
			if (loc_reopt_params.replan_reopt_truck && ((traveled_dist > loc_reopt_params.truck_duration) || breakup_delivery_service )) {
				ArtificialNode new_artificial_node(loc_reopt_params.next_atificial_node_id, instance.n_nodes,first_node,temp_second_node,distance_move, traveled_dist,loc_reopt_params.truck_duration, cmd_inputs.service_time, breakup_delivery_service, delivery_solution_model.art_node_map,delivery_solution_model.drone_distances);
				if (areEqual(new_artificial_node.real_incoming_edge.second,0.0)) {
					loc_reopt_params.curr_drone_node = first_node;
				}
				else if (areEqual(new_artificial_node.real_outgoing_edge.second, 0.0) || breakup_delivery_service) {
					//check if already reached second node and begun service time
					if(breakup_delivery_service) {
						loc_reopt_params.curr_drone_node = loc_reopt_params.next_atificial_node_id;
						new_artificial_node = ArtificialNode(loc_reopt_params.next_atificial_node_id, instance.n_nodes,temp_second_node,temp_second_node,distance_move, traveled_dist,loc_reopt_params.truck_duration, cmd_inputs.service_time, breakup_delivery_service, delivery_solution_model.art_node_map,delivery_solution_model.drone_distances);

					}
					else{
						loc_reopt_params.curr_drone_node = temp_second_node;
					}
					delivery_sol.drone_sorties.back().push_back(temp_second_node);
					if (!drone_customer_delivered && temp_second_node == delivery_solution_model.drone_customers[loc_i].back()) {
						delivery_sol.drone_customers.back() = delivery_solution_model.drone_customers[loc_i];
						loc_reopt_params.curr_drone_cust = -1;
						drone_customer_delivered = true;
						//needed for accounting of service times
						delivery_sol.list_of_op_customers.back().push_back(temp_second_node);
						//dont add the service time twice
						if(breakup_delivery_service==false) {
							traveled_dist = traveled_dist+cmd_inputs.service_time;
						}
						whether_the_customer_node_visited[temp_second_node] = true;
						loc_temp_customers_to_visit[temp_second_node] = false;
						loc_reopt_params.n_visited_nodes = loc_reopt_params.n_visited_nodes + 1;
					}
					if (temp_second_node < instance.n_nodes) {
						if(cmd_inputs.setting_sichtfeld==0){
							loc_reopt_params.replan_reopt_drone = check_edges(temp_second_node, delivery_solution_model, arcs_status_after_surveillance, whether_the_node_visited);
						}
						else{
							loc_reopt_params.replan_reopt_drone = check_edges_sichtfeld_drone(temp_second_node, delivery_solution_model, arcs_status_after_surveillance, whether_the_node_visited);
						};
						if (loc_reopt_params.replan_reopt_drone || (loc_reopt_params.replan_reopt_truck && areEqual(drone_duration,loc_reopt_params.truck_duration))) {
							if(breakup_delivery_service==false){
								loc_reopt_params.curr_drone_node = temp_second_node;
							}
						}
					}
				}
				else {
					loc_reopt_params.curr_drone_node = loc_reopt_params.next_atificial_node_id;
				}
				drone_duration = loc_reopt_params.truck_duration;
				//break
				if (!drone_customer_delivered) {
					loc_reopt_params.curr_drone_cust = delivery_solution_model.drone_customers[loc_i].back();
				}
				loc_reopt_params.artificial_node = new_artificial_node;
				return drone_duration;
			}
			delivery_sol.drone_sorties.back().push_back(temp_second_node);
			drone_duration = traveled_dist;
			if (temp_second_node < instance.n_nodes) {
				if(cmd_inputs.setting_sichtfeld==0){
					loc_reopt_params.replan_reopt_drone = check_edges(temp_second_node, delivery_solution_model, arcs_status_after_surveillance, whether_the_node_visited);
				}
				else{
					loc_reopt_params.replan_reopt_drone = check_edges_sichtfeld_drone(temp_second_node, delivery_solution_model, arcs_status_after_surveillance, whether_the_node_visited);
				};
				if (loc_reopt_params.replan_reopt_drone || (loc_reopt_params.replan_reopt_truck && areEqual(drone_duration,loc_reopt_params.truck_duration))) {
					loc_reopt_params.curr_drone_node = temp_second_node;
					break;
				}
			}
			if (!drone_customer_delivered && temp_second_node == delivery_solution_model.drone_customers[loc_i].back()) {
				delivery_sol.drone_customers.back() = delivery_solution_model.drone_customers[loc_i];
				loc_reopt_params.curr_drone_cust = -1;
				drone_customer_delivered = true;
				delivery_sol.list_of_op_customers.back().push_back(temp_second_node);
				drone_duration = drone_duration+cmd_inputs.service_time;
				whether_the_customer_node_visited[temp_second_node] = true;
				loc_temp_customers_to_visit[temp_second_node] = false;
				loc_reopt_params.n_visited_nodes = loc_reopt_params.n_visited_nodes + 1;
			}
			loc_reopt_params.curr_drone_node = temp_second_node;
			first_node = temp_second_node;
		}
	}
	else {
		delivery_sol.drone_customers.back() = delivery_solution_model.drone_customers[loc_i];
		delivery_sol.drone_sorties.back().push_back(-1);
		loc_reopt_params.curr_drone_node = loc_reopt_params.curr_truck_node;
		drone_duration = loc_reopt_params.truck_duration;
		if (!drone_customer_delivered) {
			loc_reopt_params.curr_drone_cust = delivery_solution_model.drone_customers[loc_i].back();
		}
		return drone_duration;
	};
	if (!drone_customer_delivered) {
		loc_reopt_params.curr_drone_cust = delivery_solution_model.drone_customers[loc_i].back();
	}
	//CASE that the duration is exactly equal, we do need an artificial node to close the operation
	if (loc_reopt_params.replan_reopt_truck && (drone_duration < loc_reopt_params.truck_duration)) {
		ArtificialNode new_artificial_node(loc_reopt_params.next_atificial_node_id, instance.n_nodes,loc_reopt_params.curr_drone_node,loc_reopt_params.curr_drone_node,0, loc_reopt_params.truck_duration,loc_reopt_params.truck_duration, cmd_inputs.service_time, false, delivery_solution_model.art_node_map,delivery_solution_model.drone_distances);
		new_artificial_node.real_incoming_edge.second = (loc_reopt_params.truck_duration-drone_duration);
		new_artificial_node.real_outgoing_edge.second = 0;
		new_artificial_node.artificial_outgoing_edge.second = std::numeric_limits<double>::infinity();
		loc_reopt_params.curr_drone_node = loc_reopt_params.next_atificial_node_id;
		loc_reopt_params.artificial_node = new_artificial_node;
		drone_duration = loc_reopt_params.truck_duration;
	}
	return drone_duration;
};

std::pair<bool, bool> Simulation::check_replan_reopt(int loc_i,Solution& delivery_solution, vector<bool>& loc_temp_customers_to_visit)
//0 false no replan
//1 drone detects replan
//2 truck detects replan
//case 4 where both decects replan converts to case 1 or 2
{
	//replan[0] truck detects damaged edge, replan[1] drone detects damaged edge
	std::pair<bool, bool> replan = { false,false };
	vector<bool> visited_nodes = vector<bool>(instance.n_nodes,false);
	double drone_duration = 0;
	double truck_duration = 0;

	int first_node = delivery_solution.truck_sorties[loc_i].front();
	for (int loc_j = 1; loc_j < (int)delivery_solution.truck_sorties[loc_i].size(); loc_j++) {
		int second_node = delivery_solution.truck_sorties[loc_i][loc_j];

		//check next arc
		if(first_node<instance.n_nodes && second_node < instance.n_nodes){
			if ( (iterations.back().current_damages[first_node][second_node] <= -1)) {
				replan.first = true;
				break;
			};
		}
		double distance_move = delivery_solution.truck_distances[first_node][second_node];
		truck_duration = truck_duration + distance_move;
		if (second_node < instance.n_nodes) {
			for (int loc_f = 0; loc_f < instance.adjacent_nodes[second_node][0]; loc_f++) {
				int node_candidate = instance.adjacent_nodes[second_node][loc_f + 1];
				if (((delivery_solution.list_truck_edges[second_node][node_candidate] >= 1) || (delivery_solution.list_truck_edges[node_candidate][second_node] >= 1)) && ((iterations.back().current_damages[second_node][node_candidate] <= -1) || (iterations.back().current_damages[node_candidate][second_node] <= -1))) { //&& (truck_tour_edges.find({ second_node,node_candidate }) != truck_tour_edges.end() || truck_tour_edges.find({ node_candidate,second_node }) != truck_tour_edges.end())) {
					replan.first = true;
				};
			};
		}
		if (replan.first) {
			break;
		}
		if((second_node!= delivery_solution.drone_customers[loc_i].back()) && visited_nodes[second_node]==false && (std::find(delivery_solution.list_of_op_customers[loc_i].begin(), delivery_solution.list_of_op_customers[loc_i].end(), second_node) != delivery_solution.list_of_op_customers[loc_i].end())) {
			truck_duration = truck_duration + delivery_solution.service_time_map[second_node];
			visited_nodes[second_node] = true;
		}
		first_node = second_node;
	};
	first_node = delivery_solution.drone_sorties[loc_i].front();


	if (first_node > -1 || delivery_solution.drone_customers[loc_i].front()>-1) {
		for (int loc_j = 1; loc_j < (int)delivery_solution.drone_sorties[loc_i].size(); loc_j++) {
			int temp_second_node = delivery_solution.drone_sorties[loc_i][loc_j];
			double distance_move = delivery_solution.drone_distances[first_node][temp_second_node];
			drone_duration = drone_duration + distance_move;
			if (temp_second_node < instance.n_nodes) {
				for (int loc_f = 0; loc_f < instance.adjacent_nodes[temp_second_node][0]; loc_f++) {
					int node_candidate = instance.adjacent_nodes[temp_second_node][loc_f + 1];
					if (((delivery_solution.list_truck_edges[temp_second_node][node_candidate] >= 1) || (delivery_solution.list_truck_edges[node_candidate][temp_second_node] >= 1)) && ((iterations.back().current_damages[temp_second_node][node_candidate] <= -1) || (iterations.back().current_damages[node_candidate][temp_second_node] <= -1))) {
						replan.second = true;
					};
				};
			}
			first_node = temp_second_node;
			if (replan.second) {
				break;
			}
			if((temp_second_node == delivery_solution.drone_customers[loc_i].back()) && (visited_nodes[temp_second_node]==false)) {
				drone_duration = drone_duration + delivery_solution.service_time_map[temp_second_node];
				visited_nodes[temp_second_node] = true;
			}
		}
	}
	else {
		drone_duration = truck_duration;
	}

		//For loc_i loop end
	if (replan.first && replan.second) {
		if (truck_duration<drone_duration) {
			replan.second = false;
		}
		else {
			replan.first = false;
		}
	}

	return replan;
}


std::pair<bool, bool> Simulation::check_replan_reopt_sichtfeld(int loc_i,Solution& delivery_solution, vector<bool>& loc_temp_customers_to_visit)
//0 false no replan
//1 drone detects replan
//2 truck detects replan
//case 4 where both decects replan converts to case 1 or 2
{
	//replan[0] truck detects damaged edge, replan[1] drone detects damaged edge
	std::pair<bool, bool> replan = { false,false };
	vector<bool> visited_nodes = vector<bool>(instance.n_nodes,false);
	double drone_duration = 0;
	double truck_duration = 0;

	int first_node = delivery_solution.truck_sorties[loc_i].front();
	for (int loc_j = 1; loc_j < (int)delivery_solution.truck_sorties[loc_i].size(); loc_j++) {
		int second_node = delivery_solution.truck_sorties[loc_i][loc_j];

		//check next arc
		if(first_node<instance.n_nodes && second_node < instance.n_nodes){
			if ( (iterations.back().current_damages[first_node][second_node] <= -1)) {
				replan.first = true;
				break;
			};
		}
		double distance_move = delivery_solution.truck_distances[first_node][second_node];
		truck_duration = truck_duration + distance_move;
		if (second_node < instance.n_nodes) {
			for (int loc_f = 0; loc_f < instance.adjacent_nodes[second_node][0]; loc_f++) {
				int node_candidate = instance.adjacent_nodes[second_node][loc_f + 1];
				if ((delivery_solution.list_truck_edges[second_node][node_candidate] >= 1 || delivery_solution.list_truck_edges[node_candidate][second_node] >= 1) && (iterations.back().current_damages[second_node][node_candidate] <= -1 || iterations.back().current_damages[node_candidate][second_node] <= -1)) { //&& (truck_tour_edges.find({ second_node,node_candidate }) != truck_tour_edges.end() || truck_tour_edges.find({ node_candidate,second_node }) != truck_tour_edges.end())) {
					replan.first = true;
				};
			};
		}
		if (replan.first) {
			break;
		}
		if((second_node!= delivery_solution.drone_customers[loc_i].back()) && visited_nodes[second_node]==false && (std::find(delivery_solution.list_of_op_customers[loc_i].begin(), delivery_solution.list_of_op_customers[loc_i].end(), second_node) != delivery_solution.list_of_op_customers[loc_i].end())) {
			truck_duration = truck_duration + delivery_solution.service_time_map[second_node];
			visited_nodes[second_node] = true;
		}
		first_node = second_node;
	};
	first_node = delivery_solution.drone_sorties[loc_i].front();


	if (first_node > -1 || delivery_solution.drone_customers[loc_i].front()>-1) {
		for (int loc_j = 1; loc_j < (int)delivery_solution.drone_sorties[loc_i].size(); loc_j++) {
			int temp_second_node = delivery_solution.drone_sorties[loc_i][loc_j];
			double distance_move = delivery_solution.drone_distances[first_node][temp_second_node];
			drone_duration = drone_duration + distance_move;
			if (temp_second_node < instance.n_nodes) {
				for (int loc_f = 0; loc_f < instance.adjacent_nodes[temp_second_node][0]; loc_f++) {
					int node_candidate = instance.adjacent_nodes[temp_second_node][loc_f + 1];
					if ((delivery_solution.list_truck_edges[temp_second_node][node_candidate] >= 1 || delivery_solution.list_truck_edges[node_candidate][temp_second_node] >= 1) && (iterations.back().current_damages[temp_second_node][node_candidate] <= -1 || iterations.back().current_damages[node_candidate][temp_second_node] <= -1)) {
						replan.second = true;
					};
					int temp_next_node = node_candidate;
					while((node_candidate>=instance.real_n_nodes) && visited_nodes[node_candidate] == false){
						for (int i = 0; i < instance.adjacent_nodes[node_candidate][0]; i++) {
							visited_nodes[node_candidate] = true;
							int node_candidate2 = instance.adjacent_nodes[node_candidate][i + 1];
							if ((delivery_solution.list_truck_edges[node_candidate][node_candidate2] >= 1 || delivery_solution.list_truck_edges[node_candidate2][node_candidate] >= 1) && (iterations.back().current_damages[node_candidate][node_candidate2] <= -1 || iterations.back().current_damages[node_candidate2][node_candidate]<= -1)) {
								replan.second = true;
							};
							if(node_candidate2>=instance.real_n_nodes){
								temp_next_node = node_candidate2;
							}
						}
						node_candidate = temp_next_node;
					}
				};
			}
			first_node = temp_second_node;
			if (replan.second) {
				break;
			}
			if((temp_second_node == delivery_solution.drone_customers[loc_i].back()) && (visited_nodes[temp_second_node]==false)) {
				drone_duration = drone_duration + delivery_solution.service_time_map[temp_second_node];
				visited_nodes[temp_second_node] = true;
			}
		}
	}
	else {
		drone_duration = truck_duration;
	}

		//For loc_i loop end
	if (replan.first && replan.second) {
		if (truck_duration<drone_duration) {
			replan.second = false;
		}
		else {
			replan.first = false;
		}
	}

	return replan;
}


bool Simulation::check_edges(int curr_node_position, Solution& delivery_solution_model, vector<vector<int>> &arcs_status_after_surveillance, vector<bool> &whether_the_node_visited){
	bool replan = false;
	if (whether_the_node_visited[curr_node_position] == false) {
		whether_the_node_visited[curr_node_position] = true;
		for (int loc_f = 0; loc_f < instance.adjacent_nodes[curr_node_position][0]; loc_f++) {
			int node_candidate = instance.adjacent_nodes[curr_node_position][loc_f + 1];
			arcs_status_after_surveillance[curr_node_position][node_candidate] = iterations.back().current_damages[curr_node_position][node_candidate];
			arcs_status_after_surveillance[node_candidate][curr_node_position] = iterations.back().current_damages[node_candidate][curr_node_position];
			if ((delivery_solution_model.list_truck_edges[curr_node_position][node_candidate] >= 1 || delivery_solution_model.list_truck_edges[node_candidate][curr_node_position] >= 1) && ((arcs_status_after_surveillance[curr_node_position][node_candidate] <= -1) || (arcs_status_after_surveillance[node_candidate][curr_node_position] <= -1))) {
				replan  = true;
			};
		};
	}
	return replan;
}

bool Simulation::check_edges_sichtfeld_drone(int curr_node_position, Solution& delivery_solution_model, vector<vector<int>> &arcs_status_after_surveillance, vector<bool> &whether_the_node_visited){
	bool replan = false;
	whether_the_node_visited[curr_node_position] = true;
	for (int loc_f = 0; loc_f < instance.adjacent_nodes[curr_node_position][0]; loc_f++) {
		int node_candidate = instance.adjacent_nodes[curr_node_position][loc_f + 1];
		arcs_status_after_surveillance[curr_node_position][node_candidate] = iterations.back().current_damages[curr_node_position][node_candidate];
		arcs_status_after_surveillance[node_candidate][curr_node_position] = iterations.back().current_damages[node_candidate][curr_node_position];
		if ((delivery_solution_model.list_truck_edges[curr_node_position][node_candidate] >= 1 || delivery_solution_model.list_truck_edges[node_candidate][curr_node_position] >= 1) && ((arcs_status_after_surveillance[curr_node_position][node_candidate] <= -1) || (arcs_status_after_surveillance[node_candidate][curr_node_position] <= -1))) {			replan  = true;
		};
		int temp_next_node = node_candidate;
		while((node_candidate>=instance.real_n_nodes) && whether_the_node_visited[node_candidate] == false){
			for (int i = 0; i < instance.adjacent_nodes[node_candidate][0]; i++) {
				int node_candidate2 = instance.adjacent_nodes[node_candidate][i + 1];
				whether_the_node_visited[node_candidate] = true;
				arcs_status_after_surveillance[node_candidate][node_candidate2] = iterations.back().current_damages[node_candidate][node_candidate2];
				arcs_status_after_surveillance[node_candidate2][node_candidate] = iterations.back().current_damages[node_candidate2][node_candidate];
				if ((delivery_solution_model.list_truck_edges[node_candidate][node_candidate2] >= 1 || delivery_solution_model.list_truck_edges[node_candidate2][node_candidate] >= 1) && ((arcs_status_after_surveillance[node_candidate][node_candidate2] <= -1) || (arcs_status_after_surveillance[node_candidate2][node_candidate] <= -1))) {
					replan  = true;
				};
				if(node_candidate2>=instance.real_n_nodes){
					temp_next_node = node_candidate2;
				}
			}
			node_candidate = temp_next_node;
		}
	};
	return replan;
}

bool Simulation::surveillance_first_naive(vector<vector<int>>& arc_status_after_surveillance, double& cost_of_surveillance, double& cost_of_delivery, Solution& delivery_sol, Solution& surveillance_solution, int surveillance_first_setting, bool arc_based_tsp) {
	double duration_of_surveillance = 0;
	bool stop_surveillance = false;
	bool replan = false;
	bool solution_OK = true;
	vector<bool>loc_temp_customers_to_visit = instance.customers_to_visit;
	loc_temp_customers_to_visit[0] = false;
	//compute optimistic delivery route
	vector<vector<int>> loc_optimistic_arcs;
	for (int loc_f = 0; loc_f < instance.n_nodes; loc_f++) {
		loc_optimistic_arcs.push_back(arc_status_after_surveillance[loc_f]);
		for (int loc_r = 0; loc_r < instance.adjacent_nodes[loc_f][0]; loc_r++) {
			int neighbornode_loc = instance.adjacent_nodes[loc_f][loc_r + 1];
			if (loc_optimistic_arcs[loc_f][neighbornode_loc] == 0) {
				loc_optimistic_arcs[loc_f][neighbornode_loc] = 1;
			};
		};
	};

	surveillance_solution.drone_sorties.resize(0);
	surveillance_solution.truck_sorties.resize(0);
	surveillance_solution.drone_sorties.push_back(vector<int>(1, 0));
	surveillance_solution.n_stages = 1;
	surveillance_solution.obj_value = 0;
	surveillance_solution.drone_distances = instance.drone_distances;
	surveillance_solution.previous_drone_node = instance.drone_fromtriple_previousnode;

	vector<bool>nodes_to_visit = vector<bool>(instance.n_nodes, true);
	vector<bool>visited_nodes = vector<bool>(instance.n_nodes, true);
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		nodes_to_visit[loc_i] = false;
		visited_nodes[loc_i] = false;
	};
	int curr_position = 0;
	visited_nodes[curr_position] = true;
	int n_nodes_to_visit;

	iterations.back().n_replan_surveillance_first = -1;
	int casecode = 2; 
	while (stop_surveillance == false) {
		replan = false;
		iterations.back().n_replan_surveillance_first = iterations.back().n_replan_surveillance_first + 1;
		//initialize
		delivery_sol.truck_sorties.resize(0);
		delivery_sol.drone_sorties.resize(0);
		delivery_sol.list_truck_edges.resize(0);
		Model_DP temp_model_dp(instance, loc_optimistic_arcs,arc_status_after_surveillance, 0, loc_temp_customers_to_visit,cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time);
		switch (surveillance_first_setting) {
			case 0: {
				std::logic_error("Heuristic functions not yet implemented");
				//Model_heuristic Model_heuristic(instance, loc_optimistic_arcs, 0, temp_customers_to_visit, delivery_sol, 0, 0, false);
				break;
			}
			default: {
				//Model model(instance, cmd_inputs.maxn_stages_long, loc_optimistic_arcs, 0, temp_customers_to_visit, delivery_sol, false);
				//check if there is any customer to visit left
				if (std::find(begin(loc_temp_customers_to_visit), end(loc_temp_customers_to_visit), true) == end(loc_temp_customers_to_visit)) {
					delivery_sol.obj_value = 0;
					delivery_sol.n_stages = 0;
				}
				else {
					temp_model_dp.solve(surveillance_first_setting);
					delivery_sol = temp_model_dp.model_solution;
					solution_OK = temp_model_dp.check_solution(temp_model_dp.model_solution, loc_optimistic_arcs);
					iterations.back().mistake_reported = temp_model_dp.model_solution.mistake_reported;
					iterations.back().mistake_text = iterations.back().mistake_text + temp_model_dp.model_solution.mistake_text;
					update_planned_truck_arcs(casecode,temp_model_dp.Ledges_struct.Ledges_from_end_to_start);
					iterations.back().damaged_edges_in_solution.surv = iterations.back().damaged_edges_in_solution.surv+delivery_sol.get_damaged_trucks_arcs(iterations.back().current_damages);
					if(iterations.back().n_replan_surveillance_first==0){
						iterations.back().optimistic_sol_operations = (int) temp_model_dp.model_solution.truck_sorties.size();
						iterations.back().optimistic_initial_solution = delivery_sol.obj_value;
					}
				};
				break;
			}
		}
		cost_of_delivery = delivery_sol.obj_value;

		//find the route of the drone
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			nodes_to_visit[loc_i] = false;
		};
		n_nodes_to_visit = 0;
		std::set<int> truck_customers;
		for (int loc_i = 0; loc_i < delivery_sol.n_stages; loc_i++) {
			for (int loc_j = 0; loc_j < (int)delivery_sol.truck_sorties[loc_i].size(); loc_j++) {
				int node_number = delivery_sol.truck_sorties[loc_i][loc_j];
				//make sure to check if the nodenumber is not a lookout node for sichtfeld
				if ((visited_nodes[node_number] == false) && (nodes_to_visit[node_number] == false) && (node_number<instance.real_n_nodes)) {
					nodes_to_visit[node_number] = true;
					truck_customers.insert(node_number);
					n_nodes_to_visit = n_nodes_to_visit + 1;
				};
			};
		};
		if (n_nodes_to_visit <= 0) {
			stop_surveillance = true;
		}
		else {		
			Model_TSP Model_TSP(instance,temp_model_dp.Ledges_struct.Ledges_from_end_to_start, arc_status_after_surveillance, nodes_to_visit,cmd_inputs.truck_distance_metric_manhattan,curr_position, curr_position, arc_based_tsp, iterations.back().tilim);
			Model_TSP.solve();
			Solution drone_tour = Model_TSP.model_solution;
			if(arc_based_tsp==true){
				//if new surveillance policy is used
				Model_TSP.check_solution_new_tsp(Model_TSP.model_solution,loc_optimistic_arcs);
				for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
					nodes_to_visit[loc_i] = false;
				};
				n_nodes_to_visit = 0;
				for (int loc_i = 0; loc_i < drone_tour.n_stages; loc_i++) {
					for (int loc_j = 0; loc_j < (int) drone_tour.drone_sorties[loc_i].size(); loc_j++) {
						int node_number = drone_tour.drone_sorties[loc_i][loc_j];
						if((node_number!=curr_position) && (nodes_to_visit[node_number]==false && (visited_nodes[node_number] == false))) {
							nodes_to_visit[node_number] = true;
							n_nodes_to_visit = n_nodes_to_visit+1;
						}
					}
				}
			}
			else{
				Model_TSP.check_solution_tsp(Model_TSP.model_solution,loc_optimistic_arcs);
			};
			//drone starts surveillance along the truck route until all the edges are known.
			for (int loc_i = 0; loc_i < drone_tour.n_stages; loc_i++) {
				int first_node = drone_tour.drone_sorties[loc_i][0];
				int second_node = drone_tour.drone_sorties[loc_i][0];
				for (int loc_j = 0; loc_j < (int)drone_tour.drone_sorties[loc_i].size(); loc_j++) {
					first_node = second_node;
					second_node = drone_tour.drone_sorties[loc_i][loc_j];
					surveillance_solution.obj_value = surveillance_solution.obj_value + instance.drone_distances[surveillance_solution.drone_sorties.back().back()][second_node];
					duration_of_surveillance = duration_of_surveillance + instance.drone_distances[first_node][second_node];
					surveillance_solution.drone_sorties.back().push_back(second_node);
					if ((nodes_to_visit[second_node] == true) && (visited_nodes[second_node] == false)) {
						visited_nodes[second_node] = true;
						iterations.back().surveillance_nodes.insert(second_node);
						n_nodes_to_visit = n_nodes_to_visit - 1;
					};
					//get information on new arcs
					for (int loc_l = 0; loc_l < instance.adjacent_nodes[second_node][0]; loc_l++) {
						int third_node = instance.adjacent_nodes[second_node][loc_l + 1];
						arc_status_after_surveillance[second_node][third_node] = iterations.back().current_damages[second_node][third_node];
						arc_status_after_surveillance[third_node][second_node] = iterations.back().current_damages[second_node][third_node];
						loc_optimistic_arcs[second_node][third_node] = iterations.back().current_damages[second_node][third_node];
						loc_optimistic_arcs[third_node][second_node] = iterations.back().current_damages[second_node][third_node];
						if (((delivery_sol.list_truck_edges[second_node][third_node] >= 1) || (delivery_sol.list_truck_edges[third_node][second_node] >= 1)) && ((iterations.back().current_damages[second_node][third_node] <= -1) || (iterations.back().current_damages[third_node][second_node] <= -1))) {
							replan = true;
						};
						if(cmd_inputs.setting_sichtfeld>0) {
							int temp_next_node = third_node;
							while((third_node>=instance.real_n_nodes)&& (visited_nodes[third_node] == false)){
								for (int i = 0; i < instance.adjacent_nodes[third_node][0]; i++) {
									if (visited_nodes[third_node] == false) {
										visited_nodes[third_node] = true;
									};
									int node_candidate2 = instance.adjacent_nodes[third_node][i + 1];
									arc_status_after_surveillance[third_node][node_candidate2] = iterations.back().current_damages[third_node][node_candidate2];
									arc_status_after_surveillance[node_candidate2][third_node] = iterations.back().current_damages[node_candidate2][third_node];
									loc_optimistic_arcs[third_node][node_candidate2] = iterations.back().current_damages[third_node][node_candidate2];
									loc_optimistic_arcs[node_candidate2][third_node] = iterations.back().current_damages[node_candidate2][third_node];
									if (((delivery_sol.list_truck_edges[third_node][node_candidate2] >= 1) || (delivery_sol.list_truck_edges[node_candidate2][third_node] >= 1)) && ((arc_status_after_surveillance[third_node][node_candidate2] <= -1) || (arc_status_after_surveillance[node_candidate2][second_node] <= -1))) {
										replan  = true;
									};
									if(node_candidate2>=instance.real_n_nodes){
										temp_next_node = node_candidate2;
									}
								}
								third_node = temp_next_node;
							}
						};
					};
					//if replanning, then stop
					if (n_nodes_to_visit <= 0 && !replan) {
						stop_surveillance = true;
					};
					if ((stop_surveillance == true) || (replan == true)) {
						curr_position = second_node;
						break;
					};
				}//for loc_j
				if ((stop_surveillance == true) || (replan == true)) {
					break;
				};
			};//for loc_i
		};//if there are nodes to visit
	};//while

	if (surveillance_solution.drone_sorties.back().size() == 1) {
		surveillance_solution.drone_sorties.back().push_back(0);
	};

	if (stop_surveillance && curr_position != 0) {
		surveillance_solution.drone_sorties.back().push_back(0);
		surveillance_solution.obj_value = surveillance_solution.obj_value + instance.drone_distances[curr_position][0];
		duration_of_surveillance = duration_of_surveillance + instance.drone_distances[curr_position][0];
		curr_position = 0;
	}

	cost_of_surveillance = duration_of_surveillance;

	if (areEqual(surveillance_solution.obj_value,cost_of_surveillance)==false) {
		cout << "MISTAKE_in_survfirst: two different calculated surveillance cost values; " << '\n';
		iterations.back().mistake_reported = true;
		iterations.back().mistake_text = iterations.back().mistake_text + "MISTAKE_in_survfirst: two different calculated surveillance cost values; ";
	};
	return solution_OK;
}

bool Simulation::surveillance_first_orienteering_variant(vector<vector<int>> &arc_status_after_surveillance, double &cost_of_surveillance, double &cost_of_delivery, Solution &delivery_sol, Solution &surveillance_solution, int simulation_setting, double cmax, double penalty_factor, bool arc_based_tsp)
{
    double duration_of_surveillance = 0;
	bool stop_surveillance = false;
	bool replan = false;
	bool solution_OK = true;
	vector<bool>loc_temp_customers_to_visit = instance.customers_to_visit;
	loc_temp_customers_to_visit[0] = false;
	
	//compute optimistic delivery route
	vector<vector<int>> loc_optimistic_arcs;
	for (int loc_f = 0; loc_f < instance.n_nodes; loc_f++) {
		loc_optimistic_arcs.push_back(arc_status_after_surveillance[loc_f]);
		for (int loc_r = 0; loc_r < instance.adjacent_nodes[loc_f][0]; loc_r++) {
			int neighbornode_loc = instance.adjacent_nodes[loc_f][loc_r + 1];
			if (loc_optimistic_arcs[loc_f][neighbornode_loc] == 0) {
				loc_optimistic_arcs[loc_f][neighbornode_loc] = 1;
			};
		};
	};

	surveillance_solution.drone_sorties.resize(0);
	surveillance_solution.truck_sorties.resize(0);
	surveillance_solution.drone_sorties.push_back(vector<int>(1, 0));
	surveillance_solution.n_stages = 1;
	surveillance_solution.obj_value = 0;
	surveillance_solution.drone_distances = instance.drone_distances;
	surveillance_solution.previous_drone_node = instance.drone_fromtriple_previousnode;

	vector<bool>nodes_to_visit = vector<bool>(instance.n_nodes, true);
	vector<bool>visited_nodes = vector<bool>(instance.n_nodes, true);
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		nodes_to_visit[loc_i] = false;
		visited_nodes[loc_i] = false;
	};
	int curr_position = 0;
	visited_nodes[curr_position] = true;
	int n_nodes_to_visit;

	iterations.back().n_replan_surveillance_op_survphase = -1;
	//reopt phase dp model init
	surv_op_reopt_parameters = reoptParams();
	
	double curr_penalty_factor = penalty_factor;
	int casecode = 4; 
	while (stop_surveillance == false) {
		replan = false;
		iterations.back().n_replan_surveillance_op_survphase = iterations.back().n_replan_surveillance_op_survphase+ 1;
		//initialize
		delivery_sol.truck_sorties.resize(0);
		delivery_sol.drone_sorties.resize(0);
		delivery_sol.list_truck_edges.resize(0);
		//adapt penalty factor if needed
		if(iterations.back().n_replan_surveillance_op_survphase==0) {
			curr_penalty_factor = 1.0;
		}
		else{
			curr_penalty_factor = penalty_factor;
		}
		Model_DP temp_model_dp(instance, loc_optimistic_arcs, arc_status_after_surveillance, 0, loc_temp_customers_to_visit,cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time, curr_penalty_factor);
		//update known arc status before surveillance
		//add arc status at model input!
		
		switch (simulation_setting) {
			case 0: {
				std::logic_error("Heuristic functions not yet implemented");
				//Model_heuristic Model_heuristic(instance, loc_optimistic_arcs, 0, temp_customers_to_visit, delivery_sol, 0, 0, false);
				break;
			}
			default: {
				//Model model(instance, cmd_inputs.maxn_stages_long, loc_optimistic_arcs, 0, temp_customers_to_visit, delivery_sol, false);
				//check if there is any customer to visit left
				if (std::find(begin(loc_temp_customers_to_visit), end(loc_temp_customers_to_visit), true) == end(loc_temp_customers_to_visit)) {
					delivery_sol.obj_value = 0;
					delivery_sol.n_stages = 0;
				}
				else {
					temp_model_dp.solve(simulation_setting);
					delivery_sol = temp_model_dp.model_solution;
					solution_OK = temp_model_dp.check_solution(temp_model_dp.model_solution, loc_optimistic_arcs);
					iterations.back().mistake_reported = temp_model_dp.model_solution.mistake_reported;
					iterations.back().mistake_text = iterations.back().mistake_text + temp_model_dp.model_solution.mistake_text;
					update_planned_truck_arcs(casecode,temp_model_dp.Ledges_struct.Ledges_from_end_to_start);
					iterations.back().damaged_edges_in_solution.surv_op_surveillance = iterations.back().damaged_edges_in_solution.surv_op_surveillance+ delivery_sol.get_damaged_trucks_arcs(iterations.back().current_damages);
					if(iterations.back().n_replan_surveillance_op_survphase==0){
						iterations.back().optimistic_sol_operations = (int) temp_model_dp.model_solution.truck_sorties.size();
						iterations.back().optimistic_initial_solution = delivery_sol.obj_value;
					}
				};
				break;
			}
		}
		cost_of_delivery = delivery_sol.obj_value;

		//find the route of the drone
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			nodes_to_visit[loc_i] = false;
		};
		n_nodes_to_visit = 0;
		std::set<int> truck_customers;
		for (int loc_i = 0; loc_i < delivery_sol.n_stages; loc_i++) {
			for (int loc_j = 0; loc_j < (int)delivery_sol.truck_sorties[loc_i].size(); loc_j++) {
				int node_number = delivery_sol.truck_sorties[loc_i][loc_j];
				if ((visited_nodes[node_number] == false) && (nodes_to_visit[node_number] == false) && (node_number<instance.real_n_nodes)) {
					nodes_to_visit[node_number] = true;
					truck_customers.insert(node_number);
					n_nodes_to_visit += 1;
				};
			};
		};
		if(n_nodes_to_visit==0) {
			stop_surveillance = true;
			break;
		}
		//check if dynamic cmax or fixed cmax is used
		if(cmax<0.0 && iterations.back().n_replan_surveillance_op_survphase==0) {
			Model_TSP Model_TSP(instance,temp_model_dp.Ledges_struct.Ledges_from_end_to_start, arc_status_after_surveillance, nodes_to_visit,cmd_inputs.truck_distance_metric_manhattan,curr_position, curr_position, arc_based_tsp, iterations.back().tilim);
			Model_TSP.solve();
			if(arc_based_tsp==true){
				Model_TSP.check_solution_new_tsp(Model_TSP.model_solution,loc_optimistic_arcs);
			}
			else{
				Model_TSP.check_solution_tsp(Model_TSP.model_solution,loc_optimistic_arcs);
			}  
			cmax = Model_TSP.model_solution.obj_value*cmd_inputs.cmax_ratio;
			iterations.back().dyn_cmax = cmax;
			cout <<"cmax: " << cmax <<'\n';
		}
		else{
			iterations.back().dyn_cmax = cmax;
		} 	
		Model_OP Model_op(instance,arc_status_after_surveillance, nodes_to_visit,cmd_inputs.truck_distance_metric_manhattan,curr_position, curr_position, duration_of_surveillance, cmax, iterations.back().tilim);
		Model_op.solve();
		Model_op.check_solution_survop(Model_op.model_solution,loc_optimistic_arcs, cmax);
		Solution drone_tour = Model_op.model_solution;
		vector<bool>nodes_to_visit_surveillance = vector<bool>(instance.n_nodes, false);
		n_nodes_to_visit = 0;
		for(int stage =0;stage< (int)drone_tour.drone_sorties.size();stage++){
			for(int node: drone_tour.drone_sorties[stage]){
				if((node!=curr_position || node!= 0) && (visited_nodes[node] == false) && (nodes_to_visit_surveillance[node]==false)) {
					nodes_to_visit_surveillance[node] = true;
					n_nodes_to_visit = n_nodes_to_visit + 1;
				}
			}
		}
		if (n_nodes_to_visit <= 0) {
			stop_surveillance = true;
			break;
		}
		else{
			//drone starts surveillance along the truck route until all the edges are known.
			for (int loc_i = 0; loc_i < drone_tour.n_stages; loc_i++) {
				int first_node = drone_tour.drone_sorties[loc_i][0];
				int second_node = drone_tour.drone_sorties[loc_i][0];
				for (int loc_j = 0; loc_j < (int)drone_tour.drone_sorties[loc_i].size(); loc_j++) {
					first_node = second_node;
					second_node = drone_tour.drone_sorties[loc_i][loc_j];
					surveillance_solution.drone_sorties.back().push_back(second_node);
					duration_of_surveillance = duration_of_surveillance + instance.drone_distances[first_node][second_node];
					if ((nodes_to_visit_surveillance[second_node] == true) && (visited_nodes[second_node] == false)) {
						visited_nodes[second_node] = true;
						n_nodes_to_visit -= 1;
						iterations.back().surveillance_op_nodes.insert(second_node);
					};
					//get information on new arcs
					for (int loc_l = 0; loc_l < instance.adjacent_nodes[second_node][0]; loc_l++) {
						int third_node = instance.adjacent_nodes[second_node][loc_l + 1];
						arc_status_after_surveillance[second_node][third_node] = iterations.back().current_damages[second_node][third_node];
						arc_status_after_surveillance[third_node][second_node] = iterations.back().current_damages[second_node][third_node];
						loc_optimistic_arcs[second_node][third_node] = iterations.back().current_damages[second_node][third_node];
						loc_optimistic_arcs[third_node][second_node] = iterations.back().current_damages[second_node][third_node];
						if (((delivery_sol.list_truck_edges[second_node][third_node] >= 1) || (delivery_sol.list_truck_edges[third_node][second_node] >= 1)) && ((iterations.back().current_damages[second_node][third_node] <= -1) || (iterations.back().current_damages[third_node][second_node] <= -1))) {
							if(duration_of_surveillance<cmax) {
								replan = true;
							}
						};
						//additional surveillance for sichtfeld
						if(cmd_inputs.setting_sichtfeld>0) {
							int temp_next_node = third_node;
							while((third_node>=instance.real_n_nodes)&& (visited_nodes[third_node] == false)){
								for (int i = 0; i < instance.adjacent_nodes[third_node][0]; i++) {
									int node_candidate2 = instance.adjacent_nodes[third_node][i + 1];
									if (visited_nodes[third_node] == false) {
										visited_nodes[third_node] = true;
									};
									arc_status_after_surveillance[third_node][node_candidate2] = iterations.back().current_damages[third_node][node_candidate2];
									arc_status_after_surveillance[node_candidate2][third_node] = iterations.back().current_damages[node_candidate2][third_node];
									loc_optimistic_arcs[third_node][node_candidate2] = iterations.back().current_damages[third_node][node_candidate2];
									loc_optimistic_arcs[node_candidate2][third_node] = iterations.back().current_damages[node_candidate2][third_node];
									if (((delivery_sol.list_truck_edges[second_node][third_node] >= 1) || (delivery_sol.list_truck_edges[third_node][second_node] >= 1)) && ((iterations.back().current_damages[second_node][third_node] <= -1) || (iterations.back().current_damages[third_node][second_node] <= -1))) {
										if(duration_of_surveillance<cmax) {
											replan  = true;
										}
									};
									if(node_candidate2>=instance.real_n_nodes){
										temp_next_node = node_candidate2;
									}
								}
								third_node = temp_next_node;
							}
						};
					};
					//if replanning, then stop
					curr_position = second_node;
					if (!replan && n_nodes_to_visit<=0) {
						stop_surveillance = true;
					};
					if ((stop_surveillance == true) || (replan == true)) {
						curr_position = second_node;
						break;
					};
				}//for loc_j
				if ((stop_surveillance == true) || (replan == true)) {
					break;
				};
			};//for loc_i
		};//if there are nodes to visit
	};//while

	if (surveillance_solution.drone_sorties.back().size() == 1) {
		surveillance_solution.drone_sorties.back().push_back(0);
	};

	if (stop_surveillance && curr_position != 0) {
		surveillance_solution.drone_sorties.back().push_back(0);
		duration_of_surveillance = duration_of_surveillance + instance.drone_distances[curr_position][0];
		curr_position = 0;
	}

	cost_of_surveillance = duration_of_surveillance;
	surveillance_solution.obj_value = duration_of_surveillance;
	vector<bool>whether_the_node_visited = vector<bool>(instance.n_nodes);
	whether_the_node_visited[0] = true;
	vector<bool>whether_the_customer_node_visited = vector<bool>(instance.n_nodes);
	whether_the_node_visited[0] = true;
	
	surv_op_reopt_parameters.next_atificial_node_id = instance.n_nodes;

	double time_up_to_now = 0.0;

	//Initialize
	if(iterations.back().n_replan_surveillance_op_survphase==0) {
			curr_penalty_factor = 1.0;
	}
	else{
			curr_penalty_factor = penalty_factor;
	}
	Model_DP euler_model(instance, loc_optimistic_arcs, arc_status_after_surveillance, 0, loc_temp_customers_to_visit,cmd_inputs.truck_distance_metric_manhattan, cmd_inputs.service_time, curr_penalty_factor);
	iterations.back().n_replan_survop_reopt_last_operation = 0;
	while (surv_op_reopt_parameters.replan_reopt_truck == true || surv_op_reopt_parameters.replan_reopt_drone == true) {
		surv_op_reopt_parameters.replan_reopt_truck = false;
		surv_op_reopt_parameters.replan_reopt_drone = false;
		//change penalty.
		if(iterations.back().n_replan_surveillance_op_survphase==0 && surv_op_reopt_parameters.n_replan_reopt==0) {
			euler_model.penalty_factor = 1.0;
		}
		else {
			euler_model.penalty_factor  = penalty_factor;
		}
		surv_op_reopt_parameters.n_replan_reopt = surv_op_reopt_parameters.n_replan_reopt + 1;
		if (surv_op_reopt_parameters.n_replan_reopt == 0) {
			euler_model.solve(simulation_setting);
			solution_OK = euler_model.check_solution(euler_model.model_solution, loc_optimistic_arcs);
			update_planned_truck_arcs(casecode,euler_model.Ledges_struct.Ledges_from_end_to_start);
			iterations.back().damaged_edges_in_solution.surv_op_delivery = iterations.back().damaged_edges_in_solution.surv_op_delivery+ euler_model.model_solution.get_damaged_trucks_arcs(iterations.back().current_damages);
			if(iterations.back().n_replan_surveillance_op_survphase==0 && surv_op_reopt_parameters.n_replan_reopt==0){
				iterations.back().optimistic_sol_operations = (int) euler_model.model_solution.truck_sorties.size();
			}
		}
		else{
			euler_model.solve_reopt(loc_optimistic_arcs, arc_status_after_surveillance, loc_temp_customers_to_visit, surv_op_reopt_parameters, simulation_setting);
			surv_op_reopt_parameters.next_atificial_node_id = euler_model.dp.n_nodes + euler_model.dp.artificial_nodes;
			solution_OK = euler_model.check_solution(euler_model.model_solution, loc_optimistic_arcs);
			update_planned_truck_arcs(casecode,euler_model.Ledges_struct.Ledges_from_end_to_start);
			iterations.back().damaged_edges_in_solution.surv_op_delivery = iterations.back().damaged_edges_in_solution.surv_op_delivery+ euler_model.model_solution.get_damaged_trucks_arcs(iterations.back().current_damages);

		}
		for (int loc_i = 0; loc_i < euler_model.model_solution.n_stages; loc_i++) {
			std::pair<bool, bool> check_replan = check_replan_reopt(loc_i, euler_model.model_solution, loc_temp_customers_to_visit);
			if(cmd_inputs.setting_sichtfeld>0) {
				check_replan= check_replan_reopt_sichtfeld(loc_i, euler_model.model_solution, loc_temp_customers_to_visit);
			}
			double stage_duration = 0;
			surv_op_reopt_parameters.drone_duration = 0;
			surv_op_reopt_parameters.truck_duration = 0;
			
			if (!surv_op_reopt_parameters.fix_operation) {
				surv_op_reopt_parameters.last_combined_node = euler_model.model_solution.truck_sorties[loc_i].front();
				//needed for the service time accounting
				euler_model.reopt_solution.list_of_op_customers.push_back(vector<int>(0));
			}
			if (check_replan.first) {
				surv_op_reopt_parameters.truck_duration = simulate_reopt_truck_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, loc_optimistic_arcs, loc_temp_customers_to_visit, whether_the_node_visited,whether_the_customer_node_visited,surv_op_reopt_parameters);
				surv_op_reopt_parameters.drone_duration = simulate_reopt_drone_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, loc_optimistic_arcs, loc_temp_customers_to_visit, whether_the_node_visited,whether_the_customer_node_visited, surv_op_reopt_parameters);
			}
			else {
				surv_op_reopt_parameters.drone_duration = simulate_reopt_drone_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, loc_optimistic_arcs, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited, surv_op_reopt_parameters);
				surv_op_reopt_parameters.truck_duration = simulate_reopt_truck_walk(loc_i, euler_model.reopt_solution, euler_model.model_solution, loc_optimistic_arcs, loc_temp_customers_to_visit, whether_the_node_visited, whether_the_customer_node_visited,surv_op_reopt_parameters);
			}
			if((!surv_op_reopt_parameters.replan_reopt_drone && !surv_op_reopt_parameters.replan_reopt_truck && loc_i == euler_model.model_solution.n_stages-1 && euler_model.model_solution.last_delivered_customer_obj2>-1 && euler_model.model_solution.last_delivered_customers_last_delivery_op.second ==-1 )) {
				surv_op_reopt_parameters.drone_duration = 0.0;
			}	
			stage_duration = surv_op_reopt_parameters.drone_duration;
			if (surv_op_reopt_parameters.truck_duration > stage_duration) {
				stage_duration = surv_op_reopt_parameters.truck_duration;
			};
			time_up_to_now = time_up_to_now + stage_duration;
			cout << "Surv OP Reopt simulation time up to now: " << time_up_to_now << '\n';
			surv_op_reopt_parameters.fix_operation = false;
			if (surv_op_reopt_parameters.replan_reopt_drone || surv_op_reopt_parameters.replan_reopt_truck) {
				if (surv_op_reopt_parameters.curr_drone_node != surv_op_reopt_parameters.curr_truck_node) {
					surv_op_reopt_parameters.fix_operation = true;
				}
				//count replanning in last operation
				if(simulation_setting ==2 && loc_i == euler_model.model_solution.n_stages-1){
					iterations.back().n_replan_survop_reopt_last_operation = iterations.back().n_replan_survop_reopt_last_operation+1;
				}
				break;
			};

		};//for all stages
		//bool visited_all = std::none_of(loc_temp_customers_to_visit.begin(), loc_temp_customers_to_visit.end(), [](bool v) { return v; });
	};
	cost_of_delivery = time_up_to_now;
	euler_model.reopt_solution.obj_value = time_up_to_now;
	euler_model.customers_to_visit = instance.customers_to_visit;
	euler_model.reopt_solution.n_stages = euler_model.reopt_solution.truck_sorties.size();
	euler_model.last_combined_node = 0;
	euler_model.curr_drone_node = 0;
	euler_model.curr_truck_node = 0;
	euler_model.reopt_solution.mistake_text = euler_model.mistake_text;
	//euler_model.customer_set = initial_customer_set;
	euler_model.reset_model(loc_optimistic_arcs, instance.customers_to_visit, 0, 0, 0);
	euler_model.init_dp_struct(1.0,false);
	cout << "Final Check" << '\n';
	solution_OK = euler_model.check_solution(euler_model.reopt_solution, iterations.back().current_damages);
	delivery_sol = euler_model.reopt_solution;
	return solution_OK;
};

string Simulation::create_method_string() {
	string method_string = "";
	int settings_sum = cmd_inputs.setting_full_info+cmd_inputs.setting_surv_first+cmd_inputs.setting_reopt+cmd_inputs.setting_conservative+cmd_inputs.setting_surv_op;
	switch(settings_sum) {
		case 5: {
			method_string = "completion_time";
			return method_string;
		}
		case 10: {
			method_string = "last_delivery";
			return method_string;
		}
	}
	switch (cmd_inputs.setting_full_info) {
		case 0: {
			method_string = method_string + "_full_info_heuristic";
			break;
		}
		case 1: {
			method_string = method_string + "_full_info_completion_time";
			break;
		}
		case 2: {
			method_string = method_string + "_full_info_last_delivery";
			break;
		} 
	}
	switch (cmd_inputs.setting_surv_first) {
		case 0: {
			method_string = method_string + "_surveillance_first_heuristic";
			break;
		}
		case 1: {
			method_string = method_string + "_surveillance_first_completion_time";
			break;
		}
		case 2: {
			method_string = method_string + "_surveillance_last_delivery";
			break;
		}
	}
	switch (cmd_inputs.setting_surv_op) {
		case 0: {
			method_string = method_string + "_survop_heuristic";
			break;
		}
		case 1: {
			method_string = method_string + "_survop_completion_time";
			break;
		}
		case 2: {
			method_string = method_string + "_survop_last_delivery";
			break;
		}
	}
	switch (cmd_inputs.setting_reopt) {
		case 0: {
			method_string = method_string + "_reopt_heuristic";
			break;
		}
		case 1: {
			method_string = method_string + "_reopt_completion_time";
			break;
		}
		case 2: {
			method_string = method_string + "_reopt_last_delivery";
			break;
		}
	}
	switch (cmd_inputs.setting_conservative) {
		case 0: {
			method_string = method_string + "_conservative_heuristic";
			break;
		}
		case 1: {
			method_string = method_string + "_conservative_completion_time";
			break;
		}
		case 2: {
			method_string = method_string + "_conservative_last_delivery";
			break;
		}
	}
	return method_string;
}

string Simulation::write_number(double num, int precision) {
	string loc_s = "";
	int integer_part = num;
	loc_s = loc_s + to_string(integer_part);
	loc_s = loc_s + ".";
	int precision_val = pow(10.0, precision);
	int ostatok = precision_val * num; //rest auf russisch
	ostatok = (ostatok % precision_val);
	if (num < 0.0) {
		ostatok = ostatok * (-1);
	}
	if (precision > 0) {
		loc_s = loc_s + to_string(ostatok);
	}
	return loc_s;
}



string Simulation::convert_time_to_str(double duration, int precisionVal) {

	std::string trimmedString = std::to_string(duration).substr(0, std::to_string(duration).find(".") + precisionVal + 1);
	return trimmedString;
}

std::string Simulation::vecToString(const std::vector<double> &vec){
    std::ostringstream oss;
    oss << "[";

    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << ",";
        }
    }

    oss << "]";
    return oss.str();
}


bool Simulation::areEqual(double a, double b)
{
    return fabs(a-b)<0.1;
}

void Simulation::ausgabe_result_file() {

	std::ofstream OFile;
	string locfilename = "";
	string methods = create_method_string();

	locfilename = locfilename + cmd_inputs.path_to_results;
	locfilename = locfilename + "ausgabe_" + methods;
	locfilename = locfilename + "_"+ cmd_inputs.zusatz_ausgabe_filename;
	if (cmd_inputs.input_filename.length() > 0) {
		locfilename = locfilename;
	}
	locfilename = locfilename + ".csv";

	bool new_file = false;
	if (FILE* file = fopen(locfilename.c_str(), "r")) {
		fclose(file);
	}
	else {
		new_file = true;
	}

	OFile.open(locfilename, std::ios_base::app);


	if (OFile.is_open()) {
		int integer_part;
		string header = "";
		string hilf02c = "";

		try{
			header = header + "ins" + '\t' + "tilim_cplex" +  '\t' + "instance_nodes"+'\t' + "num_arcs_instance"+'\t' + "globparam_howmuch_drone_is_faster"+ '\t'+ "globparam_cmax"+ '\t'+ "globparam_cmax_ratio"+ '\t'+  "globparam_penalty_survop"+ '\t'+ "globparam_service_time"+ '\t'  + "shortcuts_allowed" + '\t' + "truck_manhattan_distance_metric" + '\t'+ "num_damaged_arcs" + '\t' + "list_of_dam_arcs" + '\t' + "globparam_input_clustered_damage" + '\t' + "globparam_input_customer_ratio" + '\t' + "list_of_customers" + '\t' + "customer_num" + '\t' +"input_parking_nodes" + '\t' + "list_of_parking_nodes" + '\t' + "parking_nodes_num" + '\t' + "optimistic_surv_reopt_ops" + '\t' + "globparam_sichtfeld_setting" +'\t'+"optimistic_initial_solution"+"\t" ;

			//instance
			string loc_s = std::to_string(instance.ins_number);
			hilf02c = hilf02c + loc_s + '\t';

			//tilim
			int loc_int = iterations.back().tilim;
			loc_s = convert_time_to_str(loc_int, 6);
			hilf02c = hilf02c + loc_s + '\t';

			//instance nodes
			hilf02c = hilf02c + to_string(instance.n_nodes) + '\t';

			//num arcs
			loc_int = 0;
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				for (int loc_j = loc_i + 1; loc_j < instance.n_nodes; loc_j++) {
					if (instance.arc_status[loc_i][loc_j] > -1) {
						loc_int = loc_int+1;
					};
				};
			};
			hilf02c = hilf02c + to_string(loc_int) + '\t';

			//drone_faster
			if (instance.s_drone > -1) {
				loc_s = write_number(instance.s_drone,2);
			}
			else {
				loc_s = "n/a";
			};
			hilf02c = hilf02c + loc_s + '\t';

			//cmax
			loc_s =  write_number(cmd_inputs.cmax,2);
			hilf02c = hilf02c + loc_s + '\t';

			//cmax_ratio
			loc_s =  write_number(cmd_inputs.cmax_ratio,2);
			hilf02c = hilf02c + loc_s + '\t';

			//penalty_surv_op
			loc_s =  to_string(cmd_inputs.penalty_factor);
			hilf02c = hilf02c + loc_s + '\t';

			//constant service_time
			loc_s =  to_string(cmd_inputs.service_time);
			hilf02c = hilf02c + loc_s + '\t';

			//shortcuts allowed
			hilf02c = hilf02c + BoolToString(cmd_inputs.shortcuts) + '\t';

			//truck manhattan distance metric
			hilf02c = hilf02c + BoolToString(cmd_inputs.truck_distance_metric_manhattan) + '\t';

			//Number of damaged arcs
			loc_s = to_string(iterations.back().n_damaged_arcs);
			hilf02c = hilf02c + loc_s + '\t';

			//List of damaged arcs
			loc_s = "[";
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
					if ((iterations.back().current_damages[loc_i][loc_j] == -1) && (instance.arc_status[loc_i][loc_j] == 0)) {
						loc_s = loc_s + "(";
						loc_s = loc_s + to_string(loc_i);
						loc_s = loc_s + ", ";
						loc_s = loc_s + to_string(loc_j);
						loc_s = loc_s + "),";
					};
				};
			};
			if(loc_s.size()>1) {
				loc_s.pop_back();
			}
			loc_s = loc_s + "]";
			hilf02c = hilf02c + loc_s + '\t';

			//arc damage clustered
			loc_s = to_string(cmd_inputs.input_clustered_damages);
			hilf02c = hilf02c + loc_s + '\t';

			//customer node ratio
			loc_s = to_string(cmd_inputs.input_required_nodes_proportion);
			hilf02c = hilf02c + loc_s + '\t';

			//list of customer nodes
			loc_int = 0;
			loc_s = "[";
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				if (instance.customers_to_visit[loc_i]) {
					loc_s = loc_s + to_string(loc_i);
					loc_s = loc_s + ", ";
					loc_int = loc_int+1;
				};
			};
			if(loc_s.size()>1) {
				loc_s.pop_back();
			}
			loc_s = loc_s + "]";
			hilf02c = hilf02c + loc_s + '\t';

			//customer_num"
			loc_s = to_string(loc_int);
			hilf02c = hilf02c + loc_s + '\t';
			//input_parking_nodes
			loc_s = to_string(cmd_inputs.input_parking_nodes);
			hilf02c = hilf02c + loc_s + '\t';
			//list_of_parking_nodes
			loc_int = 0;
			loc_s = "[";
			for (int loc_i : instance.parking_nodes) {
					loc_s = loc_s + to_string(loc_i);
					loc_s = loc_s + ", ";
					loc_int = loc_int+1;
			};
			if(loc_s.size()>1) {
				loc_s.pop_back();
			}
			loc_s = loc_s + "]";
			hilf02c = hilf02c + loc_s + '\t';
			//parking_nodes_num
			loc_s = to_string(loc_int);
			hilf02c = hilf02c + loc_s + '\t';

			//optimistic_ops_num
			loc_s = to_string(iterations.back().optimistic_sol_operations);
			hilf02c = hilf02c + loc_s + '\t';

			//sichtfeld setting
			switch (cmd_inputs.setting_sichtfeld) {
					case 1:
						loc_s = "uniform";
						break;
					case 2:
						loc_s = "midpoint";
						break;
					case 3:
						loc_s = "one_fourth";
						break;
					default:
						loc_s = "NONE";	
			}
			hilf02c = hilf02c + loc_s + '\t';

			//Optimistic Solution
			loc_s = write_number(iterations.back().optimistic_initial_solution,2);
			hilf02c = hilf02c + loc_s + '\t';


			//full_info
			if (cmd_inputs.setting_full_info > -1 && cmd_inputs.setting_full_info < 3) {

				header = header + "full_info_obj"+ '\t' + "time_full_info" + '\t'+ "full_info_truck_arcs" + '\t' + "full_info_solution_procedure" + '\t' + "full_info_truck_travel_duration"+ '\t' + "full_info_drone_travel_duration" +'\t' + "full_info_tandem_travel" + '\t'+ "full_info_drone_sortie_duration" + '\t'+"full_info_truck_wait" +'\t'+"full_info_drone_wait"+'\t'+"full_info_ops_num"+'\t'+"full_info_drone_truck_tandem_ops"+'\t';

				//full_info_obj
				hilf02c = hilf02c + write_number(iterations.back().full_info_obj, 3) + '\t';

				//full_info_time_lb
				hilf02c = hilf02c + convert_time_to_str(iterations.back().time_fullinfo, 6) + '\t';

				//List of truck arcs
				loc_s = "[";
				for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
					for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
						if (iterations.back().truck_arcs_fullinfo[loc_i][loc_j] > -1) {
							loc_s = loc_s + "(";
							loc_s = loc_s + to_string(loc_i);
							loc_s = loc_s + ", ";
							loc_s = loc_s + to_string(loc_j);
							loc_s = loc_s + "),";
						};
					};
				};
				if(loc_s.size()>1) {
					loc_s.pop_back();
				}
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';

				//method
				hilf02c = hilf02c + methods + '\t';

				//truck travel_duration
				hilf02c = hilf02c + write_number(iterations.back().truck_time_used.fullinfo, 3) + '\t';
				//drone travel_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_time_used_without_sorties.fullinfo, 3) + '\t';
				//truck drone tandem duration
				hilf02c = hilf02c + write_number(iterations.back().truck_drone_tandem_time_used.fullinfo, 3) + '\t';
				//drone sortie travel_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_sortie_time.fullinfo, 3) + '\t';
				//truck waiting time
				hilf02c = hilf02c + write_number(iterations.back().truck_wait.fullinfo, 3) + '\t';
				//drone waiting time
				hilf02c = hilf02c + write_number(iterations.back().drone_wait.fullinfo, 3) + '\t';
				//op nums
				loc_s = to_string(iterations.back().operations_num.fullinfo);
				hilf02c = hilf02c + loc_s + '\t';
				//drone truck tandem op nums
				loc_s = to_string(iterations.back().drone_on_truck_operations_num.fullinfo);
				hilf02c = hilf02c + loc_s + '\t';
			}
			////survfirst
			if (cmd_inputs.setting_surv_first > -1 && cmd_inputs.setting_surv_first < 3) {

				header = header + "survfirst_surveillance_sol" + '\t' + "survfirst_obj"  + '\t' + "total_cost_surveillance_first" + '\t' + "n_replan_surveillance_first"+ '\t'+ "list_of_surveillance_customers" + '\t' + "num_surv_customers" + '\t' + "time_survfirst_surveillance" + '\t'+"surv_truck_arcs"+ '\t' + "survfirst_solution_procedure" + '\t' + "survfirst_truck_travel_duration"+ '\t' + "survfirst_drone_travel_duration" +'\t' + "survfirst_tandem_travel" +'\t' + "survfirst_drone_sortie_duration" + '\t' + "survfirst_truck_wait"+ '\t' + "survfirst_drone_wait" + '\t'+ "survfirst_planned_truck_damages" + '\t'+"survfirst_ops_num"+'\t'+"survfirst_drone_truck_tandem_ops"+'\t'+"survfirst_truck_times_vec"+'\t'+"survfirst_drone_times_vec"+'\t'+"survfirst_truck_wait_vec"+'\t'+"survfirst_drone_wait_vec"+'\t'+"survfirst_drone_times_comb_vec"+'\t';


				//result surfirst
				hilf02c = hilf02c + write_number(iterations.back().survfirst_surveillance_tour_cost, 3) + '\t';

				//survfirst_obj
				hilf02c = hilf02c + write_number(iterations.back().survfirst_obj, 3) + '\t';

				//total_deliverycost_surveillance_first
				hilf02c = hilf02c + write_number(iterations.back().survfirst_surveillance_tour_cost + iterations.back().survfirst_obj, 3) + '\t';

				//number of replanning survfirst
				integer_part = iterations.back().n_replan_surveillance_first;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//list_of_surveillance_customers
				loc_int = 0;
				loc_s = "[";
				for (int loc_i : iterations.back().surveillance_nodes) {
						loc_s = loc_s + to_string(loc_i);
						loc_s = loc_s + ", ";
						loc_int = loc_int+1;
				};
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';
				//surv_customers_num
				loc_s = to_string(loc_int);
				hilf02c = hilf02c + loc_s + '\t';

				//time surveillance
				hilf02c = hilf02c + convert_time_to_str(iterations.back().time_surveillance, 6) + '\t';


				//List of truck arcs
				loc_s = "[";
				for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
					for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
						if (iterations.back().truck_arcs_surv[loc_i][loc_j] > -1) {
							loc_s = loc_s + "(";
							loc_s = loc_s + to_string(loc_i);
							loc_s = loc_s + ", ";
							loc_s = loc_s + to_string(loc_j);
							loc_s = loc_s + "),";
						};
					};
				};
				if(loc_s.size()>1) {
					loc_s.pop_back();
				}
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';


				//method
				hilf02c = hilf02c + methods + '\t';

				//truck travel_duration
				hilf02c = hilf02c + write_number(iterations.back().truck_time_used.surv, 3) + '\t';
				//drone travel_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_time_used_without_sorties.surv, 3) + '\t';
				//tandem travel duration
				hilf02c = hilf02c + write_number(iterations.back().truck_drone_tandem_time_used.surv, 3) + '\t';
				//drone sortie_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_sortie_time.surv, 3) + '\t';
				//truck waiting time
				hilf02c = hilf02c + write_number(iterations.back().truck_wait.surv, 3) + '\t';
				//drone waiting time
				hilf02c = hilf02c + write_number(iterations.back().drone_wait.surv, 3) + '\t';
				//planned truck damages
				hilf02c = hilf02c + to_string(iterations.back().damaged_edges_in_solution.surv)+ '\t';
				//op nums
				loc_s = to_string(iterations.back().operations_num.surv);
				hilf02c = hilf02c + loc_s + '\t';
				//drone truck tandem op nums
				loc_s = to_string(iterations.back().drone_on_truck_operations_num.surv);
				hilf02c = hilf02c + loc_s + '\t';

				//truck travel times vector
				loc_s = vecToString(iterations.back().truck_op_times.surv);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel times vector
				loc_s = vecToString(iterations.back().drone_op_times.surv);
				hilf02c = hilf02c + loc_s + '\t';
				//truck waiting times vector
				loc_s = vecToString(iterations.back().truck_op_wait_times.surv);
				hilf02c = hilf02c + loc_s + '\t';
				//drone waiting times vector
				loc_s = vecToString(iterations.back().drone_op_wait_times.surv);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel and waiting times vector
				loc_s = vecToString(iterations.back().drone_op_times_with_wait_time.surv);
				hilf02c = hilf02c + loc_s + '\t';
			}
			////survop
			if (cmd_inputs.setting_surv_op > -1 && cmd_inputs.setting_surv_op < 3) {

				header = header + "survop_surveillance_sol" + '\t' + "survop_obj"  + '\t' + "total_cost_survop" + '\t' + "n_replan_survop_survphase"+ '\t'+"dyn_cmax" + '\t'+ "list_of_surveillance_survop_customers" + '\t' + "num_survop_customers" +'\t' + "n_replan_survop_reoptphase"+ '\t' + "n_replan_survop_reopt_last_operation"+ '\t'+ "time_survop_surveillance" + '\t'+"survop_truck_arcs"+ '\t' + "survop_solution_procedure" + '\t' + "survop_truck_travel_duration"+ '\t' + "survop_drone_travel_duration" +'\t'+ "survop_tandem_travel" +'\t' + "survop_drone_sortie_duration" + '\t' + "survop_truck_wait"+ '\t' + "survop_drone_wait" + '\t'+ "survop_surv_planned_truck_damages" + '\t'+ "survop_deliv_planned_truck_damages" + '\t'+"survop_ops_num"+'\t'+"survop_drone_truck_tandem_ops"+'\t'+"survop_truck_times_vec"+'\t'+"survop_drone_times_vec"+'\t'+"survop_truck_wait_vec"+'\t'+"survop_drone_wait_vec"+'\t'+"survop_drone_times_comb_vec"+'\t';


				//result surfirst
				hilf02c = hilf02c + write_number(iterations.back().survop_surveillance_tour_cost, 3) + '\t';

				//survop_obj
				hilf02c = hilf02c + write_number(iterations.back().survop_obj, 3) + '\t';

				//total_deliverycost_surveillance_first
				hilf02c = hilf02c + write_number(iterations.back().survop_surveillance_tour_cost + iterations.back().survop_obj, 3) + '\t';

				//number of replanning survop
				integer_part = iterations.back().n_replan_surveillance_op_survphase;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//dyn_cmax
				hilf02c = hilf02c + write_number(iterations.back().dyn_cmax, 2) + '\t';

				//list_of_surveillance_customers
				loc_int = 0;
				loc_s = "[";
				for (int loc_i : iterations.back().surveillance_op_nodes) {
						loc_s = loc_s + to_string(loc_i);
						loc_s = loc_s + ", ";
						loc_int = loc_int+1;
				};
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';
				//surv_customers_num
				loc_s = to_string(loc_int);
				hilf02c = hilf02c + loc_s + '\t';

				//number of replanning survop reopt
				integer_part = surv_op_reopt_parameters.n_replan_reopt;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//number of replanning survop reopt last customer
				integer_part = iterations.back().n_replan_survop_reopt_last_operation;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//time surveillance
				hilf02c = hilf02c + convert_time_to_str(iterations.back().time_surveillance_op, 6) + '\t';


				//List of truck arcs
				loc_s = "[";
				for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
					for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
						if (iterations.back().truck_arcs_surv_op[loc_i][loc_j] > -1) {
							loc_s = loc_s + "(";
							loc_s = loc_s + to_string(loc_i);
							loc_s = loc_s + ", ";
							loc_s = loc_s + to_string(loc_j);
							loc_s = loc_s + "),";
						};
					};
				};
				if(loc_s.size()>1) {
					loc_s.pop_back();
				}
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';


				//method
				hilf02c = hilf02c + methods + '\t';

				//truck travel_duration
				hilf02c = hilf02c + write_number(iterations.back().truck_time_used.surv_op, 3) + '\t';
				//drone travel_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_time_used_without_sorties.surv_op, 3) + '\t';
				//truck drone tandem
				hilf02c = hilf02c + write_number(iterations.back().truck_drone_tandem_time_used.surv_op, 3) + '\t';
				//drone sortie_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_sortie_time.surv_op, 3) + '\t';
				//truck waiting time
				hilf02c = hilf02c + write_number(iterations.back().truck_wait.surv_op, 3) + '\t';
				//drone waiting time
				hilf02c = hilf02c + write_number(iterations.back().drone_wait.surv_op, 3) + '\t';
				//surveillance planned truck damages
				hilf02c = hilf02c + to_string(iterations.back().damaged_edges_in_solution.surv_op_surveillance)+ '\t';
				//delivery planned truck damages
				hilf02c = hilf02c + to_string(iterations.back().damaged_edges_in_solution.surv_op_delivery)+ '\t';
				//op nums
				loc_s = to_string(iterations.back().operations_num.surv_op_delivery);
				hilf02c = hilf02c + loc_s + '\t';
				//drone truck tandem op nums
				loc_s = to_string(iterations.back().drone_on_truck_operations_num.surv_op_delivery);
				hilf02c = hilf02c + loc_s + '\t';

				//truck travel times vector
				loc_s = vecToString(iterations.back().truck_op_times.surv_op);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel times vector
				loc_s = vecToString(iterations.back().drone_op_times.surv_op);
				hilf02c = hilf02c + loc_s + '\t';
				//truck waiting times vector
				loc_s = vecToString(iterations.back().truck_op_wait_times.surv_op);
				hilf02c = hilf02c + loc_s + '\t';
				//drone waiting times vector
				loc_s = vecToString(iterations.back().drone_op_wait_times.surv_op);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel and waiting times vector
				loc_s = vecToString(iterations.back().drone_op_times_with_wait_time.surv_op);
				hilf02c = hilf02c + loc_s + '\t';
			}
			//reopt
			if (cmd_inputs.setting_reopt > -1 && cmd_inputs.setting_reopt < 3) {
				header = header + "reopt_obj" + '\t' + "n_replan_reopt" +'\t' + "n_replan_last_op_last_delivery" +   '\t' + "time_reopt" + '\t' + "reopt_truck_arcs" +'\t' + "reopt_solution_procedure" + '\t' + "reopt_truck_travel_duration"+ '\t' + "reopt_drone_travel_duration" +'\t'+ "reopt_tandem_travel" +'\t' + "reopt_drone_sortie_duration" + '\t'+ "reopt_truck_wait"+ '\t' + "reopt_drone_wait" + '\t'+ "reopt_planned_truck_damages" + '\t'+"reopt_ops_num"+'\t'+"reopt_drone_truck_tandem_ops"+'\t'+"reopt_truck_times_vec"+'\t'+"reopt_drone_times_vec"+'\t'+"reopt_truck_wait_vec"+'\t'+"reopt_drone_wait_vec"+'\t'+"reopt_drone_times_comb_vec"+'\t';
				//delivery cost reopt obj
				hilf02c = hilf02c + write_number(iterations.back().reopt_obj, 3) + '\t';

				//n_replan_reopt
				integer_part = reopt_parameters.n_replan_reopt;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//n_replan_last_op_last_delivery
				integer_part = iterations.back().n_replan_reopt_last_operation;
				loc_s = to_string(integer_part);
				hilf02c = hilf02c + loc_s + '\t';

				//time reopt
				hilf02c = hilf02c + convert_time_to_str(iterations.back().time_reopt, 6) + '\t';

				//List of truck arcs
				loc_s = "[";
				for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
					for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
						if (iterations.back().truck_arcs_reopt[loc_i][loc_j] > -1) {
							loc_s = loc_s + "(";
							loc_s = loc_s + to_string(loc_i);
							loc_s = loc_s + ", ";
							loc_s = loc_s + to_string(loc_j);
							loc_s = loc_s + "),";
						};
					};
				};
				if(loc_s.size()>1) {
					loc_s.pop_back();
				}
				loc_s = loc_s + "]";
				hilf02c = hilf02c + loc_s + '\t';
				

				//method
				hilf02c = hilf02c + methods + '\t';

				//truck travel_duration
				hilf02c = hilf02c + write_number(iterations.back().truck_time_used.reopt, 3) + '\t';
				//drone travel_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_time_used_without_sorties.reopt, 3) + '\t';
				//tandem travel_duration
				hilf02c = hilf02c + write_number(iterations.back().truck_drone_tandem_time_used.reopt, 3) + '\t';
				//drone sortie_duration
				hilf02c = hilf02c + write_number(iterations.back().drone_sortie_time.reopt, 3) + '\t';
				//truck waiting time
				hilf02c = hilf02c + write_number(iterations.back().truck_wait.reopt, 3) + '\t';
				//drone waiting time
				hilf02c = hilf02c + write_number(iterations.back().drone_wait.reopt, 3) + '\t';
				//planned truck damages
				hilf02c = hilf02c + to_string(iterations.back().damaged_edges_in_solution.reopt)+ '\t';
				//op nums
				loc_s = to_string(iterations.back().operations_num.reopt);
				hilf02c = hilf02c + loc_s + '\t';
				//drone truck tandem op nums
				loc_s = to_string(iterations.back().drone_on_truck_operations_num.reopt);
				hilf02c = hilf02c + loc_s + '\t';

				//truck travel times vector
				loc_s = vecToString(iterations.back().truck_op_times.reopt);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel times vector
				loc_s = vecToString(iterations.back().drone_op_times.reopt);
				hilf02c = hilf02c + loc_s + '\t';
				//truck waiting times vector
				loc_s = vecToString(iterations.back().truck_op_wait_times.reopt);
				hilf02c = hilf02c + loc_s + '\t';
				//drone waiting times vector
				loc_s = vecToString(iterations.back().drone_op_wait_times.reopt);
				hilf02c = hilf02c + loc_s + '\t';
				//drone travel and waiting times vector
				loc_s = vecToString(iterations.back().drone_op_times_with_wait_time.reopt);
				hilf02c = hilf02c + loc_s + '\t';
			}
			//conservative
			if (cmd_inputs.setting_conservative > -1 && cmd_inputs.setting_conservative < 3) {
				header = header + "conservative_obj" + '\t' +  "time_conservative" + '\t'+"drone_op_times"+ '\t';
				//naiveub
				hilf02c = hilf02c + write_number(iterations.back().conservative_obj, 3) + '\t';
				hilf02c = hilf02c + convert_time_to_str(iterations.back().time_conservative, 6) + '\t';
				//drone travel times vector
				loc_s = vecToString(iterations.back().drone_op_times.conservative);
				hilf02c = hilf02c + loc_s + '\t';
			}

			//we just need first seed
			loc_s = to_string(iterations.back().ranseed);
			hilf02c = hilf02c + loc_s;
			header = header + "random_seeds" + '\t' + "iteration" + '\t' + "error_text" + '\t'+"time_stamp" + '\t' + "year" + '\t' + "month" + '\t' + "day"+ '\t' + "time";
			time_t now = time(0);
			tm* ltm = localtime(&now);
			hilf02c = hilf02c + '\t' + to_string(iterations.back().current_iteration) + '\t' + iterations.back().mistake_text + '\t' + to_string(time(0)) + '\t'+to_string(1900+ltm->tm_year) + '\t' + to_string(1+ltm->tm_mon) + '\t' + to_string(ltm->tm_mday) + '\t' + to_string(ltm->tm_hour)+":"+ to_string(ltm->tm_min)+":"+ to_string(ltm->tm_sec);

			if (new_file == true) {
				OFile << header << '\n';
			}

			OFile << hilf02c << '\n';
			//};
		}
		catch(const std::exception &e){
			if (new_file == true) {
				OFile << header << '\n';
			}
			OFile << hilf02c << e.what() << '\n';
		}	
	};	

	OFile.close();
};


void Simulation::ausgabe_tour(Solution curr_solution, string solution_ID, bool checked_and_status) {
	if(cmd_inputs.setting_debug){
		std::ofstream OFile;
		string locfilename = "";

		locfilename = locfilename + cmd_inputs.path_to_results;
		locfilename = locfilename + to_string(instance.ins_number)+"_";
		locfilename = locfilename + "d"+ write_number(instance.s_drone,2) + "_";
		locfilename = locfilename + "c" + write_number(cmd_inputs.input_required_nodes_proportion,3) + "_";
		locfilename = locfilename + "p" + write_number(cmd_inputs.input_parking_nodes,3) + "_";
		locfilename = locfilename + "k" + to_string(cmd_inputs.input_k_arc_damages) + "_";
		//locfilename = locfilename + "cd" + write_number(cmd_inputs.input_clustered_damages,1)+ "_";
		locfilename = locfilename + "s" + BoolToString(cmd_inputs.shortcuts)+ "_";
		locfilename = locfilename + "m" + BoolToString(cmd_inputs.truck_distance_metric_manhattan)+ "_";
		locfilename = locfilename + "pf" + write_number(cmd_inputs.penalty_factor,2) + "_";
		locfilename = locfilename + "st" + write_number(cmd_inputs.service_time,2) + "_";
		locfilename = locfilename + "cmr" + write_number(cmd_inputs.cmax_ratio,2) + "_";
		locfilename = locfilename + "r"+ to_string(cmd_inputs.ranseed) + "_";
		locfilename = locfilename + solution_ID;
		//if (cmd_inputs.input_filename.length() > 0) {
		//	locfilename = locfilename + cmd_inputs.input_filename;
		//}
		locfilename = locfilename + ".tspdsol";


		OFile.open(locfilename, std::ios_base::app);


		if (OFile.is_open()) {
			string hilf02c = "<Legend>";
			OFile << hilf02c << '\n';
			hilf02c = "Stage; Node1; Node2; Length_of_the_arc_for_the_vehicle";
			OFile << hilf02c << '\n';
			hilf02c = "<nodes_in_instance>";
			OFile << hilf02c << '\n';
			hilf02c = to_string(instance.n_nodes);
			OFile << hilf02c << '\n';
			hilf02c = "<customer_nodes>";
			OFile << hilf02c << '\n';
			hilf02c = "";
			for(int loc_node=0;loc_node< (int) instance.customers_to_visit.size(); loc_node++){
				if(instance.customers_to_visit[loc_node]==true) {
					hilf02c = hilf02c+to_string(loc_node)+"; ";
				}
			}
			OFile << hilf02c << '\n';
			hilf02c = "<drone_parking_nodes>";
			OFile << hilf02c << '\n';
			hilf02c = "";
			for(int loc_node: instance.parking_nodes){
				hilf02c = hilf02c+to_string(loc_node)+"; ";
			}
			OFile << hilf02c << '\n';
			hilf02c = "";
			OFile << hilf02c << '\n';

			//Output truck sorties
			hilf02c = "<Truck_sorties>";
			OFile << hilf02c << '\n';
			if ((int)curr_solution.truck_sorties.size() > 0) {
				for (int loc_i = 0; loc_i < curr_solution.n_stages; loc_i++) {
					int first_node = curr_solution.truck_sorties[loc_i][0];
					if ((int)curr_solution.truck_sorties[loc_i].size() > 1) {//it is a traveling stage
						for (int loc_j = 1; loc_j < (int)curr_solution.truck_sorties[loc_i].size(); loc_j++) {
							int second_node = curr_solution.truck_sorties[loc_i][loc_j];
							string loc_s = to_string(loc_i + 1);
							hilf02c = loc_s;
							hilf02c = hilf02c + "; ";

							loc_s = to_string(first_node);
							hilf02c = hilf02c + loc_s + "; ";

							loc_s = to_string(second_node);
							hilf02c = hilf02c + loc_s + "; ";
							loc_s = write_number(curr_solution.truck_distances[first_node][second_node],1);
							hilf02c = hilf02c + loc_s;
							OFile << hilf02c << '\n';
							first_node = second_node;
						};
					}
					else {//it is a waiting stage
						string loc_s = to_string(loc_i + 1);
						hilf02c = loc_s;
						hilf02c = hilf02c + "; ";

						loc_s = to_string(first_node);
						hilf02c = hilf02c + loc_s + "; ";

						loc_s = to_string(first_node);
						hilf02c = hilf02c + loc_s + "; ";

						loc_s = to_string(0);
						hilf02c = hilf02c + loc_s;
						OFile << hilf02c << '\n';
					};
				};//for all stages
			}
			hilf02c = "";
			OFile << hilf02c << '\n';

			//Output Drone sorties
			hilf02c = "<Drone_sorties>";
			OFile << hilf02c << '\n';
			if ((int)curr_solution.drone_sorties.size() > 0) {
				for (int loc_i = 0; loc_i < curr_solution.n_stages; loc_i++) {
					int first_node = curr_solution.drone_sorties[loc_i][0];
					if ((int)curr_solution.drone_sorties[loc_i].size() > 1) {//it is a drone op stage
						for (int loc_j = 1; loc_j < (int)curr_solution.drone_sorties[loc_i].size(); loc_j++) {
							int second_node = curr_solution.drone_sorties[loc_i][loc_j];
							string loc_s = to_string(loc_i + 1);
							hilf02c = loc_s;
							hilf02c = hilf02c + "; ";

							loc_s = to_string(first_node);
							hilf02c = hilf02c + loc_s + "; ";

							loc_s = to_string(second_node);
							hilf02c = hilf02c + loc_s + "; ";
							loc_s = write_number(curr_solution.drone_distances[first_node][second_node], 1);
							hilf02c = hilf02c + loc_s;
							OFile << hilf02c << '\n';
							first_node = second_node;
						};
					}
				};//for all stages
			}

			hilf02c = "";
			OFile << hilf02c << '\n';

			hilf02c = "<is_the_solution_correct>";
			OFile << hilf02c << '\n';
			if (checked_and_status == true) {
				hilf02c = "false";
				OFile << hilf02c << '\n';
			}
			else {
				hilf02c = "true";
				OFile << hilf02c << '\n';
			};

			hilf02c = "";
			OFile << hilf02c << '\n';

			hilf02c = "<end_file>";
		};//if file is open

		OFile.close();
	}	
};

void Simulation::check_solution_tsp(Solution input_solution, bool& result) {//only possible for the LONG solution
	double loc_obj_value = 0;
	bool there_is_mistake = false;
	//check whether all the customers are delivered
	vector<bool>delivered_customers;
	delivered_customers = vector<bool>(instance.n_nodes);
	for (int loc_i = 0; (loc_i < instance.n_nodes); loc_i++) {
		delivered_customers[loc_i] = false;
	};


	if (input_solution.drone_sorties.size() > 0) {

		for (int loc_i = 0; loc_i < input_solution.n_stages; loc_i++) {
			double drone_sortie_duration = 0.0;
			int loc_current_node = input_solution.drone_sorties[loc_i][1];
			for (int loc_j = 1; loc_j < input_solution.drone_sorties[loc_i][0]; loc_j++) {
				int drone_delivery_node = input_solution.drone_sorties[loc_i][loc_j + 1];
				drone_sortie_duration = drone_sortie_duration + instance.drone_distances[loc_current_node][drone_delivery_node];
				delivered_customers[drone_delivery_node] = true;
				loc_current_node = drone_delivery_node;
			};

			//stage duration
			loc_obj_value = loc_obj_value + drone_sortie_duration;

			if ((drone_sortie_duration > 0) && ((drone_sortie_duration > instance.max_drone_sortie))) {
				cout << "MISTAKEsimul: drone sortie exceed the max sortie duration of";
				cout << (instance.max_drone_sortie) << '\n';
				iterations.back().mistake_reported = true;
				string loc_s = to_string(loc_i);
				iterations.back().mistake_text = iterations.back().mistake_text + "@tsp_model simul: drone sortie at ";
				iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
				iterations.back().mistake_text = iterations.back().mistake_text + " exceeds max sortie duration of ";
				iterations.back().mistake_text = iterations.back().mistake_text + to_string(instance.max_drone_sortie);
				iterations.back().mistake_text = iterations.back().mistake_text + "; ";
			};
		};
	}
	else {
		iterations.back().mistake_text = iterations.back().mistake_text + "no drone sorties were used in tsp model ";
	};
	//check, whether all customers have been delivered
	for (int loc_i = 1; loc_i < (instance.n_nodes); loc_i++) {
		if (delivered_customers[loc_i] == false) {
			cout << "MISTAKE: the following customer was not delivered ";
			cout << (loc_i) << '\n';
			there_is_mistake = true;
			iterations.back().mistake_reported = true;
			string loc_s = std::to_string(loc_i);
			iterations.back().mistake_text = iterations.back().mistake_text + "@check_sol: customer ";
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
			iterations.back().mistake_text = iterations.back().mistake_text + " not delivered; ";
		};
	};
	if ((loc_obj_value + 0.5 < input_solution.obj_value) || (loc_obj_value - 0.5 > input_solution.obj_value)) {
		cout << "objective value is false. The true one; the one saved in the solution structure ";
		cout << loc_obj_value;
		cout << "; ";
		there_is_mistake = true;
		cout << input_solution.obj_value << '\n';
		iterations.back().mistake_reported = true;
		iterations.back().mistake_text = iterations.back().mistake_text + "@check_sol: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + ", from heuristic ";
		loc_s = std::to_string(input_solution.obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + "; ";
	};


	result = there_is_mistake;
};

void Simulation::check_solution_surveillance(Solution input_solution, bool& result) {
	double loc_obj_value = 0;
	bool there_is_mistake = false;

	int last_previous_drone_node = 0;

	for (int loc_i = 0; loc_i < input_solution.n_stages; loc_i++) {
		double drone_sortie_duration = 0.0;
		int current_node = input_solution.drone_sorties[loc_i][0];
		if (last_previous_drone_node != current_node) {
			cout << "MISTAKE_surv: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			iterations.back().mistake_reported = true;
			there_is_mistake = true;
			iterations.back().mistake_text = iterations.back().mistake_text + "@check_solsurv: drone route is jumpy at the beginning of stage; ";
		};


		if ((int)input_solution.drone_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_drone_previous_node = current_node;
			for (int loc_j = 1; loc_j < (int)input_solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_delivery_node = input_solution.drone_sorties[loc_i][loc_j];
				drone_sortie_duration = drone_sortie_duration + instance.drone_distances[loc_drone_previous_node][drone_delivery_node];

				loc_drone_previous_node = drone_delivery_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
		};

		//stage duration
		loc_obj_value = loc_obj_value + drone_sortie_duration;

	};

	//check
	if ((loc_obj_value + 0.5 < input_solution.obj_value) || (loc_obj_value - 0.5 > input_solution.obj_value)) {
		cout << "objective value is false. The true one; the one saved in the solution structure ";
		cout << loc_obj_value;
		cout << "; ";
		there_is_mistake = true;
		cout << input_solution.obj_value << '\n';
		iterations.back().mistake_reported = true;
		iterations.back().mistake_text = iterations.back().mistake_text + "@check_solsurv: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + ", from heuristic ";
		loc_s = std::to_string(input_solution.obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + "; ";
	};


	result = there_is_mistake;
};

void Simulation::check_solution_surveillance_op(Solution input_solution, bool& result) {
	double loc_obj_value = 0;
	bool there_is_mistake = false;

	int last_previous_drone_node = 0;

	for (int loc_i = 0; loc_i < input_solution.n_stages; loc_i++) {
		double drone_sortie_duration = 0.0;
		int current_node = input_solution.drone_sorties[loc_i][0];
		if (last_previous_drone_node != current_node) {
			cout << "MISTAKE_surv: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			iterations.back().mistake_reported = true;
			there_is_mistake = true;
			iterations.back().mistake_text = iterations.back().mistake_text + "@check_solsurv: drone route is jumpy at the beginning of stage; ";
		};


		if ((int)input_solution.drone_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_drone_previous_node = current_node;
			for (int loc_j = 1; loc_j < (int)input_solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_delivery_node = input_solution.drone_sorties[loc_i][loc_j];
				drone_sortie_duration = drone_sortie_duration + instance.drone_distances[loc_drone_previous_node][drone_delivery_node];

				loc_drone_previous_node = drone_delivery_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
		};

		//stage duration
		loc_obj_value = loc_obj_value + drone_sortie_duration;

	};

	//check
	if ((loc_obj_value + 0.5 < input_solution.obj_value) || (loc_obj_value - 0.5 > input_solution.obj_value)) {
		cout << "objective value is false. The true one; the one saved in the solution structure ";
		cout << loc_obj_value;
		cout << "; ";
		there_is_mistake = true;
		cout << input_solution.obj_value << '\n';
		iterations.back().mistake_reported = true;
		iterations.back().mistake_text = iterations.back().mistake_text + "@check_solsurv: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + ", from heuristic ";
		loc_s = std::to_string(input_solution.obj_value);
		iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
		iterations.back().mistake_text = iterations.back().mistake_text + "; ";
	};

	//check max tour duration constraint
	if(cmd_inputs.cmax>0.0) {
		if (loc_obj_value > cmd_inputs.cmax) {
			cout << "error in the surveillance op solution, the tour exceeds cmax!";
			cout << loc_obj_value;
			cout << "; ";
			there_is_mistake = true;
			cout << input_solution.obj_value << '\n';
			iterations.back().mistake_reported = true;
			iterations.back().mistake_text = iterations.back().mistake_text + "@check_survop_surveillance: solution violates cmax constraint, the tour exceeds cmax ";
			string loc_s = std::to_string(cmd_inputs.cmax);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s+"!";
			loc_s = std::to_string(loc_obj_value);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
			iterations.back().mistake_text = iterations.back().mistake_text + ", from model ";
			loc_s = std::to_string(input_solution.obj_value);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
			iterations.back().mistake_text = iterations.back().mistake_text + "; ";
		}
	}
	else{
		if (loc_obj_value > ceil(iterations.back().dyn_cmax)) {
			cout << "error in the surveillance op solution, the tour exceeds cmax!";
			cout << loc_obj_value;
			cout << "; ";
			there_is_mistake = true;
			cout << input_solution.obj_value << '\n';
			iterations.back().mistake_reported = true;
			iterations.back().mistake_text = iterations.back().mistake_text + "@check_survop_surveillance: solution violates cmax constraint, the tour exceeds cmax ";
			string loc_s = std::to_string(cmd_inputs.cmax);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s+"!";
			loc_s = std::to_string(loc_obj_value);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
			iterations.back().mistake_text = iterations.back().mistake_text + ", from model ";
			loc_s = std::to_string(input_solution.obj_value);
			iterations.back().mistake_text = iterations.back().mistake_text + loc_s;
			iterations.back().mistake_text = iterations.back().mistake_text + "; ";
		}
	}
	
	result = there_is_mistake;
};


void Simulation::ausgabe_tour_surveillance(Solution curr_solution, string solution_ID, bool checked_and_status) {
	if(cmd_inputs.setting_debug){
		std::ofstream OFile;
		string locfilename = "";

		locfilename = locfilename + cmd_inputs.path_to_results;
		locfilename = locfilename + to_string(instance.ins_number)+"_";
		locfilename = locfilename + "d"+ write_number(instance.s_drone,1) + "_";
		locfilename = locfilename + "c" + write_number(cmd_inputs.input_required_nodes_proportion,3) + "_";
		locfilename = locfilename + "p" + write_number(cmd_inputs.input_parking_nodes,3) + "_";
		locfilename = locfilename + "k" + to_string(cmd_inputs.input_k_arc_damages) + "_";
		locfilename = locfilename + "cd" + write_number(cmd_inputs.input_clustered_damages,1)+ "_";
		locfilename = locfilename + "s" + BoolToString(cmd_inputs.shortcuts)+ "_";
		locfilename = locfilename + "m" + BoolToString(cmd_inputs.truck_distance_metric_manhattan)+ "_";
		locfilename = locfilename + "r"+ to_string(cmd_inputs.ranseed) + "_";
		locfilename = locfilename + solution_ID;
		locfilename = locfilename + ".tspdsol";


		OFile.open(locfilename, std::ios_base::app);


		if (OFile.is_open()) {
			string hilf02c = "<Legend>";
			OFile << hilf02c << '\n';
			hilf02c = "Stage; Node1; Node2; Length_of_the_arc_for_the_vehicle";
			OFile << hilf02c << '\n';
			hilf02c = "<nodes_in_instance>";
			OFile << hilf02c << '\n';
			hilf02c = to_string(instance.n_nodes);
			OFile << hilf02c << '\n';
			hilf02c = "<customer_nodes>";
			OFile << hilf02c << '\n';
			hilf02c = "";
			for(int loc_node=0;loc_node< (int) instance.customers_to_visit.size(); loc_node++){
				if(instance.customers_to_visit[loc_node]==true) {
					hilf02c = hilf02c+to_string(loc_node)+"; ";
				}
			}
			OFile << hilf02c << '\n';
			hilf02c = "<drone_parking_nodes>";
			OFile << hilf02c << '\n';
			hilf02c = "";
			for(int loc_node: instance.parking_nodes){
				hilf02c = hilf02c+to_string(loc_node)+"; ";
			}
			OFile << hilf02c << '\n';
			hilf02c = "";
			OFile << hilf02c << '\n';

			//Output truck sorties
			hilf02c = "<Truck_sorties>";
			OFile << hilf02c << '\n';
			hilf02c = "";
			OFile << hilf02c << '\n';



			//Output Drone sorties
			hilf02c = "<Drone_sorties>";
			OFile << hilf02c << '\n';
			for (int loc_i = 0; loc_i < curr_solution.n_stages; loc_i++) {
				int first_node = curr_solution.drone_sorties[loc_i][0];

				for (int loc_j = 1; loc_j < (int)curr_solution.drone_sorties[loc_i].size(); loc_j++) {
					int drone_node = curr_solution.drone_sorties[loc_i][loc_j];

					vector<int>help_vector = vector<int>(instance.n_nodes + 1);
					int n_entries_help_vector = 1;
					help_vector[0] = drone_node;
					int curr_node = drone_node;

					while (first_node != instance.drone_fromtriple_previousnode[first_node][curr_node]) {
						curr_node = instance.drone_fromtriple_previousnode[first_node][curr_node];
						help_vector[n_entries_help_vector] = curr_node;
						n_entries_help_vector = n_entries_help_vector + 1;
					};
					help_vector[n_entries_help_vector] = first_node;
					n_entries_help_vector = n_entries_help_vector + 1;

					first_node = help_vector[n_entries_help_vector - 1];
					for (int loc_l = 1; loc_l < n_entries_help_vector; loc_l++) {
						int second_node = help_vector[n_entries_help_vector - 1 - loc_l];
						if(first_node!=second_node) {
							string loc_s = to_string(loc_i + 1);
							hilf02c = loc_s;
							hilf02c = hilf02c + "; ";

							loc_s = to_string(first_node);
							hilf02c = hilf02c + loc_s + "; ";

							loc_s = to_string(second_node);
							hilf02c = hilf02c + loc_s + "; ";

							int integer_part = instance.drone_distances[first_node][second_node];
							loc_s = to_string(integer_part);
							loc_s = loc_s + ".";
							int ostatok = 1000 * instance.drone_distances[first_node][second_node];
							ostatok = (ostatok % 1000) * 1000;
							int loc_int = ostatok;
							loc_s = loc_s + to_string(loc_int);
							hilf02c = hilf02c + loc_s;
							OFile << hilf02c << '\n';
							first_node = second_node;
						}
					};

					first_node = drone_node;
				};//for each stage


				hilf02c = "";
				OFile << hilf02c << '\n';

				hilf02c = "<is_the_solution_correct>";
				OFile << hilf02c << '\n';
				if (checked_and_status == true) {
					hilf02c = "false";
					OFile << hilf02c << '\n';
				}
				else {
					hilf02c = "true";
					OFile << hilf02c << '\n';
				};

				hilf02c = "";
				OFile << hilf02c << '\n';

				hilf02c = "<end_file>";
			};//if file is open

			OFile.close();
		};//if file is open
	}	
}

void Simulation::update_planned_truck_arcs(int casecode, vector<vector<int>> truck_ledges)
{
	int node1 = 0;
	int node2 = 0;
	for(int i=0;i<(int)truck_ledges.size();i++) {
		for(int j=0;j<(int)truck_ledges[i].size()-1;j++) {
			node1 = truck_ledges[i][j];
			node2 = truck_ledges[i][j+1];
			if(node1<instance.n_nodes && node2<instance.n_nodes){
				switch ( casecode )
					{
						//fullinfo
						case 1:
							iterations.back().truck_arcs_fullinfo[node1][node2]= 1;
							iterations.back().truck_arcs_fullinfo[node2][node1]= 1;
							break;
						//surv
						case 2:
							iterations.back().truck_arcs_surv[node1][node2]= 1;
							iterations.back().truck_arcs_surv[node2][node1]= 1;
							break;
						//reopt
						case 3:
							iterations.back().truck_arcs_reopt[node1][node2]= 1;
							iterations.back().truck_arcs_reopt[node2][node1]= 1;
							break;
						case 4:
							iterations.back().truck_arcs_surv_op[node1][node2]= 1;
							iterations.back().truck_arcs_surv_op[node2][node1]= 1;
							break;
					}
			}
			
		}
	}
}
// end procedure