#include "Model_DP.h"


//default _penalty=0
Model_DP::Model_DP(Instance& _instance, vector<vector<int>>_edge_status, vector<vector<int>>_arc_status_after_surveillance, int _curr_position, vector<bool>_customers_to_visit, bool _manhattan_truck, double _service_time, double _penalty_factor) :
	Model(_instance, _edge_status, _arc_status_after_surveillance, _customers_to_visit, _manhattan_truck, _service_time),curr_position(_curr_position), number_of_nodes(_instance.n_nodes), d_table_operations_graph(vector<vector<double>>(_instance.n_nodes, vector<double>(pow(2, customer_n_nodes), max_lim))), prev_d_operations_graph(vector<vector<tuple<int, int, int>>>(_instance.n_nodes, vector<tuple<int, int, int>>(pow(2, customer_n_nodes), { -1 , -1 , -1}))), reopt_solution(Solution(0, _instance.n_nodes)),penalty_factor(_penalty_factor), reopt_parameters(reoptParams())
{
	init_dp_struct(penalty_factor);
}

void Model_DP::solve(int setting /*=1*/) {
	//DP Model_DP
	//setting 1: completion time objective
	//setting 2: last delivered customer

	if (conservative) {
		add_conservative_Loops();
		conservative_calculation(setting);
	}
	else {
		solve_operations(setting);
		make_operations_graph(setting);
		if(setting==1){
			write_solution();
		}
		else{
			write_solution_last_delivery();
		} 
		
	}
	putsolutionOnScreenDP();
	read_solution_complete_dp();
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	model_solution.service_time_map = dp.service_time_map;
	model_solution.art_node_map = dp.artificial_nodes_map;
	reopt_solution.truck_distances = dp.distance_metric_naive;
	reopt_solution.drone_distances = dp.reopt_drone_distances;
	reopt_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	reopt_solution.service_time_map = dp.service_time_map;
	reopt_solution.art_node_map = dp.artificial_nodes_map;
}


void Model_DP::solve_reopt(vector<vector<int>> _edge_status, vector<vector<int>>_arc_status_after_surveillance, vector<bool> _customers_to_visit, reoptParams _reopt_parameters, int simulation_setting)
{
	reopt_parameters = _reopt_parameters;
	arc_status_after_surveillance = _arc_status_after_surveillance;
	if (reopt_parameters.curr_truck_node != reopt_parameters.curr_drone_node) {
		reset_model_dp_reopt(_edge_status, _customers_to_visit, reopt_parameters.curr_truck_node, reopt_parameters.curr_drone_node,  reopt_parameters.curr_drone_cust, reopt_parameters.last_combined_node);
		init_dp_struct(penalty_factor,true);
	}
	else {
		reset_model(_edge_status, _customers_to_visit, reopt_parameters.curr_truck_node, reopt_parameters.curr_drone_node, reopt_parameters.last_combined_node);
		init_dp_struct(penalty_factor,false);
		/*
		if(dp.real_artificial_edge_distances.size()>0 && dp.real_artificial_edge_distances.size()> dp.artificial_edge_distances.size()){
			dp.real_artificial_edge_distances.pop_back();
			dp.real_artificial_edges.pop_back();
			//dp.artificial_edge_distances.pop_back();
			//dp.artificial_truck_node_bool.pop_back();
			//dp.artificial_edges.pop_back();
		}
		*/
	}
	
	init_reopt_model_struct();
	int new_size = dp.n_nodes + dp.artificial_nodes;

	solve_operations(simulation_setting);
	if (curr_drone_node != curr_truck_node) {
		curr_position = dp.n_nodes + dp.artificial_nodes - 1;
	}
	else {
		curr_position = curr_truck_node;
	}
	this->d_table_operations_graph = vector<vector<double>>(new_size, vector<double>(pow(2, customer_n_nodes), max_lim));
	this->prev_d_operations_graph = vector<vector<tuple<int, int, int>>>(new_size, vector<tuple<int, int, int>>(pow(2, customer_n_nodes), { -1 , -1 , -1 }));
	this->Ledges_struct = Ledges(new_size);
	make_operations_graph(simulation_setting);
	add_real_incoming_edges_after_solving();
	if(simulation_setting==1){
		write_solution_dp_reopt();
	}
	else{
		write_solution_dp_reopt_last_delivery();
	} 
	putsolutionOnScreenDP();
	read_solution_complete_dp();
	//adjust_artificial_edges();
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	model_solution.service_time_map = dp.service_time_map;
	model_solution.art_node_map = dp.artificial_nodes_map;
	reopt_solution.truck_distances = dp.distance_metric_naive;
	reopt_solution.drone_distances = dp.reopt_drone_distances;
	reopt_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	reopt_solution.service_time_map = dp.service_time_map;
	reopt_solution.art_node_map = dp.artificial_nodes_map;
}

void Model_DP::dp_operations_graph(int v) {
	d_table_operations_graph[v][0] = 0.0;
	for (int i = 0; i < (int)dp.subsets.size(); i++) {
		for (auto vec_U : dp.subsets[i]) {
			std::vector<int> set_T_big;
			std::set_difference(std::begin(customer_set), std::end(customer_set), // the first vector...
				std::begin(vec_U.first), std::end(vec_U.first), // ...minus the second...
				std::back_inserter(set_T_big));     // ...is stored into here
			std::vector<vector<int>> all_T_subsets = get_all_subsets(set_T_big);
			for (vector<int> vec_T : all_T_subsets) {
				for (int u = 0; u < (int)dp.dtsp_op.size(); u++) {
					for (int w = 0; w < (int)dp.dtsp_op.size(); w++) {
						std::set<int> set_T(std::make_move_iterator(vec_T.begin()), std::make_move_iterator(vec_T.end()));
						set<int> set_T_w = set_T;
						std::set<int> set_U_T(std::make_move_iterator(vec_U.first.begin()), std::make_move_iterator(vec_U.first.end()));
						//if (std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end() && std::find(vec_U.first.begin(), vec_U.first.end(), w) == vec_U.first.end()) {
						//	set_T_w.insert(w);
						//	set_U_T.insert(w);
						//}
						set_U_T.insert(set_T.begin(), set_T.end());
						//if (std::find(customer_set.begin(), customer_set.end(), u) != customer_set.end()) {
						//	set_U_T.insert(u);
							//set_T_w.insert(u); //didn't i remove it because of errors?
						//}
						std::vector<int> vec_U_T(set_U_T.begin(), set_U_T.end());
						std::vector<int> vec_T_w(set_T_w.begin(), set_T_w.end());
						int index0 = dp.subsets[vec_U.first.size()].at(vec_U.first);
						int index1 = dp.subsets[vec_T_w.size()].at(vec_T_w);
						int index2 = dp.subsets[vec_U_T.size()].at(vec_U_T);
						//check if index0 + index1 == index2
						if (d_table_operations_graph[u][index0] < max_lim && dp.dtsp_op[u][w][index1] < max_lim) {
							double z = d_table_operations_graph[u][index0] + dp.dtsp_op[u][w][index1];
							if (z < d_table_operations_graph[w][index2]) {
								d_table_operations_graph[w][index2] = z;
								prev_d_operations_graph[w][index2] = { u,index0,index1 };
							}
							//We don't want to improve the algorithm for this project
							/*
							if(index0>0 && index1>0) {
								if (z <= d_table_operations_graph[w][index2]) {
									d_table_operations_graph[w][index2] = z;
									prev_d_operations_graph[w][index2] = { u,index0,index1 };
								}
							}
							*/
						}
					}
				}
			}
		}
	}
}

void Model_DP::dp_operations_graph_last_delivery(int v) {
	d_table_operations_graph[v][0] = 0.0;
	for (int i = 0; i < (int)dp.subsets.size(); i++) {
		for (auto vec_U : dp.subsets[i]) {
			std::vector<int> set_T_big;
			std::set_difference(std::begin(customer_set), std::end(customer_set), // the first vector...
				std::begin(vec_U.first), std::end(vec_U.first), // ...minus the second...
				std::back_inserter(set_T_big));     // ...is stored into here
			std::vector<vector<int>> all_T_subsets = get_all_subsets(set_T_big);
			for (vector<int> vec_T : all_T_subsets) {
				for (int u = 0; u < (int)dp.dtsp_op.size(); u++) {
					for (int w = 0; w < (int)dp.dtsp_op.size(); w++) {
						std::set<int> set_T(std::make_move_iterator(vec_T.begin()), std::make_move_iterator(vec_T.end()));
						set<int> set_T_w = set_T;
						std::set<int> set_U_T(std::make_move_iterator(vec_U.first.begin()), std::make_move_iterator(vec_U.first.end()));
						//check if customer node and not in U
						//if (std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end() && std::find(vec_U.first.begin(), vec_U.first.end(), w) == vec_U.first.end()) {
						//	set_T_w.insert(w);
						//	set_U_T.insert(w);
						//}
						set_U_T.insert(set_T.begin(), set_T.end());
						//if (std::find(customer_set.begin(), customer_set.end(), u) != customer_set.end()) {
						//	set_U_T.insert(u);
							//set_T_w.insert(u);
						//}
						std::vector<int> vec_U_T(set_U_T.begin(), set_U_T.end());
						std::vector<int> vec_T_w(set_T_w.begin(), set_T_w.end());
						int index0 = dp.subsets[vec_U.first.size()].at(vec_U.first);
						int index1 = dp.subsets[vec_T_w.size()].at(vec_T_w);
						int index2 = dp.subsets[vec_U_T.size()].at(vec_U_T);
						if (d_table_operations_graph[u][index0] < max_lim) {
							if(index2<(int)d_table_operations_graph[v].size() - 1 && dp.dtsp_op[u][w][index1] < max_lim){ 
								double z = d_table_operations_graph[u][index0] + dp.dtsp_op[u][w][index1];
								if (z < d_table_operations_graph[w][index2]) {
									d_table_operations_graph[w][index2] = z;
									prev_d_operations_graph[w][index2] = { u,index0,index1 };
								}
							}
							if(index2== (int)d_table_operations_graph[v].size() - 1 && dp.dtsp_op_last_delivery[u][w][index1] < max_lim){
								double z = d_table_operations_graph[u][index0] + dp.dtsp_op_last_delivery[u][w][index1];
								if (z < d_table_operations_graph[w][index2]) {
									d_table_operations_graph[w][index2] = z;
									prev_d_operations_graph[w][index2] = { u,index0,index1 };
								}
							} 
						}
					}
				}
			}
		}
	}
}


void Model_DP::make_operations_graph(int setting)
//setting 1: completion time, setting 2: last delivered customer
{
	
	switch(setting){
		case 1:
			dp_operations_graph(curr_position);
			backtrack_ledges_dp();
			model_solution.obj_value = d_table_operations_graph[0][d_table_operations_graph[0].size() - 1];
			break;
		case 2:
			dp_operations_graph_last_delivery(curr_position);
			model_solution.obj_value = get_last_delivery_objective();
			backtrack_ledges_dp(model_solution.last_delivered_customer_obj2, setting);
			break;
	} 	
}

void Model_DP::backtrack_op_dp2(int u, int w, vector<int> curr_s) {
	vector<int> op(0);
	std::set<int> new_op(std::make_move_iterator(curr_s.begin()), std::make_move_iterator(curr_s.end()));
	int ww = w;
	int new_i = curr_s.size();
	int idx = dp.subsets[new_i].at(curr_s);
	int idx2 = idx;
	int drone_node = dp.dtsp_drone[u][w][idx];
	int num_erase = 0;
	int start_node = u;
	Ledges_struct.Ledges_dronenode.push_back(drone_node);
	Ledges_struct.Ledges_customers.push_back(dp.subsets_by_id.at(idx));
	if (u >= instance.n_nodes && curr_truck_node<instance.n_nodes) {
		start_node = curr_truck_node;
	}
	if (drone_node > -1) {
		if (drone_node != w) {
			num_erase = new_op.erase(drone_node);
		}
		Ledges_struct.nodes_on_Ledge[drone_node].push_back(Ledges_struct.n_Ledges);
		std::vector<int> setPredD(new_op.begin(), new_op.end());
		new_i = new_i - num_erase;
		idx2 = dp.subsets[new_i].at(setPredD);
	}
	int idx1 = dp.dtsp_pred[start_node][ww][idx2].second;
	int pred = dp.dtsp_pred[start_node][ww][idx2].first;
	while (pred > -1) {
		op.push_back(ww);
		if (ww != w && ww != start_node) {
			Ledges_struct.nodes_on_Ledge[ww].push_back(Ledges_struct.n_Ledges);
		}
		ww = pred;
		pred = dp.dtsp_pred[start_node][ww][idx1].first;
		//cout << "{" << dp.dtsp_pred[start_node][ww][idx1].first << " " << dp.dtsp_pred[start_node][ww][idx1].second << "}\n";
		idx2 = idx1;
		idx1 = dp.dtsp_pred[start_node][ww][idx2].second;
	}
	op.push_back(start_node);
	if (u >= instance.n_nodes) {
		Ledges_struct.reopt_ct_ledges.push_back(Ledges_struct.n_Ledges);
		if (curr_truck_node >= instance.n_nodes) {
			op.push_back(u);
		}
		op.push_back(0);
	}
	//remove consecutive duplicated nodes
	vector<int>::iterator it;
	it = unique(op.begin(), op.end());
	op.erase(it, op.end());
	Ledges_struct.incoming_Ledges[w].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.outgoing_Ledges[u].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.list_of_Ledges.push_back(dp.dtsp_op[u][w][idx]);
	Ledges_struct.Ledges_from_end_to_start.push_back(op);
	Ledges_struct.add_Ledge();
};

void Model_DP::backtrack_last_delivery_op_dp2(int u, int w, vector<int> curr_s) {
	vector<int> op(0);
	std::set<int> new_op(std::make_move_iterator(curr_s.begin()), std::make_move_iterator(curr_s.end()));
	int new_i = curr_s.size();
	int idx = dp.subsets[new_i].at(curr_s);
	int idx2 = idx;
	int ww = dp.dtsp_last_delivery_nodes[u][w][idx].first;
	model_solution.last_delivered_customers_last_delivery_op = dp.dtsp_last_delivery_nodes[u][w][idx];
	reopt_solution.last_delivered_customers_last_delivery_op = dp.dtsp_last_delivery_nodes[u][w][idx];
	Ledges_struct.Ledges_customers.push_back(dp.subsets_by_id.at(idx));
	if(ww<0){
		ww = w;
	} 
	int drone_node = dp.dtsp_last_delivery_nodes[u][w][idx].second;
	int num_erase = 0;
	int start_node = u;
	Ledges_struct.Ledges_dronenode.push_back(drone_node);
	if (u >= instance.n_nodes && curr_truck_node<instance.n_nodes) {
		start_node = curr_truck_node;
	}
	if (drone_node > -1) {
		if (drone_node != ww) {
			num_erase = new_op.erase(drone_node);
		}
		Ledges_struct.nodes_on_Ledge[drone_node].push_back(Ledges_struct.n_Ledges);
		std::vector<int> setPredD(new_op.begin(), new_op.end());
		new_i = new_i - num_erase;
		idx2 = dp.subsets[new_i].at(setPredD);
	}
	int idx1 = dp.dtsp_pred[start_node][ww][idx2].second;
	int pred = dp.dtsp_pred[start_node][ww][idx2].first;
	while (pred > -1) {
		op.push_back(ww);
		if (ww != w && ww != start_node) {
			Ledges_struct.nodes_on_Ledge[ww].push_back(Ledges_struct.n_Ledges);
		}
		ww = pred;
		pred = dp.dtsp_pred[start_node][ww][idx1].first;
		//cout << "{" << dp.dtsp_pred[start_node][ww][idx1].first << " " << dp.dtsp_pred[start_node][ww][idx1].second << "}\n";
		idx2 = idx1;
		idx1 = dp.dtsp_pred[start_node][ww][idx2].second;
	}
	op.push_back(start_node);
	if (u >= instance.n_nodes) {
		Ledges_struct.reopt_ct_ledges.push_back(Ledges_struct.n_Ledges);
		if (curr_truck_node >= instance.n_nodes) {
			op.push_back(u);
		}
		op.push_back(0);
	}
	//remove consecutive duplicated nodes
	vector<int>::iterator it;
	it = unique(op.begin(), op.end());
	op.erase(it, op.end());
	Ledges_struct.incoming_Ledges[u].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.outgoing_Ledges[u].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.list_of_Ledges.push_back(dp.dtsp_op[u][w][idx]);
	Ledges_struct.Ledges_from_end_to_start.push_back(op);
	Ledges_struct.add_Ledge();
};

void Model_DP::backtrack_ledges_dp(int last_node, int simulation_setting) {
	std::vector<int> current_vec = customer_set;
	int i = customer_set.size();
	int curr_S = dp.subsets[i].at(customer_set);
	vector<int> help_vec(0);
	std::tuple<int, int, int> pred_op = { 0,0,0 };
	pred_op = prev_d_operations_graph[last_node][curr_S];
	int loc_w = last_node;
	std::tuple<int, int, int> terminal_prev = { -1,-1,-1 };
	if(simulation_setting==2){
		std::vector<int> vec_T = dp.subsets_by_id[std::get<2>(pred_op)];
		backtrack_last_delivery_op_dp2(std::get<0>(pred_op), loc_w, vec_T);
		loc_w = std::get<0>(pred_op);
		pred_op = prev_d_operations_graph[loc_w][std::get<1>(pred_op)];
	} 
	while (pred_op != terminal_prev) {
		std::vector<int> vec_T = dp.subsets_by_id[std::get<2>(pred_op)];
		backtrack_op_dp2(std::get<0>(pred_op), loc_w, vec_T);
		loc_w = std::get<0>(pred_op);
		pred_op = prev_d_operations_graph[loc_w][std::get<1>(pred_op)];
	}
}


void Model_DP::init_reopt_model_struct()
{
	if (curr_drone_node >= instance.n_nodes) {
		add_artificial_drone_node(reopt_parameters.artificial_node);
	}
	else if ((curr_truck_node >= instance.n_nodes)) {
		add_artificial_truck_node(reopt_parameters.artificial_node);
	}
	else if ((curr_truck_node != curr_drone_node)) {
		add_artificial_drone_node(reopt_parameters.artificial_node);
	}
	//reopt_parameters.artificial_node = ArtificialNode();
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	model_solution.service_time_map = dp.service_time_map;
	model_solution.art_node_map = dp.artificial_nodes_map;
	if ((int)dp.distance_metric_naive.size() > instance.n_nodes || (int)dp.reopt_drone_distances.size() > instance.n_nodes) {
		number_of_nodes = max(dp.distance_metric_naive.size(), dp.reopt_drone_distances.size());
	}
}

void Model_DP::add_real_incoming_edges_after_solving()
{
	if (curr_drone_node >= instance.n_nodes) {
		add_real_incoming_edges(reopt_parameters.artificial_node, false);
	}
	else if ((curr_truck_node >= instance.n_nodes)) {
		add_real_incoming_edges(reopt_parameters.artificial_node, true);
	}
	else if ((curr_truck_node != curr_drone_node)) {
		add_real_incoming_edges(reopt_parameters.artificial_node, false);
	}
	reopt_parameters.artificial_node = ArtificialNode();
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	model_solution.service_time_map = dp.service_time_map;
	model_solution.art_node_map = dp.artificial_nodes_map;
}

double Model_DP::get_last_delivery_objective()
{	
	double best_obj = max_lim;
	double z;
	for(int w=0;w<instance.n_nodes;w++){
		z = d_table_operations_graph[w][d_table_operations_graph[0].size() - 1];
		if(z<best_obj){
			best_obj = z;
			model_solution.last_delivered_customer_obj2 = w;
			reopt_solution.last_delivered_customer_obj2 = w;
		} 
	}
    return best_obj;
}

/*
void Model_DP::adjust_artificial_edges()
{
	bool truck_node;
	for(int k=0; k< (int)dp.artificial_truck_node_bool.size();k++){
		truck_node = dp.artificial_truck_node_bool[k];
		int a = dp.real_artificial_edges[k].first;
		int b = dp.real_artificial_edges[k].second;
		int v = instance.n_nodes+k; 
		if(truck_node){
			if(b!=a){
				dp.distance_metric_naive[a][v] = dp.real_artificial_edge_distances[k].first;
				dp.distance_metric_naive[v][a] = dp.real_artificial_edge_distances[k].first;
				dp.distance_metric_naive[b][v] = dp.real_artificial_edge_distances[k].second; 
				dp.distance_metric_naive[v][b] = dp.real_artificial_edge_distances[k].second; 
			}
			else{
				dp.distance_metric_naive[a][v] = dp.real_artificial_edge_distances[k].first;
				dp.distance_metric_naive[v][a] = dp.real_artificial_edge_distances[k].second;
			} 
		}
		else{
			if(b!=a){
				dp.reopt_drone_distances[a][v] = dp.real_artificial_edge_distances[k].first;
				dp.reopt_drone_distances[v][a] = dp.real_artificial_edge_distances[k].first;  
				dp.reopt_drone_distances[b][v] = dp.real_artificial_edge_distances[k].second;
				dp.reopt_drone_distances[v][b] = dp.real_artificial_edge_distances[k].second; 
			}
			else{
				dp.reopt_drone_distances[a][v] = dp.real_artificial_edge_distances[k].first;
				dp.reopt_drone_distances[v][a] = dp.real_artificial_edge_distances[k].second; 
			} 	
		}  

	} 
}
*/

double Model_DP::evaluate_artificial_truck_op(int _curr_truck_node, int w, int artificial_node, int index, double truck_distance)
{
	std::vector<int> temp_sets = {0}; 
	double z = 0.0;
	if(std::find(customer_set.begin(), customer_set.end(), artificial_node) != customer_set.end()){
		std::vector<int> temp_vec = {artificial_node};
		int temp_set_id = dp.subsets[1][temp_vec];
		temp_sets.push_back(temp_set_id);
	}
	for(int temp_set_id : temp_sets) 	{
		if(dp.dtsp[_curr_truck_node][artificial_node][temp_set_id]+dp.dtsp[artificial_node][w][index]< truck_distance){
			z = dp.dtsp[_curr_truck_node][artificial_node][temp_set_id]+dp.dtsp[artificial_node][w][index];
			vector<int> temp_set = dp.subsets_by_id[temp_set_id];
			vector<int> index_set = dp.subsets_by_id[index];
			set<int> index2_set(index_set.begin(), index_set.end()); 
			for(int i:temp_set){
				index2_set.insert(i);
			} 
			vector<int> index2_vec(index2_set.begin(), index2_set.end());
			int index2 = dp.subsets[index2_vec.size()][index2_vec];
			if (index2==0 && std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end()) {
				vector<int> temp_vec(1,w);
				index2 = dp.subsets[1].at(temp_vec);
			}	
			if(z<dp.dtsp[_curr_truck_node][w][index2]){
				dp.dtsp[_curr_truck_node][w][index2] = z;
				int pred_node = dp.dtsp_pred[artificial_node][w][index].first;
				int pred_set = dp.dtsp_pred[artificial_node][w][index].second;
				dp.dtsp_pred[_curr_truck_node][w][index2] = { pred_node,pred_set};
				dp.dtsp_pred[_curr_truck_node][artificial_node][temp_set_id] = {_curr_truck_node,0};
				//fix truck dtsp predecessor
				pred_node = dp.dtsp_pred[artificial_node][w][index].first;
				pred_set = dp.dtsp_pred[artificial_node][w][index].second;
				int temp_pred = pred_node;
				int temp_pred_set = pred_set;
				bool improved_path = true;
				while(pred_node>-1) {
					temp_pred = pred_node;
					temp_pred_set = pred_set;
					//only update zero sets if no customer node next
					if (temp_pred_set==0 && std::find(customer_set.begin(), customer_set.end(), temp_pred) == customer_set.end()) {
						if(dp.dtsp[_curr_truck_node][temp_pred][temp_pred_set] > dp.dtsp[artificial_node][temp_pred][temp_pred_set]+dp.dtsp[_curr_truck_node][artificial_node][temp_set_id]){
							dp.dtsp[_curr_truck_node][temp_pred][temp_pred_set] = dp.dtsp[artificial_node][temp_pred][temp_pred_set]+dp.dtsp[_curr_truck_node][artificial_node][temp_set_id];
							dp.dtsp_pred[_curr_truck_node][temp_pred][temp_pred_set] = dp.dtsp_pred[artificial_node][temp_pred][temp_pred_set];
						}	
						else{
							improved_path=false;
						}
					}
					else{
						improved_path=false;
					}
					pred_node = dp.dtsp_pred[artificial_node][temp_pred][temp_pred_set].first;
				 	pred_set = dp.dtsp_pred[artificial_node][temp_pred][temp_pred_set].second;  
					
				}
				if(improved_path){
					dp.dtsp_pred[_curr_truck_node][temp_pred][temp_pred_set] = { _curr_truck_node,temp_set_id};
				} 	
				truck_distance = z;
			} 
		}
	}
	return truck_distance;
}


void Model_DP::putsolutionOnScreenDP() {
	cout << "Objective Value: ";
	//correct the objective by the penalty_factor value
	if(penalty_factor>1.0) {
		init_dp_struct(0);
		double corrected_z = 0.0;
		int curr_temp_truck_node = curr_truck_node;
		int curr_temp_drone_node = curr_drone_node;
		int next_node = curr_truck_node;
		vector<bool> delivered_customers = vector<bool>(instance.n_nodes, false);
		for (int stage=0; stage < (int) model_solution.truck_sorties.size(); stage++) {
			double truck_operation = 0.0;
			double drone_operation = 0.0;
			//check if first node in case of replanning
			if ((delivered_customers[curr_temp_truck_node] == false) && (customers_to_visit[curr_temp_truck_node] == true) && (curr_temp_truck_node!=model_solution.drone_customers[stage].back())) {
				if(std::find(model_solution.list_of_op_customers[stage].begin(), model_solution.list_of_op_customers[stage].end(), curr_temp_truck_node) != model_solution.list_of_op_customers[stage].end()) {
					truck_operation = truck_operation+service_time;
					delivered_customers[curr_temp_truck_node] = true;
				}
			}
			if((int) model_solution.truck_sorties[stage].size()>1) {
				for(int curr_node_id = 1;curr_node_id< (int) model_solution.truck_sorties[stage].size();curr_node_id++) {
					next_node = model_solution.truck_sorties[stage][curr_node_id];
					truck_operation = truck_operation+dp.distance_metric_naive[curr_temp_truck_node][next_node];
					//should not be a drone customer
					if ((delivered_customers[next_node] == false) && (customers_to_visit[next_node] == true) && (next_node!=model_solution.drone_customers[stage].back())) {
						if(std::find(model_solution.list_of_op_customers[stage].begin(), model_solution.list_of_op_customers[stage].end(), next_node) != model_solution.list_of_op_customers[stage].end()) {
							truck_operation = truck_operation+service_time;
							delivered_customers[next_node] = true;
						}
					}
					curr_temp_truck_node = next_node;
				}
			}
			next_node = curr_temp_drone_node;
			//check if first customer in replanning situations
			if ((delivered_customers[next_node] == false) && (customers_to_visit[next_node] == true) && (next_node==model_solution.drone_customers[stage].back())) {
				drone_operation = drone_operation+service_time;
				delivered_customers[next_node] = true;
			}
			if((int) model_solution.drone_sorties[stage].size()>1) {
				for(int curr_node_id = 1;curr_node_id< (int) model_solution.drone_sorties[stage].size();curr_node_id++) {
					next_node = model_solution.drone_sorties[stage][curr_node_id];
					drone_operation = drone_operation+dp.reopt_drone_distances[curr_temp_drone_node][next_node];
					//check if drone customer true
					if ((delivered_customers[next_node] == false) && (customers_to_visit[next_node] == true) && (next_node==model_solution.drone_customers[stage].back())) {
						drone_operation = drone_operation+service_time;
						delivered_customers[next_node] = true;
					}
					curr_temp_drone_node = next_node;
				}
			}
			else{
				//drone carried by truck
				curr_temp_drone_node = curr_temp_truck_node;
			}
			corrected_z = corrected_z + max(truck_operation,drone_operation);
		}
		model_solution.obj_value = corrected_z;
	}
	cout << model_solution.obj_value;
	cout << '\n';
};

void Model_DP::write_solution_dp_reopt() {

	//int n_nodes = dp.n_nodes + dp.artificial_nodes;

	//initialize short_output.list_truck_edges
	model_solution.list_truck_edges.resize(0);
	model_solution.drone_sorties.resize(0);
	model_solution.truck_sorties.resize(0);
	model_solution.list_of_op_customers = Ledges_struct.Ledges_customers;
	//reversal needed because from end to start
	std::reverse(model_solution.list_of_op_customers.begin(), model_solution.list_of_op_customers.end());
	vector<int>hilf_vector_sorties = vector<int>(0);
	model_solution.list_truck_edges = vector<vector<int>>(instance.n_nodes, vector<int>(instance.n_nodes));
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			model_solution.list_truck_edges[loc_i][loc_j] = 0;
		};
	};
	//initialize further structures
	model_solution.n_stages = 0;
	vector<vector<int>>list_of_cycles = vector<vector<int>>(0);//current position in the cycle (important for the cycle traversal), then Ledges of the cycle
	vector<vector<int>>list_of_cycles_from_nodes = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//for each node, the number of cycles to which it belongs, then the IDs of the cycles.

	vector<vector<int>>loc_list_ledges = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//find the list of outgoing Ledges. NUmber of Ledges, then IDs
	vector<vector<int>>loc_list_visited_ledges = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//number visited Ledges, then 1 if visited, otherwise ==>0.For notating cycles
	int total_n_ledges = 0;
	vector<int> extracted_Ledges_num = get_ledges_to_visit();
	for (int loc_i : extracted_Ledges_num) {
		int from_node = Ledges_struct.Ledges_from_end_to_start[loc_i].back();
		loc_list_ledges[from_node][0] = loc_list_ledges[from_node][0] + 1;
		loc_list_ledges[from_node].push_back(loc_i);
		loc_list_visited_ledges[from_node].push_back(0);
		total_n_ledges = total_n_ledges + 1;
	};//for each Ledge
	vector<int>sequence_of_ledges_to_visit = vector<int>(total_n_ledges, -1);
	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//if (curr_truck_node == curr_drone_node && curr_truck_node == 0) {
		//std::reverse(sequence_of_ledges_to_visit.begin(), sequence_of_ledges_to_visit.end());
	//}
	//if (n_replan_reopt > 0) {
	//	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//}

	

	model_solution.n_stages = 0;
	//std::iota(std::begin(sequence_of_ledges_to_visit), std::end(sequence_of_ledges_to_visit), 0);
	for (int loc_t = 0; loc_t < total_n_ledges; loc_t++) {//
		int next_ledge_num = sequence_of_ledges_to_visit[loc_t];
		//if (globparameters.n_replan_reopt > 0 && next_ledge_num==n_Ledges-1) {
		//	continue;
		//}
		//int node_1 = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1];

		//int node_2 = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][0];


		//write the stage: if either a drone node or a truck edge are present
		bool drone_stage_exists = false;
		if (Ledges_struct.Ledges_dronenode[next_ledge_num] > -1) {
			drone_stage_exists = true;
		};
		model_solution.n_stages = model_solution.n_stages + 1;
		model_solution.truck_sorties.push_back(hilf_vector_sorties);
		model_solution.drone_sorties.push_back(hilf_vector_sorties);
		//insert a stage
		if (drone_stage_exists == true) {
			//insert the drone node
			int first_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
			model_solution.drone_customers.push_back(vector<int>(1, Ledges_struct.Ledges_dronenode[next_ledge_num]));
			int third_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].front();
			int second_node = Ledges_struct.Ledges_dronenode[next_ledge_num];
			vector<int> drone_op(0);
			if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
				first_node = curr_drone_node;
			}
			
			drone_op = backtrack_path(first_node, second_node, dp.reopt_drone_fromtriple_previousnode, drone_op);
			drone_op = backtrack_path(second_node, third_node, dp.reopt_drone_fromtriple_previousnode, drone_op);

			model_solution.drone_sorties.back() = drone_op;
		}//if there is a drone stage
		else {
			if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
				vector<int> drone_op(0);
				int first_node = curr_drone_node;
				int second_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].front();
				drone_op = backtrack_path(first_node, second_node, dp.reopt_drone_fromtriple_previousnode, drone_op);
				model_solution.drone_sorties.back() = drone_op;
			}
			else {
				model_solution.drone_sorties.back().push_back(-1);
			}
			model_solution.drone_customers.push_back(vector<int>(1, -1));
		}

		//insert truck nodes
		//if two truck node are the same==> save only 1 node, because it is a loop then (otherwise the check procedure is false)
		//int start_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
		if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
			if (curr_truck_node != 0) {
				Ledges_struct.Ledges_from_end_to_start[next_ledge_num].pop_back();
			}
			flag_reopt_start_idx = model_solution.n_stages - 1;
			//int start_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
		}
		vector<int> copy_loc = Ledges_struct.Ledges_from_end_to_start[next_ledge_num];
		std::reverse(copy_loc.begin(), copy_loc.end());
		model_solution.truck_sorties.back() = copy_loc;
		/*
		for (size_t loc_j = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1; loc_j-- > 0;) {
			int goal_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][loc_j];
			model_solution.truck_sorties.back() = backtrack_path(start_node, goal_node, dp.previous_node, model_solution.truck_sorties.back());
			start_node = goal_node;
		}*/
		//insert truck edges
		for (int loc_j = 1; loc_j < (int)model_solution.truck_sorties.back().size(); loc_j++) {
			int previousnode = model_solution.truck_sorties.back()[loc_j - 1];
			int nextnode = model_solution.truck_sorties.back()[loc_j];
			if (previousnode != nextnode && previousnode < instance.n_nodes && nextnode < instance.n_nodes) {
				model_solution.list_truck_edges[previousnode][nextnode] = 1;
				model_solution.list_truck_edges[nextnode][previousnode] = 1;
			};
		};
	};//if the next Ledge is found
}

void Model_DP::write_solution_dp_reopt_last_delivery() {

	//int n_nodes = dp.n_nodes + dp.artificial_nodes;

	//initialize short_output.list_truck_edges
	model_solution.list_truck_edges.resize(0);
	model_solution.drone_sorties.resize(0);
	model_solution.truck_sorties.resize(0);
	vector<int>hilf_vector_sorties = vector<int>(0);
	model_solution.list_of_op_customers = Ledges_struct.Ledges_customers;
	//reversal needed because from end to start
	std::reverse(model_solution.list_of_op_customers.begin(), model_solution.list_of_op_customers.end());
	model_solution.list_truck_edges = vector<vector<int>>(instance.n_nodes, vector<int>(instance.n_nodes));
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			model_solution.list_truck_edges[loc_i][loc_j] = 0;
		};
	};
	//initialize further structures
	model_solution.n_stages = 0;
	vector<vector<int>>list_of_cycles = vector<vector<int>>(0);//current position in the cycle (important for the cycle traversal), then Ledges of the cycle
	vector<vector<int>>list_of_cycles_from_nodes = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//for each node, the number of cycles to which it belongs, then the IDs of the cycles.

	vector<vector<int>>loc_list_ledges = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//find the list of outgoing Ledges. NUmber of Ledges, then IDs
	vector<vector<int>>loc_list_visited_ledges = vector<vector<int>>(instance.n_nodes, vector<int>(1, 0));//number visited Ledges, then 1 if visited, otherwise ==>0.For notating cycles
	int total_n_ledges = 0;
	vector<int> extracted_Ledges_num = get_ledges_to_visit();
	for (int loc_i : extracted_Ledges_num) {
		int from_node = Ledges_struct.Ledges_from_end_to_start[loc_i].back();
		loc_list_ledges[from_node][0] = loc_list_ledges[from_node][0] + 1;
		loc_list_ledges[from_node].push_back(loc_i);
		loc_list_visited_ledges[from_node].push_back(0);
		total_n_ledges = total_n_ledges + 1;
	};//for each Ledge
	vector<int>sequence_of_ledges_to_visit = vector<int>(total_n_ledges, -1);
	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//if (curr_truck_node == curr_drone_node && curr_truck_node == 0) {
		//std::reverse(sequence_of_ledges_to_visit.begin(), sequence_of_ledges_to_visit.end());
	//}
	//if (n_replan_reopt > 0) {
	//	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//}

	

	model_solution.n_stages = 0;
	//std::iota(std::begin(sequence_of_ledges_to_visit), std::end(sequence_of_ledges_to_visit), 0);
	for (int loc_t = 0; loc_t < total_n_ledges; loc_t++) {//
		int next_ledge_num = sequence_of_ledges_to_visit[loc_t];
		//if (globparameters.n_replan_reopt > 0 && next_ledge_num==n_Ledges-1) {
		//	continue;
		//}
		//int node_1 = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1];

		//int node_2 = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][0];


		//write the stage: if either a drone node or a truck edge are present
		bool drone_stage_exists = false;
		if (Ledges_struct.Ledges_dronenode[next_ledge_num] > -1) {
			drone_stage_exists = true;
		};
		model_solution.n_stages = model_solution.n_stages + 1;
		model_solution.truck_sorties.push_back(hilf_vector_sorties);
		model_solution.drone_sorties.push_back(hilf_vector_sorties);
		//insert a stage
		if (drone_stage_exists == true) {
			//insert the drone node
			int first_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
			model_solution.drone_customers.push_back(vector<int>(1, Ledges_struct.Ledges_dronenode[next_ledge_num]));
			int third_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].front();
			int second_node = Ledges_struct.Ledges_dronenode[next_ledge_num];
			vector<int> drone_op(0);
			if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
				first_node = curr_drone_node;
			}
			
			if(loc_t==total_n_ledges-1){
				drone_op = backtrack_path(first_node, second_node, dp.reopt_drone_fromtriple_previousnode, drone_op);
			}
			else{
				drone_op = backtrack_path(first_node, second_node, dp.reopt_drone_fromtriple_previousnode, drone_op);
				drone_op = backtrack_path(second_node, third_node, dp.reopt_drone_fromtriple_previousnode, drone_op);

			} 
			model_solution.drone_sorties.back() = drone_op;
		}//if there is a drone stage
		else {
			if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
				vector<int> drone_op(0);
				int first_node = curr_drone_node;
				int second_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].front();
				drone_op = backtrack_path(first_node, second_node, dp.reopt_drone_fromtriple_previousnode, drone_op);	
				model_solution.drone_sorties.back() = drone_op;
			}
			else {
				model_solution.drone_sorties.back().push_back(-1);
			}
			model_solution.drone_customers.push_back(vector<int>(1, -1));
		}

		//insert truck nodes
		//if two truck node are the same==> save only 1 node, because it is a loop then (otherwise the check procedure is false)
		//int start_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
		if (std::find(Ledges_struct.reopt_ct_ledges.begin(), Ledges_struct.reopt_ct_ledges.end(), next_ledge_num) != Ledges_struct.reopt_ct_ledges.end()) {
			if (curr_truck_node != 0) {
				Ledges_struct.Ledges_from_end_to_start[next_ledge_num].pop_back();
			}
			flag_reopt_start_idx = model_solution.n_stages - 1;
			//int start_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
		}
		vector<int> copy_loc = Ledges_struct.Ledges_from_end_to_start[next_ledge_num];
		std::reverse(copy_loc.begin(), copy_loc.end());
		model_solution.truck_sorties.back() = copy_loc;
		/*
		for (size_t loc_j = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1; loc_j-- > 0;) {
			int goal_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][loc_j];
			model_solution.truck_sorties.back() = backtrack_path(start_node, goal_node, dp.previous_node, model_solution.truck_sorties.back());
			start_node = goal_node;
		}*/
		//insert truck edges
		for (int loc_j = 1; loc_j < (int)model_solution.truck_sorties.back().size(); loc_j++) {
			int previousnode = model_solution.truck_sorties.back()[loc_j - 1];
			int nextnode = model_solution.truck_sorties.back()[loc_j];
			if (previousnode != nextnode && previousnode < instance.n_nodes && nextnode < instance.n_nodes) {
				model_solution.list_truck_edges[previousnode][nextnode] = 1;
				model_solution.list_truck_edges[nextnode][previousnode] = 1;
			};
		};
	};//if the next Ledge is found
}


vector<int> Model_DP::get_ledges_to_visit(){
	vector<int> loc_visited_ledges(0);
	for (int loc_i = 0; loc_i < Ledges_struct.n_Ledges; loc_i++) {
		loc_visited_ledges.push_back(loc_i);
	};//for each Ledge
	return loc_visited_ledges;
};

void Model_DP::read_solution_complete_dp(bool surv) {
	for (int loc_i = Ledges_struct.Ledges_from_end_to_start.size() - 1; loc_i >= 0; --loc_i) {
		std::ostringstream  result;
		std::copy(Ledges_struct.Ledges_from_end_to_start[loc_i].rbegin(), Ledges_struct.Ledges_from_end_to_start[loc_i].rend() - 1, std::ostream_iterator<int>(result, "->"));
		result << Ledges_struct.Ledges_from_end_to_start[loc_i].front();
		if (surv) {
			cout << "Drone op: " + result.str();
		}
		else {
			cout << "Truck op: " + result.str() + " Drone node: " + to_string(Ledges_struct.Ledges_dronenode[loc_i]);
		}
		cout << '\n';
	}
};