#include "Model.h"


std::mutex m_dtsp;
std::mutex m_dop;

Model::Model(Instance& _instance, vector<vector<int>> _edge_status, vector<vector<int>>_arc_status_after_surveillance ,vector<bool> _customers_to_visit,bool _manhattan_truck, double _service_time, int _curr_truck_node, int _curr_drone_node, int _curr_drone_cust, int _last_combined_node):
	instance(_instance), model_solution(Solution(0,instance.n_nodes)), n_stages(instance.maxn_stages_long), edge_status(_edge_status), conservative(true), manhattan_truck(_manhattan_truck), customers_to_visit(_customers_to_visit), arc_status_after_surveillance(_arc_status_after_surveillance), mistake_text(""), mistake_reported(false), customer_set(init_customer_set()), customer_n_nodes(customer_set.size()), curr_truck_node(_curr_truck_node), curr_drone_node(_curr_drone_node), curr_drone_customer(_curr_drone_cust) ,last_combined_node(_last_combined_node), service_time(_service_time), dp{DP_tables(instance.n_nodes,customer_n_nodes,max_lim,instance)}, Ledges_struct{Ledges(instance.n_nodes)}
{
	
}

vector<int> Model::init_customer_set(){
	vector<int> loc_customer_set(0);
	for (int k = 1; k < (int)customers_to_visit.size(); k++) {
		if (customers_to_visit[k]) {
			loc_customer_set.push_back(k);
		}
	}
	return loc_customer_set;
}

void Model::add_artificial_truck_node(ArtificialNode artificial_node)
{
	dp.artificial_nodes = dp.artificial_nodes + 1;
	int new_size = dp.n_nodes + dp.artificial_nodes;
	for (int i = 0; i < (int)dp.distance_metric_naive.size(); i++) {
		dp.distance_metric_naive[i].push_back(max_lim);
	}
	dp.distance_metric_naive.push_back(vector<double>(new_size, max_lim));
	for (int i = 0; i < (int)dp.reopt_drone_distances.size(); i++) {
		dp.reopt_drone_distances[i].push_back(max_lim);
		dp.reopt_drone_fromtriple_previousnode[i].push_back(-1);
	}
	dp.reopt_drone_distances.push_back(vector<double>(new_size, max_lim));
	dp.reopt_drone_fromtriple_previousnode.push_back(vector<int>(new_size, -1));
	std::fill(dp.distance_metric_naive[artificial_node.id].begin(), dp.distance_metric_naive[artificial_node.id].end(), max_lim);
	/*
	dp.distance_metric_naive[artificial_node.edge.first][artificial_node.id] = artificial_node.edge_distance.first;
	dp.distance_metric_naive[artificial_node.id][artificial_node.edge.second] = artificial_node.edge_distance.second;
	*/
	/*
	if(artificial_node.service_node==false) {
		if(artificial_node.edge.first < instance.n_nodes) {
			dp.distance_metric_naive[artificial_node.id][artificial_node.edge.first] = artificial_node.edge_distance.first;
		}
		else {
			int prev_art_id = artificial_node.edge.first;
			int curr_node_id = artificial_node.edge.first;
			double new_edge_dist = artificial_node.edge_distance.first;
			while(curr_node_id>=instance.n_nodes) {
				prev_art_id = dp.artificial_nodes_map[curr_node_id].edge.first;
				new_edge_dist = new_edge_dist + dp.distance_metric_naive[prev_art_id][curr_node_id];
				curr_node_id = prev_art_id;
			}
		}
	}
	*/
	//add dummy coordinate
	dp.coordinates_reopt.push_back({-1,-1});
	//dtsp table
	if(artificial_node.id>=dp.n_nodes) {
		dp.artificial_nodes_map[artificial_node.id] = artificial_node;
		add_edge_of_artificial_node_to_dp_table(artificial_node.real_outgoing_edge);
		add_edge_of_artificial_node_to_dp_table(artificial_node.artificial_outgoing_edge);
	}
	dp.service_time_map[artificial_node.id] = artificial_node.service_time;
}

void Model::add_edge_of_artificial_node_to_dp_table(pair<pair<int, int>, double> edge_to_add)
{
	if(edge_to_add.second<max_lim) {
		dp.distance_metric_naive[edge_to_add.first.first][edge_to_add.first.second] = edge_to_add.second;
		vector<int>first(0);
		int idx = 0; //zero set if w no customer node
		if (std::find(customer_set.begin(), customer_set.end(), edge_to_add.first.second) != customer_set.end()) {
			first.resize(1, edge_to_add.first.second);
			idx = dp.subsets[1].at(first);
		}
		dp.dtsp[edge_to_add.first.first][edge_to_add.first.second][0] = dp.distance_metric_naive[edge_to_add.first.first][edge_to_add.first.second];
		dp.dtsp_pred[edge_to_add.first.first][edge_to_add.first.second][0] = {edge_to_add.first.first,0 };
		if(idx>0) {
				dp.dtsp[edge_to_add.first.first][edge_to_add.first.second][idx] = dp.distance_metric_naive[edge_to_add.first.first][edge_to_add.first.second] +service_time;
		}
		dp.dtsp_pred[edge_to_add.first.first][edge_to_add.first.second][idx] = {edge_to_add.first.first,0 };
		dp.dtsp[edge_to_add.first.first][edge_to_add.first.first][0] = 0;
		dp.distance_metric_naive[edge_to_add.first.first][edge_to_add.first.first] = 0;
	}
}

void Model::add_artificial_drone_node(ArtificialNode artificial_node)
{
	dp.artificial_nodes = dp.artificial_nodes + 1;
	int new_size = dp.n_nodes + dp.artificial_nodes;
	for (int i = 0; i < (int)dp.distance_metric_naive.size(); i++) {
		dp.distance_metric_naive[i].push_back(max_lim);
	}
	dp.distance_metric_naive.push_back(vector<double>(new_size, max_lim));
	for (int i = 0; i < (int)dp.reopt_drone_distances.size(); i++) {
		dp.reopt_drone_distances[i].push_back(max_lim);
		dp.reopt_drone_fromtriple_previousnode[i].push_back(-1);
	}
	dp.artificial_nodes_map[artificial_node.id] = artificial_node;
	dp.reopt_drone_distances.push_back(vector<double>(new_size, max_lim));
	dp.reopt_drone_fromtriple_previousnode.push_back(vector<int>(new_size, -1));
	int v= instance.n_nodes+dp.artificial_nodes-1;
	for (int w=0;w<instance.n_nodes;w++) {
		if (dp.distance_metric_naive[curr_truck_node][w] < max_lim) {
			vector<int>first(0);
			int idx = dp.subsets[0][first]; //zero set if w no customer node
			if (std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end()) {
				first.resize(1, w);
				idx = dp.subsets[1].at(first);
			}
			dp.dtsp[v][w][idx] = dp.distance_metric_naive[curr_truck_node][w];
			if(idx>0) {
				dp.dtsp[v][w][idx] = dp.dtsp[v][w][idx] + service_time;
			}
			if (v != w) {
				dp.dtsp_pred[v][w][idx] = { v,0 };
			}
		}
	}
	if(artificial_node.id>=dp.n_nodes) {
		add_edge_of_artificial_node_to_drone_reopt_dist(artificial_node.real_outgoing_edge);
		add_edge_of_artificial_node_to_drone_reopt_dist(artificial_node.artificial_outgoing_edge);
		dp.reopt_drone_distances[artificial_node.id][artificial_node.id] = 0;
		dp.reopt_drone_fromtriple_previousnode[artificial_node.id][artificial_node.id] = -1;
	}
	/*
	dp.reopt_drone_distances[artificial_node.edge.first][artificial_node.id] = artificial_node.edge_distance.first;
	dp.reopt_drone_fromtriple_previousnode[artificial_node.edge.first][artificial_node.id] = artificial_node.edge.first;
	dp.reopt_drone_distances[artificial_node.id][artificial_node.edge.second] = artificial_node.edge_distance.second;
	dp.reopt_drone_fromtriple_previousnode[artificial_node.id][artificial_node.edge.second] = artificial_node.id;
	if(artificial_node.service_node==false) {
		if(artificial_node.edge.first < instance.n_nodes) {
			dp.reopt_drone_distances[artificial_node.id][artificial_node.edge.first] = artificial_node.edge_distance.first;
			dp.reopt_drone_fromtriple_previousnode[artificial_node.id][artificial_node.edge.first] = artificial_node.id;
		}
		else {
			int prev_art_id = artificial_node.edge.first;
			int curr_node_id = artificial_node.edge.first;
			double new_edge_dist = artificial_node.edge_distance.first;
			//Implement shortcuts!!!
			while(curr_node_id>=instance.n_nodes) {
				prev_art_id = dp.artificial_nodes_map[curr_node_id].edge.first;
				new_edge_dist = new_edge_dist + dp.reopt_drone_distance[prev_art_id][curr_node_id];
				curr_node_id = prev_art_id;
			}
		}
	}
	*/
	//check if shortcuts and the artificial node is somewhere between the two nodes (no waiting node)
	if((artificial_node.id>=instance.n_nodes) && (instance.shortcuts == true) && (artificial_node.service_node == false)) {
		//double dt = artificial_node.edge.first;
		double temp_dist = artificial_node.real_incoming_edge.second+artificial_node.real_outgoing_edge.second;
		double x_new = (artificial_node.real_incoming_edge.second*dp.coordinates_reopt[artificial_node.second_node].first+artificial_node.real_outgoing_edge.second*dp.coordinates_reopt[artificial_node.first_node].first)/temp_dist;
		double y_new =   (artificial_node.real_incoming_edge.second*dp.coordinates_reopt[artificial_node.second_node].second+artificial_node.real_outgoing_edge.second*dp.coordinates_reopt[artificial_node.first_node].second)/temp_dist;
		//if we do not update the coordinates we may get misscalculations
		dp.coordinates_reopt.push_back({x_new,y_new});
		for(int i=0;i<instance.n_nodes;i++) {
			double z =	ceil(sqrt(pow((x_new-dp.coordinates_reopt[i].first),2) + pow((y_new-dp.coordinates_reopt[i].second),2)))/instance.s_drone;
			if(z<dp.reopt_drone_distances[artificial_node.id][i]) {
				dp.reopt_drone_distances[artificial_node.id][i] = z;
				dp.reopt_drone_fromtriple_previousnode[artificial_node.id][i] = artificial_node.id;
			}
		}
	}
	else {
		//add dummy coordinates
		dp.coordinates_reopt.push_back({-1,-1});
	}
	dp.service_time_map[artificial_node.id] = artificial_node.service_time;
	for (int loc_i = instance.n_nodes; loc_i < (int)dp.reopt_drone_distances.size(); loc_i++) {//transitknoten
		for (int loc_j = 0; loc_j < (int)dp.reopt_drone_distances.size(); loc_j++) {//startknoten
			if ((dp.reopt_drone_distances[loc_i][loc_j] < max_lim) && (loc_i != loc_j)) {
				for (int loc_k = 0; loc_k < (int)dp.reopt_drone_distances.size(); loc_k++) {//endknoten
					if ((dp.reopt_drone_distances[loc_j][loc_k] < max_lim) && (dp.reopt_drone_distances[loc_i][loc_j] + dp.reopt_drone_distances[loc_j][loc_k] < dp.reopt_drone_distances[loc_i][loc_k])) {
						//we don't want paths leading to artificial nodes
						if(loc_k<instance.n_nodes) {
							dp.reopt_drone_distances[loc_i][loc_k] = dp.reopt_drone_distances[loc_i][loc_j] + dp.reopt_drone_distances[loc_j][loc_k];
							dp.reopt_drone_fromtriple_previousnode[loc_i][loc_k] = dp.reopt_drone_fromtriple_previousnode[loc_j][loc_k];
						}
					};
				};
			};
		};
	};
}

void Model::add_edge_of_artificial_node_to_drone_reopt_dist(pair<pair<int, int>, double> edge_to_add)
{
	if(edge_to_add.second<max_lim) {
		dp.reopt_drone_distances[edge_to_add.first.first][edge_to_add.first.second] = edge_to_add.second;
		dp.reopt_drone_fromtriple_previousnode[edge_to_add.first.first][edge_to_add.first.second]  = edge_to_add.first.first;
	}
}

void Model::add_real_incoming_edges(ArtificialNode artificial_node, bool truck_node)
{
	if(truck_node) {
		add_edge_of_artificial_node_to_dp_table(artificial_node.real_incoming_edge);

	}
	else {
		add_edge_of_artificial_node_to_drone_reopt_dist(artificial_node.real_incoming_edge);
	}
}

bool Model::check_solution(Solution& solution, vector<vector<int>>& current_real_damages)
{
	double loc_obj_value = 0;
	//check whether all the customers are delivered
	vector<bool>delivered_customers;
	delivered_customers = vector<bool>(instance.n_nodes);
	for (int loc_i = 0; (loc_i < instance.n_nodes); loc_i++) {
		delivered_customers[loc_i] = false;
	};

	int last_previous_truck_node = curr_truck_node;
	int last_previous_drone_node = curr_drone_node;

	for (int loc_i = 0; loc_i < solution.n_stages; loc_i++) {
		//remove consecutive duplicated node
		vector<int>:: iterator it;
		it = unique(solution.truck_sorties[loc_i].begin(), solution.truck_sorties[loc_i].end());
		solution.truck_sorties[loc_i].erase(it, solution.truck_sorties[loc_i].end());
		it = unique(solution.drone_sorties[loc_i].begin(), solution.drone_sorties[loc_i].end());
		solution.drone_sorties[loc_i].erase(it, solution.drone_sorties[loc_i].end());
		double drone_sortie_duration = 0.0;
		double truck_sortie_duration = 0.0;
		//track the set of visited nodes
		vector<int> visited_set;
		int current_node = solution.truck_sorties[loc_i][0];
		if (last_previous_truck_node > 0) {
			if ((last_previous_truck_node!= solution.drone_customers[loc_i].back() ) && (delivered_customers[last_previous_truck_node] == false) && (customers_to_visit[last_previous_truck_node] == true)) {
				if(std::find(solution.list_of_op_customers[loc_i].begin(), solution.list_of_op_customers[loc_i].end(), last_previous_truck_node) != solution.list_of_op_customers[loc_i].end()) {
					truck_sortie_duration = truck_sortie_duration +service_time;
					delivered_customers[last_previous_truck_node] = true;
				}
			}
		};
		if (last_previous_truck_node != current_node) {
			cout << "MISTAKE: truck route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: truck route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.truck_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_truck_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.truck_sorties[loc_i].size(); loc_j++) {
				int truck_delivery_node = solution.truck_sorties[loc_i][loc_j];
				double distance_leg = dp.distance_metric_naive[loc_truck_previous_node][truck_delivery_node];
				//cout << "Truck leg of operation "<< loc_i << ": " << distance_leg <<'\n';
				if (loc_truck_previous_node != truck_delivery_node && loc_truck_previous_node < instance.n_nodes && truck_delivery_node < instance.n_nodes && current_real_damages[loc_truck_previous_node][truck_delivery_node] < 0) {
					cout << "MISTAKE: damaged edge is used ";
					cout << (loc_truck_previous_node);
					cout << "; ";
					cout << (truck_delivery_node);
					cout << '\n';
					mistake_reported = true;
					string loc_s = std::to_string(loc_truck_previous_node);
					mistake_text = mistake_text + "@check_sol: damaged edge outgoing from node ";
					mistake_text = mistake_text + loc_s;
					mistake_text = mistake_text + " is used; ";
				}
				truck_sortie_duration = truck_sortie_duration + distance_leg;
				if (truck_delivery_node > 0) {
					//account service time for reopt it must be a non artificial node; make sure that truck node is not a drone node of an operation
					if ((truck_delivery_node!= solution.drone_customers[loc_i].back() ) && (delivered_customers[truck_delivery_node] == false) && (customers_to_visit[truck_delivery_node] == true)) {
						//if last delivery and customer is moved to later operation
						if(std::find(solution.list_of_op_customers[loc_i].begin(), solution.list_of_op_customers[loc_i].end(), truck_delivery_node) != solution.list_of_op_customers[loc_i].end()) {
							truck_sortie_duration = truck_sortie_duration +dp.service_time_map[truck_delivery_node];
							delivered_customers[truck_delivery_node] = true;
							//check if artificial service time is used
						}
					}
					if (loc_j >0 && truck_delivery_node>=instance.n_nodes) {
						//service time is included in the artificial node
						truck_sortie_duration = truck_sortie_duration  + dp.service_time_map[truck_delivery_node];
					}
					visited_set.push_back(truck_delivery_node);
				};
				loc_truck_previous_node = truck_delivery_node;
			};
			last_previous_truck_node = loc_truck_previous_node;//initialization for the next stage
		};
		if (loc_i == 0) {
			current_node = curr_drone_node;
		}
		if ((last_previous_drone_node != current_node)) {
			cout << "MISTAKE: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: drone route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.drone_sorties[loc_i].size() >0 && solution.drone_sorties[loc_i][0] >-1) {//if drone is doing operation
			int loc_drone_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_next_node = solution.drone_sorties[loc_i][loc_j];
				double distance_leg = dp.reopt_drone_distances[loc_drone_previous_node][drone_next_node];
				//for last delivery customer, if it is a reopt operation and we have no drone customer, then the truck does not need to wait for the drone, still need to simulate the way if edge failure
				if( loc_i< solution.n_stages-1 || (solution.last_delivered_customer_obj2>-1 && solution.last_delivered_customers_last_delivery_op.second >-1 )|| solution.last_delivered_customer_obj2==-1){
					drone_sortie_duration = drone_sortie_duration + distance_leg;
				} 
				//check if drone customer and already visited
				if ((std::find(solution.drone_customers[loc_i].begin(), solution.drone_customers[loc_i].end(), drone_next_node) != solution.drone_customers[loc_i].end()) && (delivered_customers[drone_next_node] ==false)  && (customers_to_visit[drone_next_node] == true)){
					drone_sortie_duration = drone_sortie_duration + dp.service_time_map[drone_next_node];
					delivered_customers[drone_next_node] = true;
					visited_set.push_back(drone_next_node);
				}
				//check if artificial service time is used
				if (loc_j >0 && drone_next_node>=instance.n_nodes) {
					//service time is included in the artificial node, but not if it is the first node in the sortie
					drone_sortie_duration = drone_sortie_duration  + dp.service_time_map[drone_next_node];
				};
				loc_drone_previous_node = drone_next_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
			//check if drone only starts and lands on parking nodes
			//to many false positives, seems to be no issue
			/*
			int op_start_node = solution.drone_sorties[loc_i].front();
			int op_end_node = solution.drone_sorties[loc_i].back();
			if(op_start_node<instance.n_nodes && op_start_node==solution.truck_sorties[loc_i].front() && curr_drone_node!=op_start_node) {
				if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_start_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone starts on non parking node ";
					cout << op_start_node << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone starts on non parking node at the beginning of stage " + to_string(loc_i) + ";";
				}
			}
			if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_end_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone lands on non parking node ";
					cout << op_end_node  << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone lands on non parking node at the end of stage " + to_string(loc_i) + ";";
				}
			*/
		} 
		else{
			last_previous_drone_node = solution.truck_sorties[loc_i].back();
		}

		//stage duration
		if (drone_sortie_duration > truck_sortie_duration) {
			loc_obj_value = loc_obj_value + drone_sortie_duration;
		}
		else {
			loc_obj_value = loc_obj_value + truck_sortie_duration;
		};
		//get dp infos for debugging
		std::sort(visited_set.begin(), visited_set.end());
		std::vector<int> v_intersection(0);
    	std::set_intersection(visited_set.begin(), visited_set.end(), customer_set.begin(), customer_set.end(),
                          std::back_inserter(v_intersection));
		int visited_set_id = dp.subsets[v_intersection.size()][v_intersection];
		std::stringstream ss;
		for(size_t i = 0; i <  v_intersection.size(); ++i){
					ss << ",";
					ss <<  v_intersection[i];
		}
		std::string set_string = ss.str();
		cout << to_string(loc_i)+":  Obj: "+to_string(loc_obj_value) + " ; Truck duration: " + to_string(truck_sortie_duration) + " ; Drone duration: " + to_string(drone_sortie_duration) + " ; Set: (" + set_string + ") ; Set id " +to_string(visited_set_id)<< '\n';
		if ((truck_sortie_duration > 0) && (drone_sortie_duration > 0) && ((drone_sortie_duration > instance.max_drone_sortie) || (truck_sortie_duration > instance.max_drone_sortie * instance.s_drone))) {
			cout << "MISTAKE: drone sortie exceed the max sortie duration of";
			cout << (instance.max_drone_sortie) << '\n';
			mistake_reported = true;
			string loc_s = to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: drone sortie at ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "exceeds max sortie duration of ";
			mistake_text = mistake_text + to_string(instance.max_drone_sortie);
			mistake_text = mistake_text + "; ";
		};

	};

	//check, whether all customers have been delivered
	for (int loc_i = 0; loc_i < (instance.n_nodes); loc_i++) {
		if ((delivered_customers[loc_i] == false) && (customers_to_visit[loc_i] == true)) {
			cout << "MISTAKE: the following customer was not delivered ";
			cout << (loc_i) << '\n';
			mistake_reported = true;
			string loc_s = std::to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: customer ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "not delivered; ";
		};
	};
	if (((loc_obj_value + 0.5 < solution.obj_value) || (loc_obj_value - 0.5 > solution.obj_value))) {
		cout << "objective value is false. The true one; the one saved in the computed solution structure ";
		cout << loc_obj_value;
		cout << "; the one from the algorithm: ";
		cout << solution.obj_value << '\n';
		mistake_reported = true;
		mistake_text = mistake_text + "@truck_and_drone_model: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + ", from dp algorithm ";
		loc_s = std::to_string(solution.obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + "; ";
	}
	else {
		solution.obj_value = loc_obj_value;//because by Cplex the solution is within epsilon
	};
	solution.mistake_reported = mistake_reported;
	solution.mistake_text = mistake_text;
	return mistake_reported;
}


bool Model::check_solution_tsp(Solution& solution, vector<vector<int>>& current_real_damages)
{
	double loc_obj_value = 0;
	//check whether all the customers are delivered
	vector<bool>delivered_customers;
	delivered_customers = vector<bool>(instance.n_nodes);
	for (int loc_i = 0; (loc_i < instance.n_nodes); loc_i++) {
		delivered_customers[loc_i] = false;
	};

	int last_previous_truck_node = curr_truck_node;
	int last_previous_drone_node = curr_drone_node;

	for (int loc_i = 0; loc_i < solution.n_stages; loc_i++) {
		//remove consecutive duplicated node
		vector<int>:: iterator it;
		it = unique(solution.truck_sorties[loc_i].begin(), solution.truck_sorties[loc_i].end());
		solution.truck_sorties[loc_i].erase(it, solution.truck_sorties[loc_i].end());
		it = unique(solution.drone_sorties[loc_i].begin(), solution.drone_sorties[loc_i].end());
		solution.drone_sorties[loc_i].erase(it, solution.drone_sorties[loc_i].end());
		double drone_sortie_duration = 0.0;
		double truck_sortie_duration = 0.0;
		//track the set of visited nodes
		vector<int> visited_set;
		int current_node = solution.truck_sorties[loc_i][0];
		if (last_previous_truck_node > 0) {
			if(std::find(solution.list_of_op_customers[loc_i].begin(), solution.list_of_op_customers[loc_i].end(), last_previous_truck_node) != solution.list_of_op_customers[loc_i].end()) {
				truck_sortie_duration = truck_sortie_duration +service_time;
				delivered_customers[last_previous_truck_node] = true;
			}
		};
		if (last_previous_truck_node != current_node) {
			cout << "MISTAKE: truck route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: truck route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.truck_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_truck_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.truck_sorties[loc_i].size(); loc_j++) {
				int truck_delivery_node = solution.truck_sorties[loc_i][loc_j];
				double distance_leg = dp.distance_metric_naive[loc_truck_previous_node][truck_delivery_node];
				//cout << "Truck leg of operation "<< loc_i << ": " << distance_leg <<'\n';
				if (loc_truck_previous_node != truck_delivery_node && loc_truck_previous_node < instance.n_nodes && truck_delivery_node < instance.n_nodes && current_real_damages[loc_truck_previous_node][truck_delivery_node] < 0) {
					cout << "MISTAKE: damaged edge is used ";
					cout << (loc_truck_previous_node);
					cout << "; ";
					cout << (truck_delivery_node);
					cout << '\n';
					mistake_reported = true;
					string loc_s = std::to_string(loc_truck_previous_node);
					mistake_text = mistake_text + "@check_sol: damaged edge outgoing from node ";
					mistake_text = mistake_text + loc_s;
					mistake_text = mistake_text + " is used; ";
				}
				truck_sortie_duration = truck_sortie_duration + distance_leg;
				if (truck_delivery_node > 0) {
					//account service time for reopt it must be a non artificial node; make sure that truck node is not a drone node of an operation
					if ((truck_delivery_node!= solution.drone_customers[loc_i].back() ) && (delivered_customers[truck_delivery_node] == false) && (customers_to_visit[truck_delivery_node] == true)) {
						//if last delivery and customer is moved to later operation
						if(std::find(solution.list_of_op_customers[loc_i].begin(), solution.list_of_op_customers[loc_i].end(), truck_delivery_node) != solution.list_of_op_customers[loc_i].end()) {
							delivered_customers[truck_delivery_node] = true;
							//check if artificial service time is used
						}
					}
					visited_set.push_back(truck_delivery_node);
				};
				loc_truck_previous_node = truck_delivery_node;
			};
			last_previous_truck_node = loc_truck_previous_node;//initialization for the next stage
		};
		if (loc_i == 0) {
			current_node = curr_drone_node;
		}
		if ((last_previous_drone_node != current_node)) {
			cout << "MISTAKE: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: drone route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.drone_sorties[loc_i].size() >0 && solution.drone_sorties[loc_i][0] >-1) {//if drone is doing operation
			int loc_drone_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_next_node = solution.drone_sorties[loc_i][loc_j];
				double distance_leg = dp.reopt_drone_distances[loc_drone_previous_node][drone_next_node];
				//for last delivery customer, if it is a reopt operation and we have no drone customer, then the truck does not need to wait for the drone, still need to simulate the way if edge failure
				if( loc_i< solution.n_stages-1 || (solution.last_delivered_customer_obj2>-1 && solution.last_delivered_customers_last_delivery_op.second >-1 )|| solution.last_delivered_customer_obj2==-1){
					drone_sortie_duration = drone_sortie_duration + distance_leg;
				} 
				//check if drone customer and already visited
				if ((std::find(solution.drone_customers[loc_i].begin(), solution.drone_customers[loc_i].end(), drone_next_node) != solution.drone_customers[loc_i].end()) && (delivered_customers[drone_next_node] ==false)){
					delivered_customers[drone_next_node] = true;
					visited_set.push_back(drone_next_node);
				}
				//check if artificial service time is used
				if (loc_j >0 && drone_next_node>=instance.n_nodes) {
					//service time is included in the artificial node, but not if it is the first node in the sortie
					drone_sortie_duration = drone_sortie_duration;
				};
				loc_drone_previous_node = drone_next_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
			//check if drone only starts and lands on parking nodes
			//to many false positives, seems to be no issue
			/*
			int op_start_node = solution.drone_sorties[loc_i].front();
			int op_end_node = solution.drone_sorties[loc_i].back();
			if(op_start_node<instance.n_nodes && op_start_node==solution.truck_sorties[loc_i].front() && curr_drone_node!=op_start_node) {
				if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_start_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone starts on non parking node ";
					cout << op_start_node << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone starts on non parking node at the beginning of stage " + to_string(loc_i) + ";";
				}
			}
			if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_end_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone lands on non parking node ";
					cout << op_end_node  << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone lands on non parking node at the end of stage " + to_string(loc_i) + ";";
				}
			*/
		} 
		else{
			last_previous_drone_node = solution.truck_sorties[loc_i].back();
		}

		//stage duration
		if (drone_sortie_duration > truck_sortie_duration) {
			loc_obj_value = loc_obj_value + drone_sortie_duration;
		}
		else {
			loc_obj_value = loc_obj_value + truck_sortie_duration;
		};
		//get dp infos for debugging
		std::sort(visited_set.begin(), visited_set.end());
		std::vector<int> v_intersection(0);
    	std::set_intersection(visited_set.begin(), visited_set.end(), customer_set.begin(), customer_set.end(),
                          std::back_inserter(v_intersection));
		int visited_set_id = dp.subsets[v_intersection.size()][v_intersection];
		std::stringstream ss;
		for(size_t i = 0; i <  v_intersection.size(); ++i){
					ss << ",";
					ss <<  v_intersection[i];
		}
		std::string set_string = ss.str();
		cout << to_string(loc_i)+":  Obj: "+to_string(loc_obj_value) + " ; Truck duration: " + to_string(truck_sortie_duration) + " ; Drone duration: " + to_string(drone_sortie_duration) + " ; Set: (" + set_string + ") ; Set id " +to_string(visited_set_id)<< '\n';
		if ((truck_sortie_duration > 0) && (drone_sortie_duration > 0) && ((drone_sortie_duration > instance.max_drone_sortie) || (truck_sortie_duration > instance.max_drone_sortie * instance.s_drone))) {
			cout << "MISTAKE: drone sortie exceed the max sortie duration of";
			cout << (instance.max_drone_sortie) << '\n';
			mistake_reported = true;
			string loc_s = to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: drone sortie at ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "exceeds max sortie duration of ";
			mistake_text = mistake_text + to_string(instance.max_drone_sortie);
			mistake_text = mistake_text + "; ";
		};

	};

	//check, whether all customers have been delivered
	for (int loc_i = 0; loc_i < (instance.n_nodes); loc_i++) {
		if ((delivered_customers[loc_i] == false) && (customers_to_visit[loc_i] == true)) {
			cout << "MISTAKE: the following customer was not delivered ";
			cout << (loc_i) << '\n';
			mistake_reported = true;
			string loc_s = std::to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: customer ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "not delivered; ";
		};
	};
	if (((loc_obj_value + 0.5 < solution.obj_value) || (loc_obj_value - 0.5 > solution.obj_value))) {
		cout << "objective value is false. The true one; the one saved in the computed solution structure ";
		cout << loc_obj_value;
		cout << "; the one from the algorithm: ";
		cout << solution.obj_value << '\n';
		mistake_reported = true;
		mistake_text = mistake_text + "@truck_and_drone_model: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + ", from dp algorithm ";
		loc_s = std::to_string(solution.obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + "; ";
	}
	else {
		solution.obj_value = loc_obj_value;//because by Cplex the solution is within epsilon
	};
	solution.mistake_reported = mistake_reported;
	solution.mistake_text = mistake_text;
	return mistake_reported;
}



bool Model::check_solution_new_tsp(Solution &solution, vector<vector<int>> &current_real_damages)
{
	double loc_obj_value = 0;
	//check whether all the customers are delivered
	vector<bool>delivered_customers;
	delivered_customers = vector<bool>(instance.n_nodes);
	for (int loc_i = 0; (loc_i < instance.n_nodes); loc_i++) {
		delivered_customers[loc_i] = false;
	};

	int last_previous_truck_node = curr_truck_node;
	int last_previous_drone_node = curr_drone_node;
	if (last_previous_truck_node > 0) {
		delivered_customers[last_previous_truck_node] = true;
	};
	for (int loc_i = 0; loc_i < solution.n_stages; loc_i++) {
		//remove consecutive duplicated node
		vector<int>:: iterator it;
		it = unique(solution.truck_sorties[loc_i].begin(), solution.truck_sorties[loc_i].end());
		solution.truck_sorties[loc_i].erase(it, solution.truck_sorties[loc_i].end());
		it = unique(solution.drone_sorties[loc_i].begin(), solution.drone_sorties[loc_i].end());
		solution.drone_sorties[loc_i].erase(it, solution.drone_sorties[loc_i].end());
		double drone_sortie_duration = 0.0;
		double truck_sortie_duration = 0.0;
		//track the set of visited nodes
		vector<int> visited_set;
		int current_node = solution.truck_sorties[loc_i][0];
		if (last_previous_truck_node != current_node) {
			cout << "MISTAKE: truck route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: truck route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.truck_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_truck_previous_node = current_node;
			for (int loc_j = 1; loc_j < (int)solution.truck_sorties[loc_i].size(); loc_j++) {
				int truck_delivery_node = solution.truck_sorties[loc_i][loc_j];
				double distance_leg = dp.distance_metric_naive[loc_truck_previous_node][truck_delivery_node];
				//cout << "Truck leg of operation "<< loc_i << ": " << distance_leg <<'\n';
				if (loc_truck_previous_node != truck_delivery_node && loc_truck_previous_node < instance.n_nodes && truck_delivery_node < instance.n_nodes && current_real_damages[loc_truck_previous_node][truck_delivery_node] < 0) {
					cout << "MISTAKE: damaged edge is used ";
					cout << (loc_truck_previous_node);
					cout << "; ";
					cout << (truck_delivery_node);
					cout << '\n';
					mistake_reported = true;
					string loc_s = std::to_string(loc_truck_previous_node);
					mistake_text = mistake_text + "@check_sol: damaged edge outgoing from node ";
					mistake_text = mistake_text + loc_s;
					mistake_text = mistake_text + " is used; ";
				}
				truck_sortie_duration = truck_sortie_duration + distance_leg;
				if (truck_delivery_node > 0) {
					delivered_customers[truck_delivery_node] = true;
					visited_set.push_back(truck_delivery_node);
				};
				loc_truck_previous_node = truck_delivery_node;
			};
			last_previous_truck_node = loc_truck_previous_node;//initialization for the next stage
		};
		if (loc_i == 0) {
			current_node = curr_drone_node;
		}
		if ((last_previous_drone_node != current_node)) {
			cout << "MISTAKE: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: drone route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.drone_sorties[loc_i].size() >0 && solution.drone_sorties[loc_i][0] >-1) {//if drone is doing operation
			int loc_drone_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_next_node = solution.drone_sorties[loc_i][loc_j];
				double distance_leg = dp.reopt_drone_distances[loc_drone_previous_node][drone_next_node];
				//for last delivery customer, if it is a reopt operation and we have no drone customer, then the truck does not need to wait for the drone, still need to simulate the way if edge failure
				if( loc_i< solution.n_stages-1 || (solution.last_delivered_customer_obj2>-1 && solution.last_delivered_customers_last_delivery_op.second >-1 )|| solution.last_delivered_customer_obj2==-1){
					drone_sortie_duration = drone_sortie_duration + distance_leg;
				} 
				if (std::find(solution.drone_customers[loc_i].begin(), solution.drone_customers[loc_i].end(), drone_next_node) != solution.drone_customers[loc_i].end()){
					delivered_customers[drone_next_node] = true;
					visited_set.push_back(drone_next_node);
				};
				loc_drone_previous_node = drone_next_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
			//check if drone only starts and lands on parking nodes
			//to many false positives, seems to be no issue
			/*
			int op_start_node = solution.drone_sorties[loc_i].front();
			int op_end_node = solution.drone_sorties[loc_i].back();
			if(op_start_node<instance.n_nodes && op_start_node==solution.truck_sorties[loc_i].front() && curr_drone_node!=op_start_node) {
				if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_start_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone starts on non parking node ";
					cout << op_start_node << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone starts on non parking node at the beginning of stage " + to_string(loc_i) + ";";
				}
			}
			if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_end_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone lands on non parking node ";
					cout << op_end_node  << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone lands on non parking node at the end of stage " + to_string(loc_i) + ";";
				}
			*/
		} 
		else{
			last_previous_drone_node = solution.truck_sorties[loc_i].back();
		}

		//stage duration
		if (drone_sortie_duration > truck_sortie_duration) {
			loc_obj_value = loc_obj_value + drone_sortie_duration;
		}
		else {
			loc_obj_value = loc_obj_value + truck_sortie_duration;
		};
		//get dp infos for debugging
		std::sort(visited_set.begin(), visited_set.end());
		std::vector<int> v_intersection(0);
    	std::set_intersection(visited_set.begin(), visited_set.end(), customer_set.begin(), customer_set.end(),
                          std::back_inserter(v_intersection));
		int visited_set_id = dp.subsets[v_intersection.size()][v_intersection];
		std::stringstream ss;
		for(size_t i = 0; i <  v_intersection.size(); ++i){
					ss << ",";
					ss <<  v_intersection[i];
		}
		std::string set_string = ss.str();
		cout << to_string(loc_i)+":  Obj: "+to_string(loc_obj_value) + " ; Truck duration: " + to_string(truck_sortie_duration) + " ; Drone duration: " + to_string(drone_sortie_duration) + " ; Set: (" + set_string + ") ; Set id " +to_string(visited_set_id)<< '\n';
		if ((truck_sortie_duration > 0) && (drone_sortie_duration > 0) && ((drone_sortie_duration > instance.max_drone_sortie) || (truck_sortie_duration > instance.max_drone_sortie * instance.s_drone))) {
			cout << "MISTAKE: drone sortie exceed the max sortie duration of";
			cout << (instance.max_drone_sortie) << '\n';
			mistake_reported = true;
			string loc_s = to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: drone sortie at ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "exceeds max sortie duration of ";
			mistake_text = mistake_text + to_string(instance.max_drone_sortie);
			mistake_text = mistake_text + "; ";
		};

	};

	if (((loc_obj_value + 0.5 < solution.obj_value) || (loc_obj_value - 0.5 > solution.obj_value))) {
		cout << "objective value is false. The true one; the one saved in the computed solution structure ";
		cout << loc_obj_value;
		cout << "; the one from the algorithm: ";
		cout << solution.obj_value << '\n';
		mistake_reported = true;
		mistake_text = mistake_text + "@truck_and_drone_model: objective value is false, the true one ";
		string loc_s = std::to_string(loc_obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + ", from dp algorithm ";
		loc_s = std::to_string(solution.obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + "; ";
	}
	else {
		solution.obj_value = loc_obj_value;//because by Cplex the solution is within epsilon
	};
	solution.mistake_reported = mistake_reported;
	solution.mistake_text = mistake_text;
	return mistake_reported;
}

bool Model::check_solution_survop(Solution& solution, vector<vector<int>>& current_real_damages, double cmax)
{
	double loc_obj_value = 0;
	if(areEqual(solution.obj_value,0.0)==true) {
		return true;
	}
	//check whether all the customers are delivered
	vector<bool>delivered_customers;
	delivered_customers = vector<bool>(instance.n_nodes);
	for (int loc_i = 0; (loc_i < instance.n_nodes); loc_i++) {
		delivered_customers[loc_i] = false;
	};

	int last_previous_truck_node = curr_truck_node;
	int last_previous_drone_node = curr_drone_node;
	if (last_previous_truck_node > 0) {
		delivered_customers[last_previous_truck_node] = true;
	};
	for (int loc_i = 0; loc_i < solution.n_stages; loc_i++) {
		//remove consecutive duplicated node
		vector<int>:: iterator it;
		it = unique(solution.truck_sorties[loc_i].begin(), solution.truck_sorties[loc_i].end());
		solution.truck_sorties[loc_i].erase(it, solution.truck_sorties[loc_i].end());
		it = unique(solution.drone_sorties[loc_i].begin(), solution.drone_sorties[loc_i].end());
		solution.drone_sorties[loc_i].erase(it, solution.drone_sorties[loc_i].end());
		double drone_sortie_duration = 0.0;
		double truck_sortie_duration = 0.0;
		//track the set of visited nodes
		vector<int> visited_set;
		int current_node = solution.truck_sorties[loc_i][0];
		if (last_previous_truck_node != current_node) {
			cout << "MISTAKE: truck route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: truck route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.truck_sorties[loc_i].size() > 1) {//if not waiting stage
			int loc_truck_previous_node = current_node;
			for (int loc_j = 1; loc_j < (int)solution.truck_sorties[loc_i].size(); loc_j++) {
				int truck_delivery_node = solution.truck_sorties[loc_i][loc_j];
				double distance_leg = dp.distance_metric_naive[loc_truck_previous_node][truck_delivery_node];
				//cout << "Truck leg of operation "<< loc_i << ": " << distance_leg <<'\n';
				if (loc_truck_previous_node != truck_delivery_node && loc_truck_previous_node < instance.n_nodes && truck_delivery_node < instance.n_nodes && current_real_damages[loc_truck_previous_node][truck_delivery_node] < 0) {
					cout << "MISTAKE: damaged edge is used ";
					cout << (loc_truck_previous_node);
					cout << "; ";
					cout << (truck_delivery_node);
					cout << '\n';
					mistake_reported = true;
					string loc_s = std::to_string(loc_truck_previous_node);
					mistake_text = mistake_text + "@check_sol: damaged edge outgoing from node ";
					mistake_text = mistake_text + loc_s;
					mistake_text = mistake_text + " is used; ";
				}
				truck_sortie_duration = truck_sortie_duration + distance_leg;
				if (truck_delivery_node > 0) {
					delivered_customers[truck_delivery_node] = true;
					visited_set.push_back(truck_delivery_node);
				};
				loc_truck_previous_node = truck_delivery_node;
			};
			last_previous_truck_node = loc_truck_previous_node;//initialization for the next stage
		};
		if (loc_i == 0) {
			current_node = curr_drone_node;
		}
		if ((last_previous_drone_node != current_node)) {
			cout << "MISTAKE: drone route is jumpy at the beginning of stage ";
			cout << loc_i << '\n';
			mistake_reported = true;
			mistake_text = mistake_text + "@truck_and_drone_model: drone route is jumpy at the beginning of stage " + to_string(loc_i) + ";";
		};

		if (solution.drone_sorties[loc_i].size() > 1) {//if drone is doing operation
			int loc_drone_previous_node = current_node;
			for (int loc_j = 0; loc_j < (int)solution.drone_sorties[loc_i].size(); loc_j++) {
				int drone_next_node = solution.drone_sorties[loc_i][loc_j];
				double distance_leg = dp.reopt_drone_distances[loc_drone_previous_node][drone_next_node];
				//for last delivery customer, if it is a reopt operation and we have no drone customer, then the truck does not need to wait for the drone, still need to simulate the way if edge failure
				if( loc_i< solution.n_stages-1 || (solution.last_delivered_customer_obj2>-1 && solution.last_delivered_customers_last_delivery_op.second >-1 )|| solution.last_delivered_customer_obj2==-1){
					drone_sortie_duration = drone_sortie_duration + distance_leg;
				} 
				if (std::find(solution.drone_customers[loc_i].begin(), solution.drone_customers[loc_i].end(), drone_next_node) != solution.drone_customers[loc_i].end()){
					delivered_customers[drone_next_node] = true;
					visited_set.push_back(drone_next_node);
				};
				loc_drone_previous_node = drone_next_node;
			};
			last_previous_drone_node = loc_drone_previous_node;//initialization for the next stage
			//check if drone only starts and lands on parking nodes
			//too many false positives
			/*
			int op_start_node = solution.drone_sorties[loc_i].front();
			int op_end_node = solution.drone_sorties[loc_i].back();
			if(op_start_node<instance.n_nodes && op_start_node==solution.truck_sorties[loc_i].front() && curr_drone_node!=op_start_node) {
				if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_start_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone starts on non parking node ";
					cout << op_start_node << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone starts on non parking node at the beginning of stage " + to_string(loc_i) + ";";
				}
			}
			if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), op_end_node) == instance.parking_nodes.end()) {
					cout << "MISTAKE: drone lands on non parking node ";
					cout << op_end_node  << '\n';
					mistake_reported = true;
					mistake_text = mistake_text + "@truck_and_drone_model: drone lands on non parking node at the end of stage " + to_string(loc_i) + ";";
				}
			*/
		} 
		else{
			last_previous_drone_node = solution.truck_sorties[loc_i].back();
		}

		//stage duration
		if (drone_sortie_duration > truck_sortie_duration) {
			loc_obj_value = loc_obj_value + drone_sortie_duration;
		}
		else {
			loc_obj_value = loc_obj_value + truck_sortie_duration;
		};
		//get dp infos for debugging
		std::sort(visited_set.begin(), visited_set.end());
		std::vector<int> v_intersection(0);
    	std::set_intersection(visited_set.begin(), visited_set.end(), customer_set.begin(), customer_set.end(),
                          std::back_inserter(v_intersection));
		int visited_set_id = dp.subsets[v_intersection.size()][v_intersection];
		std::stringstream ss;
		for(size_t i = 0; i <  v_intersection.size(); ++i){
					ss << ",";
					ss <<  v_intersection[i];
		}
		std::string set_string = ss.str();
		cout << to_string(loc_i)+":  Obj: "+to_string(loc_obj_value) + " ; Truck duration: " + to_string(truck_sortie_duration) + " ; Drone duration: " + to_string(drone_sortie_duration) + " ; Set: (" + set_string + ") ; Set id " +to_string(visited_set_id)<< '\n';
		if ((truck_sortie_duration > 0) && (drone_sortie_duration > 0) && ((drone_sortie_duration > instance.max_drone_sortie) || (truck_sortie_duration > instance.max_drone_sortie * instance.s_drone))) {
			cout << "MISTAKE: drone sortie exceed the max sortie duration of";
			cout << (instance.max_drone_sortie) << '\n';
			mistake_reported = true;
			string loc_s = to_string(loc_i);
			mistake_text = mistake_text + "@truck_and_drone_model: drone sortie at ";
			mistake_text = mistake_text + loc_s;
			mistake_text = mistake_text + "exceeds max sortie duration of ";
			mistake_text = mistake_text + to_string(instance.max_drone_sortie);
			mistake_text = mistake_text + "; ";
		};

	};
	//check max tour duration constraint
	if (loc_obj_value > cmax) {
		cout << "error in the surveillance op solution, the tour exceeds cmax!";
		cout << loc_obj_value;
		cout << "; ";
		mistake_reported = true;
		mistake_text = mistake_text + "@check_survop_surveillance: solution violates cmax constraint, the tour exceeds cmax ";
		string loc_s = std::to_string(cmax);
		mistake_text = mistake_text + loc_s+"! duration from model: ";
		loc_s = std::to_string(loc_obj_value);
		mistake_text = mistake_text + loc_s;
		mistake_text = mistake_text + "; ";
	};
	solution.mistake_reported = mistake_reported;
	solution.mistake_text = mistake_text;
	return mistake_reported;
}



void Model::dp_eval(int v, int w, int u, int index1, int index2)
{
    double z;
	if (dp.distance_metric_naive[u][w] < max_lim && dp.dtsp[v][u][index1] < max_lim) {
		z = dp.dtsp[v][u][index1] + dp.distance_metric_naive[u][w];
		//check if the customer w is served in the truck operation (only if index2>index1)
		if( index2!=index1 && std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end()) {
			z = z + service_time;
		}
		if (z < dp.dtsp[v][w][index2]) {
			dp.dtsp[v][w][index2] = z;
			dp.dtsp_pred[v][w][index2] = { u, index1 };
		}
	}
}


void Model::dp_make_ops_steiner_thread(int v) {
	//create all customer truck operations
	int index1 = 0;
	int index2 = 0;
	//make truck operations
	dp_make_tsp(v);
	if(v<instance.n_nodes) {
		//artificial operations needs always synchronisation
		dp.dtsp_op[v] = dp.dtsp[v];
	}
	//Drone OPS
	if(v<instance.n_nodes) {
		if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), v) != instance.parking_nodes.end()) {
			for (int i = 1; i < (int)dp.subsets.size(); i++) {
				for (auto it : dp.subsets[i]) {
					for (int w : instance.parking_nodes) {
						double z;
						double drone_dist;
						double truck_dist;
						std::set<int> setDS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
						if(service_time<=0.0) {
							setDS.erase(v);
						}
						//setDS.erase(w); // should be removed?
						for (int d : setDS) {
							std::set<int> setS_drone(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
							set<int> set_base = setS_drone;
							int erase_num = setS_drone.erase(d);
							std::vector<int> setSV_drone(setS_drone.begin(), setS_drone.end());
							if (erase_num > 0) {
								index1 = dp.subsets[i - 1].at(setSV_drone);
								index2 = dp.subsets[i].at(it.first);
								truck_dist = dp.dtsp[v][w][index1];
								if (dp.reopt_drone_distances[v][d] < max_lim && dp.reopt_drone_distances[d][w] < max_lim) {
									drone_dist = (dp.reopt_drone_distances[v][d] + dp.reopt_drone_distances[d][w]+service_time);
								}
								else {
									drone_dist = max_lim;
								}
								/* Not needed?!
								if (v == w && (int)setSV_drone.size() == 0 && areEqual(dp.dtsp[v][w][index1], max_lim)) {
									truck_dist = 0.0;
								}*/
								z = max(drone_dist, truck_dist);
								if (z < dp.dtsp_op[v][w][index2]) {
									dp.dtsp_op[v][w][index2] = z;
									dp.dtsp_drone[v][w][index2] = d;
								}
							}
						}
					}
				}
			}
		}
	}
	else {
		double z;
		double drone_dist;
		double truck_dist;
		for (int i = 0; i < (int)dp.subsets.size(); i++) {
			for (auto it : dp.subsets[i]) {
				for (int w : instance.parking_nodes) {
					if(curr_drone_customer>-1) {
						std::set<int> setDS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
						if (curr_drone_customer!=curr_truck_node){
							setDS.erase(curr_truck_node);
						}
						if(service_time<=0.0) {
							setDS.erase(v);
						}
						//setDS.erase(w); // should be removed?
						for (int d : setDS) {
							std::set<int> setS_drone(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
							set<int> set_base = setS_drone;
							int erase_num = setS_drone.erase(d);
							std::vector<int> setSV_drone(setS_drone.begin(), setS_drone.end());
							if (erase_num > 0) {
								index1 = dp.subsets[i - 1].at(setSV_drone);
								index2 = dp.subsets[i].at(it.first);
								truck_dist = dp.dtsp[curr_truck_node][w][index1];
								if (dp.reopt_drone_distances[curr_drone_node][d] < max_lim && dp.reopt_drone_distances[d][w] < max_lim) {
									drone_dist = (dp.reopt_drone_distances[curr_drone_node][d] + dp.reopt_drone_distances[d][w]+service_time);
								}
								else {
									drone_dist = max_lim;
								}
								/* Not needed?!
								if (v == w && (int)setSV_drone.size() == 0 && areEqual(dp.dtsp[v][w][index1], max_lim)) {
									truck_dist = 0.0;
								}*/
								z = max(drone_dist, truck_dist);
								if (z < dp.dtsp_op[v][w][index2]) {
									dp.dtsp_op[v][w][index2] = z;
									dp.dtsp_drone[v][w][index2] = d;
								}
							}
						}
					}
					else{
						//operations without drone customer
						//also calculate in case the drone needs to replan the truck/drone operation by dropping the customer
						index2 = dp.subsets[i].at(it.first);
						truck_dist = dp.dtsp[curr_truck_node][w][index2];
						if (dp.reopt_drone_distances[curr_drone_node][w] < max_lim) {
							//no drone customer
							drone_dist = dp.reopt_drone_distances[curr_drone_node][w];
						}
						else {
							drone_dist = max_lim;
						}
						z = max(drone_dist, truck_dist);
						if (z < dp.dtsp_op[v][w][index2]) {
							dp.dtsp_op[v][w][index2] = z;
							dp.dtsp_drone[v][w][index2] = -1;
						}
					}
				}
			}
		}
	}
}

void Model::dp_make_ops_steiner_last_delivery_thread(int v)
{
	//create all customer truck operations
	int index1 = 0;
	int index2 = 0;
	//make truck operations
	dp_make_tsp(v);
	if(v<instance.n_nodes) {
		//artificial operations needs always synchronisation
		dp.dtsp_op[v] = dp.dtsp[v];
		dp.dtsp_op_last_delivery[v] = dp.dtsp[v];
		dp.dtsp_op[v] = dp.dtsp[v];
	}

	//Drone OPS
	if(v<instance.n_nodes) {
		if(std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), v) != instance.parking_nodes.end()) {
			for (int i = 0; i < (int)dp.subsets.size(); i++) {
				for (auto it : dp.subsets[i]) {
					for (int w : instance.parking_nodes) {
						double z;
						double drone_dist;
						double drone_dist_last_delivery;
						double truck_dist;
						int last_truck_customer;
						std::set<int> setDS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
						if(service_time<=0.0) {
							setDS.erase(v);
						}
						//setDS.erase(w);
						for (int d : setDS) {
							std::set<int> setS_drone(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
							set<int> set_base = setS_drone;
							int erase_num = setS_drone.erase(d);
							std::vector<int> setSV_drone(setS_drone.begin(), setS_drone.end());
							if (erase_num > 0) {
								index1 = dp.subsets[i - 1].at(setSV_drone);
								index2 = dp.subsets[i].at(it.first);
								truck_dist = dp.dtsp[v][w][index1];
								if (dp.reopt_drone_distances[v][d] < max_lim && dp.reopt_drone_distances[d][w] < max_lim) {
									drone_dist = (dp.reopt_drone_distances[v][d] + dp.reopt_drone_distances[d][w]+service_time);
								}
								else {
									drone_dist = max_lim;
								}
								z = max(drone_dist, truck_dist);
								if (z < dp.dtsp_op[v][w][index2]) {
									dp.dtsp_op[v][w][index2] = z;
									dp.dtsp_drone[v][w][index2] = d;
								}
								//create final operations
								//Only truck customer are already in dtsp
								if(index1==0 && std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), v) != instance.parking_nodes.end()){
									//Case 1, only 1 drone customer, truck waits
									truck_dist = dp.dtsp[v][v][index1];
									last_truck_customer = v;
								} 
								else{
									//Case 2, drone customer and truck customer
									truck_dist = dp.dtsp[v][w][index1];
									last_truck_customer = w;
								}
								//calculate drone distance
								drone_dist_last_delivery = dp.reopt_drone_distances[v][d]+service_time;
								//max z and last customer assignment
								z = max(drone_dist_last_delivery,truck_dist);
								if (z < dp.dtsp_op_last_delivery[v][last_truck_customer][index2]) {
									dp.dtsp_op_last_delivery[v][last_truck_customer][index2] = z;
									dp.dtsp_last_delivery_nodes[v][last_truck_customer][index2] ={last_truck_customer,d};
								}
							}
						}
					}
				}
			}
		}
	}
	else {
		double z;
		double drone_dist;
		double drone_dist_last_delivery;
		double truck_dist;
		int last_truck_customer;
		for (int i = 0; i < (int)dp.subsets.size(); i++) {
			for (auto it : dp.subsets[i]) {
				for (int w : instance.parking_nodes) {
					if(curr_drone_customer>-1) {
						std::set<int> setDS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
						if (curr_drone_customer!=curr_truck_node){
							setDS.erase(curr_truck_node);
						}
						if(service_time<=0.0) {
							setDS.erase(v);
						}
						//setDS.erase(w);
						for (int d : setDS) {
							std::set<int> setS_drone(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
							set<int> set_base = setS_drone;
							int erase_num = setS_drone.erase(d);
							std::vector<int> setSV_drone(setS_drone.begin(), setS_drone.end());
							if (erase_num > 0) {
								index1 = dp.subsets[i - 1].at(setSV_drone);
								index2 = dp.subsets[i].at(it.first);
								truck_dist = dp.dtsp[curr_truck_node][w][index1];
								if (dp.reopt_drone_distances[curr_drone_node][d] < max_lim && dp.reopt_drone_distances[d][w] < max_lim) {
									drone_dist = (dp.reopt_drone_distances[curr_drone_node][d] + dp.reopt_drone_distances[d][w]+service_time);
								}
								else {
									drone_dist = max_lim;
								}
								z = max(drone_dist, truck_dist);
								if (z < dp.dtsp_op[v][w][index2]) {
									dp.dtsp_op[v][w][index2] = z;
									dp.dtsp_drone[v][w][index2] = d;
								}
								//create final operations
								//Only truck customer are already in dtsp
								if(index1==0 && std::find(instance.parking_nodes.begin(), instance.parking_nodes.end(), curr_truck_node) != instance.parking_nodes.end()){
									//Case 1, only 1 drone customer, truck waits
									truck_dist = dp.dtsp[curr_truck_node][curr_truck_node][index1];
									last_truck_customer = curr_truck_node;
								} 
								else{
									//Case 2, drone customer and truck customer
									truck_dist = dp.dtsp[curr_truck_node][w][index1];
									last_truck_customer = w;
								}
								//calculate drone distance
								drone_dist_last_delivery = dp.reopt_drone_distances[curr_drone_node][d]+service_time;
								//max z and last customer assignment
								z = max(drone_dist_last_delivery,truck_dist);
								if (z < dp.dtsp_op_last_delivery[v][last_truck_customer][index2]) {
									dp.dtsp_op_last_delivery[v][last_truck_customer][index2] = z;
									dp.dtsp_last_delivery_nodes[v][last_truck_customer][index2] ={last_truck_customer,d};
								}
							}
						}
					}
					else{
						//operations without drone customer
						//also calculate in case the drone needs to replan the truck/drone operation by dropping the customer
						index2 = dp.subsets[i].at(it.first);
						if (index2==0 && std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end()) {
							vector<int> temp_vec(1,w);
							index2 = dp.subsets[1].at(temp_vec);
						}
						truck_dist = dp.dtsp[curr_truck_node][w][index2];
						if (dp.reopt_drone_distances[curr_drone_node][w] < max_lim) {
							//no drone customer
							drone_dist = dp.reopt_drone_distances[curr_drone_node][w];
						}
						else {
							drone_dist = max_lim;
						}
						z = max(drone_dist, truck_dist);
						if (z < dp.dtsp_op[v][w][index2]) {
							dp.dtsp_op[v][w][index2] = z;
							dp.dtsp_drone[v][w][index2] = -1;
						}
						//create final operations
						//Only if truck can try to reach the last, customer, otherwise they need to synchronize before doing operation
						//max z and last customer assignment
						z = truck_dist;
						last_truck_customer = w;
						if (z < dp.dtsp_op_last_delivery[v][last_truck_customer][index2]) {
							dp.dtsp_op_last_delivery[v][last_truck_customer][index2] = z;
							dp.dtsp_last_delivery_nodes[v][last_truck_customer][index2] ={last_truck_customer,-1};
						}
					}
				}
			}
		}

	}
}


void Model::dp_make_tsp(int v)
{
	//create all customer truck operations
	int index1 = 0;
	int index2 = 0;
	for (int i = 0; i < (int)dp.subsets.size(); i++) {
		for (auto it : dp.subsets[i]) {		//2^T
			index2 = dp.subsets[i].at(it.first);
			for (int w =0; w<instance.n_nodes;w++) {
				for (int u = 0; u < instance.n_nodes; u++) {		//T
					std::set<int> setS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
					if (std::find(it.first.begin(), it.first.end(), w) != it.first.end()) {
						setS.erase(w);
					}
					std::vector<int> setSV(setS.begin(), setS.end());
					index1 = dp.subsets[setSV.size()].at(setSV);
					dp_eval(v, w, u, index1, index2);
				}
			}
			for (int loc_v = 0; loc_v < instance.n_nodes; loc_v++) {
				for (int w = 0; w < instance.n_nodes; w++) {
					for (int u = 0; u < instance.n_nodes; u++) {	
						dp_eval(v, w, u, index2, index2);
					}
				}
			}
		}
	}
}

/*
void Model::dp_make_tsp_for_surveillance(int v, set<int> truck_customers)
{
	//create all customer truck operations
	int index1 = 0;
	int index2 = 0;
	for (int i = 0; i < (int)dp.subsets.size(); i++) {
		for (auto it : dp.subsets[i]) {		//2^T
			index2 = dp.subsets[i].at(it.first);
			for (int w : it.first) {
				for (int u : truck_customers) {		//T
					std::set<int> setS(std::make_move_iterator(it.first.begin()), std::make_move_iterator(it.first.end()));
					if (std::find(it.first.begin(), it.first.end(), w) != it.first.end()) {
						setS.erase(w);
					}
					std::vector<int> setSV(setS.begin(), setS.end());
					index1 = dp.subsets[setSV.size()].at(setSV);
					dp_eval_tsp_surveillance(w, u, index1, index2);
				}
			}
			for (int loc_v = 0; loc_v < instance.n_nodes; loc_v++) {
				for (int w = 0; w < instance.n_nodes; w++) {
					for (int u = 0; u < instance.n_nodes; u++) {	
						dp_eval_tsp_surveillance(w, u, index2, index2);
					}
				}
			}
		}
	}
}
*/

vector<vector<int>> Model::get_all_subsets(vector<int>& base_set) {
	std::vector<vector<int>>all_subsets(0);
	vector<int> subset;
	for (int loc_i = 0; loc_i < (int)pow(2, base_set.size()); loc_i++)
	{
		subset.resize(0);
		for (int loc_j = 0; loc_j < (int)base_set.size(); loc_j++)
		{
			if (loc_i & (1 << loc_j))
				subset.push_back(base_set[loc_j]);
		}
		all_subsets.push_back(subset);
	}
	return all_subsets;
}


void Model::solve_operations(int settings) {
	std::vector<std::thread> threads;
	threads.resize(0);
	//create tsp tour for surveillance first
	std::vector<int> thread_start_nodes(instance.n_nodes); //
	std::iota(std::begin(thread_start_nodes), std::end(thread_start_nodes), 0); //get all nodes to the vector
	for (int v : thread_start_nodes) {
		switch(settings){
			case 1:
				threads.push_back(std::thread(&Model::dp_make_ops_steiner_thread, this, v));
				break;
			case 2:
				threads.push_back(std::thread(&Model::dp_make_ops_steiner_last_delivery_thread, this, v));
				break;
		} 
	} 
	for (auto& th : threads) {
		th.join();
	}
	//solve for artificial operations at the end
	if(dp.artificial_nodes>0 && curr_drone_node!=curr_truck_node) {
		int v = instance.n_nodes-1+dp.artificial_nodes;
		switch(settings){
			case 1:
				dp_make_ops_steiner_thread(v);
				break;
			case 2:
				dp_make_ops_steiner_last_delivery_thread(v);
				break;
		} 
	}
}


void Model::init_distances(double penalty_factor) {
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		dp.dtsp[loc_i][loc_i][0] = 0;
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			if (edge_status[loc_i][loc_j] < 0) {
				dp.distance_metric_naive[loc_i][loc_j] = max_lim;
			}
			else {
				if(manhattan_truck==true){
					dp.distance_metric_naive[loc_i][loc_j] = instance.manhattan_distance_matrix[loc_i][loc_j];
				}
				else{
					dp.distance_metric_naive[loc_i][loc_j] = instance.euclidean_distance_matrix[loc_i][loc_j];
				}
			};
		};
		dp.distance_metric_naive[loc_i][loc_i] = 0;
	};
	//default penalty_factor == 0
	if(penalty_factor>1.0) {
		for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
			for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
				if(arc_status_after_surveillance[loc_i][loc_j] ==0) {
					dp.distance_metric_naive[loc_i][loc_j] = dp.distance_metric_naive[loc_i][loc_j]*penalty_factor;
				}
			}
		}
	}
	for (int loc_j = 1; loc_j < instance.n_nodes; loc_j++) {
		if (edge_status[0][loc_j] > 0) {
			conservative = false;
			break;
		}		
	}		
}


void Model::init_subsets() {

	vector<int> subset;//help vector
	for (int loc_i = 0; loc_i < pow(2, customer_n_nodes); loc_i++)
	{
		subset.resize(0);
		for (int loc_j = 0; loc_j < customer_n_nodes; loc_j++)
		{
			if (loc_i & (1 << loc_j))
				subset.push_back(customer_set[loc_j]);
		}
		dp.subsets[subset.size()][subset] = loc_i;
		dp.subsets_by_id.insert({loc_i,subset});
	}
}

void Model::init_dp_table(bool artificial_node) {
	//table init
	vector<int> active_nodes(instance.n_nodes);
	std::iota(active_nodes.begin(), active_nodes.end(), 0);
	if(artificial_node==true) {
		active_nodes.push_back(instance.n_nodes+dp.artificial_nodes-1);
	}
	for (int v : active_nodes) {
		for (int w: active_nodes) {
			if (dp.distance_metric_naive[v][w] < max_lim) {
				vector<int>first(0);
				int idx = dp.subsets[0][first]; //zero set if w no customer node
				if (std::find(customer_set.begin(), customer_set.end(), w) != customer_set.end()) {
					first.resize(1, w);
					idx = dp.subsets[1].at(first);
				}
				dp.dtsp[v][w][idx] = dp.distance_metric_naive[v][w];
				dp.dtsp[v][w][0] = dp.distance_metric_naive[v][w];
				if(idx>0) {
					dp.dtsp[v][w][idx]  = dp.dtsp[v][w][idx] + service_time;
				}
				if (v != w) {
					dp.dtsp_pred[v][w][idx] = { v,0 };
					dp.dtsp_pred[v][w][0] = { v,0 };
				}
			}
		}
	}
}


void Model::init_dp_struct(double penalty_factor, bool artificial_node)
{
	init_distances(penalty_factor);
	init_subsets();
	//all_pair_shortest_path_init();
	init_dp_table(artificial_node);
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
	
}

void Model::conservative_calculation(int settings) {
	//settings: 1= completion time objective
	//settings: 2= last delivery objective
	double solution_val = 0;
	for (int i = 0; i < Ledges_struct.n_Ledges; i++) {
		solution_val = solution_val + Ledges_struct.list_of_Ledges[i];
	}
	model_solution.obj_value = solution_val;
	std::vector<double>::iterator max_ledge;
	max_ledge = std::max_element(Ledges_struct.list_of_Ledges.begin(), Ledges_struct.list_of_Ledges.end());
	if(settings==2){
		model_solution.obj_value = solution_val - *max_ledge/2 +service_time/2;
		int max_elem_index = std::distance(Ledges_struct.list_of_Ledges.begin(), max_ledge);
		Ledges_struct.list_of_Ledges.push_back(*max_ledge);
		Ledges_struct.list_of_Ledges.erase(Ledges_struct.list_of_Ledges.begin()+max_elem_index);
		Ledges_struct.Ledges_dronenode.push_back(Ledges_struct.Ledges_dronenode[max_elem_index]);
		Ledges_struct.Ledges_dronenode.erase(Ledges_struct.Ledges_dronenode.begin()+max_elem_index);
	} 
	//initialize short_output.list_truck_edges
	model_solution.list_truck_edges.resize(0);
	model_solution.list_truck_edges = vector<vector<int>>(instance.n_nodes, vector<int>(instance.n_nodes, 0));
	model_solution.list_truck_edges[0][0] = 1;

	//initialize
	model_solution.n_stages = 0;
	model_solution.truck_sorties.resize(0);
	model_solution.drone_sorties.resize(0);
	vector<int> last_drone_op(0);
	vector<int>help_vector = vector<int>(instance.n_nodes);
	for (int next_ledge_num = 0; next_ledge_num < Ledges_struct.n_Ledges; next_ledge_num++) {
		model_solution.truck_sorties.push_back(vector<int>(1,0));
		model_solution.drone_customers.push_back(vector<int>(1,Ledges_struct.Ledges_dronenode[next_ledge_num]));
		int third_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].front();
		int second_node = Ledges_struct.Ledges_dronenode[next_ledge_num];
		int first_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].back();
		vector<int> drone_op(0);
		//backtrack for the final operation only the way to the customer for the last delivery objective
		if(settings==2 && next_ledge_num==Ledges_struct.n_Ledges-1){
				drone_op = backtrack_path(first_node, second_node, instance.drone_fromtriple_previousnode, drone_op);
		}
		else{
				drone_op = backtrack_path(first_node, second_node, instance.drone_fromtriple_previousnode, drone_op);
				drone_op = backtrack_path(second_node, third_node, instance.drone_fromtriple_previousnode, drone_op);
		} 
		
		model_solution.drone_sorties.push_back(drone_op);
		model_solution.n_stages = model_solution.n_stages + 1;
	}

	check_solution(model_solution, edge_status);
}

void Model::add_conservative_Loops() {
	for (int loc_i = 0; loc_i < 1; loc_i++) {
		for (int loc_j : customer_set) {
			Ledges_struct.add_Ledge();
			vector<int>Ledges_hilf_vector = vector<int>(0);
			vector<int>Loops_hilf_vector = vector<int>(1, 0);
			Ledges_struct.Ledges_from_end_to_start.push_back(Ledges_hilf_vector);
			Ledges_struct.Ledges_from_end_to_start[Ledges_struct.n_Ledges - 1].push_back(loc_i);
			Ledges_struct.Ledges_from_end_to_start[Ledges_struct.n_Ledges - 1].push_back(loc_i);
			Ledges_struct.incoming_Ledges[loc_i].push_back(Ledges_struct.n_Ledges - 1);
			Ledges_struct.outgoing_Ledges[loc_i].push_back(Ledges_struct.n_Ledges - 1);
			Ledges_struct.nodes_on_Ledge[loc_j].push_back(Ledges_struct.n_Ledges - 1);
			Ledges_struct.Ledges_dronenode.push_back(loc_j);
			Ledges_struct.list_of_Ledges.push_back((instance.drone_distances[loc_i][loc_j] + instance.drone_distances[loc_j][loc_i]+service_time));
		}
	}
}
//Helper function to write all operations into a csv file
void Model::write_Ledges() {


	cout << "Truck ledges\n";
	std::ofstream OFile;
	string locfilename = "";
	//locfilename = locfilename + globparameters.path_to_folder;
	locfilename = locfilename + to_string(instance.ins_number) + "_truck_ledges.csv";
	OFile.open(locfilename, std::ios_base::app);
	if (OFile.is_open()) {
		for (int i = 0; i < (int)Ledges_struct.Ledges_from_end_to_start.size(); i++)
		{
			for (int j = 0; j < (int)Ledges_struct.Ledges_from_end_to_start[i].size(); j++)
			{
				OFile << Ledges_struct.Ledges_from_end_to_start[i][j];
				if (j < (int)Ledges_struct.Ledges_from_end_to_start[i].size() - 1) {
					OFile << ';';
				}
			}
			OFile << '\n';;
		}
	}
	OFile.close();

	cout << "Drone ledges\n";
	locfilename = "";
	//locfilename = locfilename + globparameters.path_to_folder;
	locfilename = locfilename + to_string(instance.ins_number) + "_drone_ledge_nodes.csv";
	OFile.open(locfilename, std::ios_base::app);
	if (OFile.is_open()) {
		for (int i = 0; i < (int)Ledges_struct.Ledges_dronenode.size(); i++)
		{
			string drone_node_and_sizes = to_string(Ledges_struct.Ledges_dronenode[i]) + ";" + write_number(Ledges_struct.list_of_Ledges[i]);
			OFile << drone_node_and_sizes << '\n';
		}
	}
	OFile.close();
}


void Model::write_solution() {

	//int n_nodes = dp.n_nodes + dp.artificial_nodes;

	//initialize short_output.list_truck_edges
	model_solution.list_truck_edges.resize(0);
	model_solution.drone_sorties.resize(0);
	model_solution.truck_sorties.resize(0);
	vector<int>hilf_vector_sorties = vector<int>(0);
	model_solution.list_truck_edges = vector<vector<int>>(instance.n_nodes, vector<int>(instance.n_nodes));
	model_solution.list_of_op_customers = Ledges_struct.Ledges_customers;
	//reversal needed because from end to start
	std::reverse(model_solution.list_of_op_customers.begin(), model_solution.list_of_op_customers.end());
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
		/*
		if (from_node >= instance.n_nodes) {
			from_node = 0;
		}
		if (curr_drone_node == curr_truck_node && curr_truck_node > 0 && from_node==curr_truck_node) {
			Ledges_struct.Ledges_from_end_to_start[loc_i].push_back(0);
			from_node = 0;
			Ledges_struct.reopt_ct_ledges.push_back(loc_i);
		}
		*/
		loc_list_ledges[from_node][0] = loc_list_ledges[from_node][0] + 1;
		loc_list_ledges[from_node].push_back(loc_i);
		loc_list_visited_ledges[from_node].push_back(0);
		total_n_ledges = total_n_ledges + 1;
	};//for each Ledge
	vector<int>sequence_of_ledges_to_visit = vector<int>(total_n_ledges, -1);
	//if (n_replan_reopt > 0) {
	//	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//}

	int current_comb_node = -1;
	int n_visited_Ledges = 0;
	int number_cycles = 0;
	vector<int>zero_vector = vector<int>(1, -1);
	while (n_visited_Ledges < total_n_ledges) {
		//find the starting node
		number_cycles = number_cycles + 1;
		list_of_cycles.push_back(zero_vector);
		if (current_comb_node == -1) {
			current_comb_node = 0;
		}
		else {
			current_comb_node = -1;
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				if ((loc_list_visited_ledges[loc_i][0] > 0) && (loc_list_visited_ledges[loc_i][0] < loc_list_ledges[loc_i][0])) {//if visited, but not all the outgoing Ledges
					current_comb_node = loc_i;
					loc_i = instance.n_nodes + 5;
				};
			};
			if (current_comb_node <= -1) {
				cout << "MISTAKE: in reporting Ledges";
				mistake_reported = true;
				mistake_text = mistake_text + "@truck_and_drone_model: MISTAKE: in reporting Ledges; ";
				throw std::invalid_argument("there is no cycle in euler tour!");
			};
		};//find comb node= first node of the cycle
		//notate a cycle starting and ending in this node
		int current_node = current_comb_node;
		int selected_Ledge = 0;
		while (selected_Ledge > -1) {
			selected_Ledge = -1;
			for (int loc_i = 0; loc_i < loc_list_ledges[current_node][0]; loc_i++) {
				if (loc_list_visited_ledges[current_node][loc_i + 1] < 1) {//if this Ledge has not been visited yet
					selected_Ledge = loc_list_ledges[current_node][loc_i + 1];
					loc_list_visited_ledges[current_node][0] = loc_list_visited_ledges[current_node][0] + 1;
					loc_list_visited_ledges[current_node][loc_i + 1] = 1;
					loc_i = loc_list_ledges[current_node][0] + 5;
				};
			};//for each outgoing visited Ledge
			//save the Ledge into the current cycle
			if (selected_Ledge > -1) {
				list_of_cycles[number_cycles - 1].push_back(selected_Ledge);
				int outgoing_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][Ledges_struct.Ledges_from_end_to_start[selected_Ledge].size() - 1];
				list_of_cycles_from_nodes[outgoing_node][0] = list_of_cycles_from_nodes[outgoing_node][0] + 1;
				list_of_cycles_from_nodes[outgoing_node].push_back(number_cycles - 1);
				n_visited_Ledges = n_visited_Ledges + 1;

				//96 0
				//966 

				//find the next node
				current_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][0];
			};
		};//while the cycle is not to the end.
	};
	//determine the sequence of ledges

	int active_cycle = 0;
	n_visited_Ledges = 0;
	int current_number_active_cycles = 0;//for transcripting the solution.
	int start_position = 0;
	vector<int>entry_position_to_the_cycle = vector<int>(number_cycles, -1);
	vector<int>list_of_opened_cycles = vector<int>(number_cycles, -1);
	entry_position_to_the_cycle[active_cycle] = start_position;
	current_number_active_cycles = 1;
	list_of_opened_cycles[current_number_active_cycles - 1] = active_cycle;
	//start from cycle 0, if it "crosses" another not yet active cycle --> proceed with this cycle
	while (start_position > -1) {
		//notate the Ledge
		list_of_cycles[active_cycle][0] = start_position;
		sequence_of_ledges_to_visit[n_visited_Ledges] = list_of_cycles[active_cycle][start_position + 1];
		int selected_Ledge = list_of_cycles[active_cycle][start_position + 1];
		n_visited_Ledges = n_visited_Ledges + 1;
		//find a new position
		int next_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][0];
		int next_position_if_in_cycle = start_position + 1;
		if (next_position_if_in_cycle >= ((int)list_of_cycles[active_cycle].size() - 1)) {
			next_position_if_in_cycle = 0;
		};
		//check all the cycles that should be closed in this node
		while (next_position_if_in_cycle == entry_position_to_the_cycle[active_cycle]) {
			list_of_cycles[active_cycle][0] = -2;//close the cycle
			current_number_active_cycles = current_number_active_cycles - 1;
			if (current_number_active_cycles > 0) {
				active_cycle = list_of_opened_cycles[current_number_active_cycles - 1];
				next_position_if_in_cycle = list_of_cycles[active_cycle][0] + 1;
				if (next_position_if_in_cycle >= ((int)list_of_cycles[active_cycle].size() - 1)) {
					next_position_if_in_cycle = 0;
				};
			}
			else {
				next_position_if_in_cycle = -1;
			};
		};
		start_position = -1;//initialize
		//check if there are new not opened cycles starting in this node
		for (int loc_c = 0; loc_c < list_of_cycles_from_nodes[next_node][0]; loc_c++) {
			int this_cycle = list_of_cycles_from_nodes[next_node][loc_c + 1];
			if (list_of_cycles[this_cycle][0] == -1) {//if not yet opened
				//find its starting position
				for (int loc_p = 0; loc_p < ((int)list_of_cycles[active_cycle].size() - 1); loc_p++) {
					active_cycle = this_cycle;
					current_number_active_cycles = current_number_active_cycles + 1;
					list_of_opened_cycles[current_number_active_cycles - 1] = active_cycle;
					int this_ledge = list_of_cycles[active_cycle][loc_p + 1];
					int first_node_of_Ledge = Ledges_struct.Ledges_from_end_to_start[this_ledge][Ledges_struct.Ledges_from_end_to_start[this_ledge].size() - 1];
					if (first_node_of_Ledge == next_node) {
						start_position = loc_p;//start position
						entry_position_to_the_cycle[active_cycle] = start_position;//save the start position for the cycle
						loc_p = (list_of_cycles[active_cycle].size() - 1) + 5;//exit the loop

					};
				};
				loc_c = list_of_cycles_from_nodes[next_node][0] + 5;
			};//if not yet opened
		};//for each candidate for a not opened cycle

		//if no start_position found (no opened cycle)==> --> proceed
		if ((start_position < 0) && (list_of_cycles[active_cycle][0] != -2)) {
			start_position = next_position_if_in_cycle;
		};

	};
	if (n_visited_Ledges != total_n_ledges) {
		cout << "ERROR: n_visited_Ledges != total_n_ledges" << '\n';
		mistake_reported = true;
		mistake_text = mistake_text + "@truck_and_drone_model: MISTAKE: n_visited_Ledges != total_n_ledges; ";
	};
	/*
	if(simulation_setting==2){
		sequence_of_ledges_to_visit.back()=sequence_of_ledges_to_visit.size()-1;
		std::reverse(std::begin(sequence_of_ledges_to_visit), std::end(sequence_of_ledges_to_visit));
	} */

	model_solution.n_stages = 0;
	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
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
		for (size_t loc_j = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1; loc_j-- > 0;){
			int goal_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][loc_j];
			model_solution.truck_sorties.back() = backtrack_path(start_node, goal_node, dp.previous_node, model_solution.truck_sorties.back());
			start_node = goal_node;
		}
		*/
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
	//check the orientation of a symmetric solution is consistent
	/*
	if(model_solution.truck_sorties.size()>1) {
		std::vector<std::vector<int>> copy_truck_sorties = model_solution.truck_sorties;
		std::vector<std::vector<int>> copy_drone_sorties = model_solution.drone_sorties;
		while(true) {
			int sum_op_first = 0;
			int sum_op_last = 0;
			if(copy_truck_sorties.size()<=1){
				break;
			}
			if((checkFirstAndLastEqual(copy_truck_sorties)==false) || (checkFirstAndLastEqual(copy_drone_sorties)==false)){
				sum_op_first = get_op_weight(copy_truck_sorties.front())+get_op_weight(copy_drone_sorties.front());
				sum_op_last = get_op_weight(copy_truck_sorties.back())+get_op_weight(copy_drone_sorties.back());
				if(sum_op_last>sum_op_first){
					reverseAllOperations(model_solution.truck_sorties);
					reverseAllOperations(model_solution.drone_sorties);
					reverseAllOperations(model_solution.list_truck_edges);
					std::reverse(model_solution.drone_customers.begin(), model_solution.drone_customers.end());
					std::reverse(model_solution.list_of_op_customers.begin(),model_solution.list_of_op_customers.end());				
				}
				break;
			}
			copy_truck_sorties = std::vector<std::vector<int>>(copy_truck_sorties.end() + 1, copy_truck_sorties.end()-1);
			copy_drone_sorties = std::vector<std::vector<int>>(copy_drone_sorties.end() + 1, copy_drone_sorties.end()-1);
		};
	}
	*/
}

// Function to reverse all vectors within a vector of vectors
void Model::reverseAllOperations(std::vector<std::vector<int>>& vec) {
    // Reverse each inner vector
    for (auto& innerVec : vec) {
        std::reverse(innerVec.begin(), innerVec.end());
    }

    // Reverse the outer vector
    std::reverse(vec.begin(), vec.end());
}

int Model::get_op_weight(const std::vector<int>& operations_vec) {
	int sum = 0;
	sum = std::accumulate(operations_vec.begin(), operations_vec.end(), sum);
	return sum;
}

void Model::write_solution_last_delivery() {

	//int n_nodes = dp.n_nodes + dp.artificial_nodes;

	//initialize short_output.list_truck_edges
	model_solution.list_truck_edges.resize(0);
	model_solution.drone_sorties.resize(0);
	model_solution.truck_sorties.resize(0);
	vector<int>hilf_vector_sorties = vector<int>(0);
	model_solution.list_truck_edges = vector<vector<int>>(instance.n_nodes, vector<int>(instance.n_nodes));
	model_solution.list_of_op_customers = Ledges_struct.Ledges_customers;
	//reversal needed because from end to start
	std::reverse(model_solution.list_of_op_customers.begin(), model_solution.list_of_op_customers.end());
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
	//fix artificial ledge for last delivery check
	//we need to remove the dummy ledge after the checks!
	vector<int> temp_vec_stored = Ledges_struct.Ledges_from_end_to_start[0];
	if(Ledges_struct.Ledges_from_end_to_start.size()>0 && Ledges_struct.Ledges_from_end_to_start.front().size()){
		Ledges_struct.Ledges_from_end_to_start[0].insert(Ledges_struct.Ledges_from_end_to_start[0].begin(), Ledges_struct.Ledges_from_end_to_start.back().back());
	} 
	for (int loc_i : extracted_Ledges_num) {
		int from_node = Ledges_struct.Ledges_from_end_to_start[loc_i].back();
		/*
		if (from_node >= instance.n_nodes) {
			from_node = 0;
		}
		if (curr_drone_node == curr_truck_node && curr_truck_node > 0 && from_node==curr_truck_node) {
			Ledges_struct.Ledges_from_end_to_start[loc_i].push_back(0);
			from_node = 0;
			Ledges_struct.reopt_ct_ledges.push_back(loc_i);
		}
		*/
		loc_list_ledges[from_node][0] = loc_list_ledges[from_node][0] + 1;
		loc_list_ledges[from_node].push_back(loc_i);
		loc_list_visited_ledges[from_node].push_back(0);
		total_n_ledges = total_n_ledges + 1;
	};//for each Ledge
	vector<int>sequence_of_ledges_to_visit = vector<int>(total_n_ledges, -1);
	//if (n_replan_reopt > 0) {
	//	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
	//}

	int current_comb_node = -1;
	int n_visited_Ledges = 0;
	int number_cycles = 0;
	vector<int>zero_vector = vector<int>(1, -1);
	while (n_visited_Ledges < total_n_ledges) {
		//find the starting node
		number_cycles = number_cycles + 1;
		list_of_cycles.push_back(zero_vector);
		if (current_comb_node == -1) {
			current_comb_node = 0;
		}
		else {
			current_comb_node = -1;
			for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
				if ((loc_list_visited_ledges[loc_i][0] > 0) && (loc_list_visited_ledges[loc_i][0] < loc_list_ledges[loc_i][0])) {//if visited, but not all the outgoing Ledges
					current_comb_node = loc_i;
					loc_i = instance.n_nodes + 5;
				};
			};
			if (current_comb_node <= -1) {
				cout << "MISTAKE: in reporting Ledges";
				mistake_reported = true;
				mistake_text = mistake_text + "@truck_and_drone_model: MISTAKE: in reporting Ledges; ";
				throw std::invalid_argument("there is no cycle in euler tour!");
			};
		};//find comb node= first node of the cycle
		//notate a cycle starting and ending in this node
		int current_node = current_comb_node;
		int selected_Ledge = 0;
		while (selected_Ledge > -1) {
			selected_Ledge = -1;
			for (int loc_i = 0; loc_i < loc_list_ledges[current_node][0]; loc_i++) {
				if (loc_list_visited_ledges[current_node][loc_i + 1] < 1) {//if this Ledge has not been visited yet
					selected_Ledge = loc_list_ledges[current_node][loc_i + 1];
					loc_list_visited_ledges[current_node][0] = loc_list_visited_ledges[current_node][0] + 1;
					loc_list_visited_ledges[current_node][loc_i + 1] = 1;
					loc_i = loc_list_ledges[current_node][0] + 5;
				};
			};//for each outgoing visited Ledge
			//save the Ledge into the current cycle
			if (selected_Ledge > -1) {
				list_of_cycles[number_cycles - 1].push_back(selected_Ledge);
				int outgoing_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][Ledges_struct.Ledges_from_end_to_start[selected_Ledge].size() - 1];
				list_of_cycles_from_nodes[outgoing_node][0] = list_of_cycles_from_nodes[outgoing_node][0] + 1;
				list_of_cycles_from_nodes[outgoing_node].push_back(number_cycles - 1);
				n_visited_Ledges = n_visited_Ledges + 1;

				//96 0
				//966 

				//find the next node
				current_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][0];
			};
		};//while the cycle is not to the end.
	};
	//determine the sequence of ledges

	int active_cycle = 0;
	n_visited_Ledges = 0;
	int current_number_active_cycles = 0;//for transcripting the solution.
	int start_position = 0;
	vector<int>entry_position_to_the_cycle = vector<int>(number_cycles, -1);
	vector<int>list_of_opened_cycles = vector<int>(number_cycles, -1);
	entry_position_to_the_cycle[active_cycle] = start_position;
	current_number_active_cycles = 1;
	list_of_opened_cycles[current_number_active_cycles - 1] = active_cycle;
	//start from cycle 0, if it "crosses" another not yet active cycle --> proceed with this cycle
	while (start_position > -1) {
		//notate the Ledge
		list_of_cycles[active_cycle][0] = start_position;
		sequence_of_ledges_to_visit[n_visited_Ledges] = list_of_cycles[active_cycle][start_position + 1];
		int selected_Ledge = list_of_cycles[active_cycle][start_position + 1];
		n_visited_Ledges = n_visited_Ledges + 1;
		//find a new position
		int next_node = Ledges_struct.Ledges_from_end_to_start[selected_Ledge][0];
		int next_position_if_in_cycle = start_position + 1;
		if (next_position_if_in_cycle >= ((int)list_of_cycles[active_cycle].size() - 1)) {
			next_position_if_in_cycle = 0;
		};
		//check all the cycles that should be closed in this node
		while (next_position_if_in_cycle == entry_position_to_the_cycle[active_cycle]) {
			list_of_cycles[active_cycle][0] = -2;//close the cycle
			current_number_active_cycles = current_number_active_cycles - 1;
			if (current_number_active_cycles > 0) {
				active_cycle = list_of_opened_cycles[current_number_active_cycles - 1];
				next_position_if_in_cycle = list_of_cycles[active_cycle][0] + 1;
				if (next_position_if_in_cycle >= ((int)list_of_cycles[active_cycle].size() - 1)) {
					next_position_if_in_cycle = 0;
				};
			}
			else {
				next_position_if_in_cycle = -1;
			};
		};
		start_position = -1;//initialize
		//check if there are new not opened cycles starting in this node
		for (int loc_c = 0; loc_c < list_of_cycles_from_nodes[next_node][0]; loc_c++) {
			int this_cycle = list_of_cycles_from_nodes[next_node][loc_c + 1];
			if (list_of_cycles[this_cycle][0] == -1) {//if not yet opened
				//find its starting position
				for (int loc_p = 0; loc_p < ((int)list_of_cycles[active_cycle].size() - 1); loc_p++) {
					active_cycle = this_cycle;
					current_number_active_cycles = current_number_active_cycles + 1;
					list_of_opened_cycles[current_number_active_cycles - 1] = active_cycle;
					int this_ledge = list_of_cycles[active_cycle][loc_p + 1];
					int first_node_of_Ledge = Ledges_struct.Ledges_from_end_to_start[this_ledge][Ledges_struct.Ledges_from_end_to_start[this_ledge].size() - 1];
					if (first_node_of_Ledge == next_node) {
						start_position = loc_p;//start position
						entry_position_to_the_cycle[active_cycle] = start_position;//save the start position for the cycle
						loc_p = (list_of_cycles[active_cycle].size() - 1) + 5;//exit the loop

					};
				};
				loc_c = list_of_cycles_from_nodes[next_node][0] + 5;
			};//if not yet opened
		};//for each candidate for a not opened cycle

		//if no start_position found (no opened cycle)==> --> proceed
		if ((start_position < 0) && (list_of_cycles[active_cycle][0] != -2)) {
			start_position = next_position_if_in_cycle;
		};

	};
	if (n_visited_Ledges != total_n_ledges) {
		cout << "ERROR: n_visited_Ledges != total_n_ledges" << '\n';
		mistake_reported = true;
		mistake_text = mistake_text + "@truck_and_drone_model: MISTAKE: n_visited_Ledges != total_n_ledges; ";
	};
	//fix the ledge for the operation creation
	Ledges_struct.Ledges_from_end_to_start[0] = temp_vec_stored;
	model_solution.n_stages = 0;
	std::iota(std::rbegin(sequence_of_ledges_to_visit), std::rend(sequence_of_ledges_to_visit), 0);
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
		for (size_t loc_j = Ledges_struct.Ledges_from_end_to_start[next_ledge_num].size() - 1; loc_j-- > 0;){
			int goal_node = Ledges_struct.Ledges_from_end_to_start[next_ledge_num][loc_j];
			model_solution.truck_sorties.back() = backtrack_path(start_node, goal_node, dp.previous_node, model_solution.truck_sorties.back());
			start_node = goal_node;
		}
		*/
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


void Model::reset_model(vector<vector<int>>_edge_status, vector<bool>_customers_to_visit, int _curr_truck_node, int _curr_drone_node, int _last_combined_node)
{
	edge_status = _edge_status;
	customers_to_visit = _customers_to_visit;
	model_solution = Solution(0,instance.n_nodes);
	conservative = true;
	curr_truck_node = _curr_truck_node;
	curr_drone_node = _curr_drone_node;
	last_combined_node = _last_combined_node;
	customer_set = init_customer_set();
	customer_n_nodes = customer_set.size();
	Ledges_struct = Ledges(instance.n_nodes);
	int new_size = dp.n_nodes + dp.artificial_nodes;
	dp.reset_dp_struct(new_size, customer_n_nodes, max_lim);
}

void Model::reset_model_dp_reopt(vector<vector<int>>_edge_status, vector<bool>_customers_to_visit, int _curr_truck_node, int _curr_drone_node, int _curr_drone_cust, int _last_combined_node)
{
	edge_status = _edge_status;
	customers_to_visit = _customers_to_visit;
	model_solution = Solution(0, instance.n_nodes);
	conservative = true;
	curr_truck_node = _curr_truck_node;
	curr_drone_node = _curr_drone_node;
	curr_drone_customer = _curr_drone_cust;
	last_combined_node = _last_combined_node;
	customer_set = init_customer_set();
	customer_n_nodes = customer_set.size();
	Ledges_struct = Ledges(instance.n_nodes);
	int new_size = dp.n_nodes + dp.artificial_nodes+1;
	dp.reset_dp_struct(new_size, customer_n_nodes, max_lim);
}

vector<int> Model::get_ledges_to_visit()
{
	return vector<int>();
}


void Model::backtrack_ledges() {
	int new_i = 1;
	pair<int, int> hilfs_pair = { 0,0 };
	for (int v = 0; v < instance.n_nodes; v++) {
		for (int w = 0; w < instance.n_nodes; w++) {
			for (int i = 0; i < (int)dp.subsets.size(); i++) {
				for (auto& s_i : dp.subsets[i])
				{
					int drone_node = dp.dtsp_drone[v][w][s_i.second];
					if (dp.dtsp_op[v][w][s_i.second] < max_lim && drone_node != 0 && dp.dtsp_op[v][w][s_i.second]>0) {
						vector<int> op(0);
						std::set<int> new_op(std::make_move_iterator(s_i.first.begin()), std::make_move_iterator(s_i.first.end()));
						int ww = w;
						int idx = s_i.second;
						new_i = i;
						int num_erase = 0;
						Ledges_struct.Ledges_dronenode.push_back(drone_node);
						if (drone_node > -1) {
							num_erase = new_op.erase(drone_node);
							Ledges_struct.nodes_on_Ledge[drone_node].push_back(Ledges_struct.n_Ledges);
							std::vector<int> setPred(new_op.begin(), new_op.end());
							new_i = i - num_erase;
							idx = dp.subsets[new_i].at(setPred);
						}
						int pred = dp.dtsp_pred[v][ww][idx].first;
						std::vector<int> setPred(new_op.begin(), new_op.end());
						bool one_loop = false;
						if (new_op.size() == 0) {
							one_loop = true;
						}
						while (pred > -1 || one_loop) {
							op.push_back(ww);
							num_erase = new_op.erase(ww);
							if (ww != w && ww != v) {
								Ledges_struct.nodes_on_Ledge[ww].push_back(Ledges_struct.n_Ledges);
							}
							ww = pred;
							new_i = new_i - num_erase;
							idx = dp.subsets[new_i].at(setPred);
							pred = dp.dtsp_pred[v][ww][idx].first;
							one_loop = false;
						}
						op.push_back(v);
						Ledges_struct.incoming_Ledges[w].push_back(Ledges_struct.n_Ledges);
						Ledges_struct.outgoing_Ledges[v].push_back(Ledges_struct.n_Ledges);
						Ledges_struct.list_of_Ledges.push_back(dp.dtsp_op[v][w][s_i.second]);
						Ledges_struct.Ledges_from_end_to_start.push_back(op);
						Ledges_struct.replan_nodes_Ledge.push_back(hilfs_pair);
						Ledges_struct.add_Ledge();
					}
				}
			}
		}
	}
}

string Model::write_number(double iteration) {
	string loc_s = "";
	int integer_part = iteration;
	loc_s = to_string(integer_part);
	loc_s = loc_s + ".";
	int ostatok = 1000 * iteration; //rest auf russisch
	ostatok = (ostatok % 1000) * 1000;
	loc_s = loc_s + to_string(ostatok);
	return loc_s;
}

vector<int> Model::backtrack_path(int start, int goal, vector<vector<int>> previous_node, vector<int> op)
{	
	vector<int> temp_op(0);
	temp_op.push_back(goal);
	int temp_start = start;
	int temp_goal = goal;
	while (temp_start != previous_node[temp_start][temp_goal]) {
		temp_goal = previous_node[temp_start][temp_goal];
		temp_op.push_back(temp_goal);
	}
	temp_op.push_back(start);
	std::reverse(temp_op.begin(), temp_op.end());
	int correction = 0;
	if (op.size()>0 && temp_op.size()>0 && temp_op.front() == op.back()) {
		correction = 1;
	}
	op.insert(op.end(), temp_op.begin()+correction, temp_op.end());
	return op;
}

bool Model::areEqual(double a, double b)
{
    return fabs(a-b)<0.1;
}

bool Model::checkFirstAndLastEqual(const std::vector<std::vector<int>>& operations_vec) {

    // Check if the vector is empty
    if (operations_vec.empty()) {
        std::cout << "The vector is empty." << std::endl;
        return false;
    }

    // Check if the vector has only one element
    if (operations_vec.size() == 1) {
        std::cout << "The vector has only one element." << std::endl;
        return true;
    }

    // Sort the first and last vectors
    std::vector<int> firstVec = operations_vec.front();
    std::vector<int> lastVec = operations_vec.back();

	if (firstVec.size() != lastVec.size())
    {   // this test saves a lot of time and means we don't need to sort
        return false; 
    }

    std::sort(firstVec.begin(), firstVec.end());
    std::sort(lastVec.begin(), lastVec.end());

    // Check if the sorted vectors are the same
    return (firstVec == lastVec);
}