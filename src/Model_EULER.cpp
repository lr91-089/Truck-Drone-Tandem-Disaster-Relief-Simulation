#include "Model_EULER.h"

Model_EULER::Model_EULER(Instance& _instance, vector<vector<int>>_edge_status, vector<bool>_customers_to_visit, bool _manhattan_truck, reoptParams& _reopt_parameters, int _tilim):
	Model(_instance, _edge_status, _edge_status, _customers_to_visit, _manhattan_truck, 0.0 , _reopt_parameters.curr_truck_node, _reopt_parameters.curr_drone_node, _reopt_parameters.last_combined_node), tilim(_tilim), reopt_parameters(_reopt_parameters), reopt_solution(Solution(0,instance.n_nodes))
{
	init_dp_struct(0);
	init_euler_model_struct();

};

void Model_EULER::solve(int setting) {
	IloEnv env;
	try {
		model = IloModel(env, "Model_EULER 1");
		cplex = IloCplex(model);

		if (conservative) {
			add_conservative_Loops();
			conservative_calculation();
		}
		else {
			solve_operations(setting);

			add_Reopt_Operations();

			backtrack_ledges();

			//if (reopt_parameters.last_combined_node != 0) {
				//add_aritficial_operation();
			//}

			buildVariables_Eulermodel(env);

			buildObjective_Eulermodel(env);

			buildConstraints_Eulermodel(env);

			cplex = IloCplex(model);
			cplex.setParam(IloCplex::Param::TimeLimit, tilim);
			//cplex.exportModel("C:\\Users\\un_po\\source\\repos\\drone_truck_disaster\\x64\\Debug\\model1.lp");
			string mod_name = "model1.lp";
			//cplex.exportModel(mod_name.c_str());
			bool loc_problem_is_feasible;
			loc_problem_is_feasible = cplex.solve();


			putsolutionOnScreen();

			//for testing
			read_solution_complete_Eulermodel(env);


			if (loc_problem_is_feasible == true) {
				if ((cplex.getStatus() == 2) || (cplex.getStatus() == 1)) {
					model_solution.obj_value = cplex.getObjValue();
				}
				else {

					model_solution.obj_value = -1;

				};
				write_solution();
				//vector<int> flags = get_indices_reopt();
				//model_solution.truck_sorties = mod_tour_reopt(flags, model_solution.truck_sorties);
				//model_solution.drone_sorties = mod_tour_reopt(flags, model_solution.drone_sorties);
				//model_solution.drone_customers = mod_tour_reopt(flags, model_solution.drone_customers);
				model_solution.n_stages = model_solution.truck_sorties.size();
			}
		};
		env.end();
	}
	catch (IloException& ex) {
		cerr << "Cplex Error: " << ex << '\n';
	}
	catch (...) {
		cerr << "Other Exception" << '\n';
	};
}

void Model_EULER::reset_euler_model(vector<vector<int>>_edge_status, vector<bool>_customers_to_visit, reoptParams& _reopt_parameters)
{
	reset_model(_edge_status, _customers_to_visit, _reopt_parameters.curr_truck_node,_reopt_parameters.curr_drone_node,_reopt_parameters.last_combined_node);
	init_dp_struct(0);
	init_euler_model_struct();
	reopt_solution.truck_distances = dp.distance_metric_naive;
	reopt_solution.drone_distances = dp.reopt_drone_distances;
	reopt_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
}

void Model_EULER::buildVariables_Eulermodel(IloEnv& env) {






	Xo = IloBoolVarArray(env, Ledges_struct.n_Ledges);
	for (int k = 0; k < Ledges_struct.n_Ledges; ++k) {
		string name = "Xo_" + to_string(k + 1);
		Xo[k].setName(name.c_str());
	};


	Yv = IloBoolVarArray(env, instance.n_nodes);
	for (int k = 0; k < instance.n_nodes; ++k) {
		string name = "Yv_" + to_string(k + 1);
		Yv[k].setName(name.c_str());
	};


};


void Model_EULER::buildObjective_Eulermodel(IloEnv& env) {
	// Building objective
	IloExpr objective(env);
	// Adding coefficients to objective
	for (int k = 0; k < Ledges_struct.n_Ledges; ++k)
		objective += Xo[k] * Ledges_struct.list_of_Ledges[k];
	// Adding abjective to Model_EULER
	try {
		//Error Unhandled exception at 0x00007FFB05871AFC (ntdll.dll) in drone_truck_disaster.exe: 0xC0000005: Access violation reading location 0x0000000000000024.
		model.add(IloMinimize(env, objective));
	}
	catch (const IloException& e)
	{
		cerr << e;
		throw; // if you like
	}
};

void Model_EULER::buildConstraints_Eulermodel(IloEnv& env) {

	//Each required node is visited at least once
	for (int i = 0; i < instance.n_nodes; ++i) {
		if (customers_to_visit[i] == true) {
			IloExpr constraint(env);
			int ledge_number = 0;
			for (int loc_m = 0; loc_m < (int)Ledges_struct.nodes_on_Ledge[i].size(); loc_m++) {
				ledge_number = Ledges_struct.nodes_on_Ledge[i][loc_m];
				constraint += Xo[ledge_number];//add middle nodes
			};
			for (int loc_m = 0; loc_m < (int)Ledges_struct.outgoing_Ledges[i].size(); loc_m++) {
				ledge_number = Ledges_struct.outgoing_Ledges[i][loc_m];
				constraint += Xo[ledge_number];//add outgoing nodes
			};
			for (int loc_m = 0; loc_m < (int)Ledges_struct.incoming_Ledges[i].size(); loc_m++) {
				ledge_number = Ledges_struct.incoming_Ledges[i][loc_m];
				constraint += Xo[ledge_number];//add incoming nodes
			};
			IloConstraint cb(constraint >= 1);
			string name = "required_nodes_visited_at_least_once_" + to_string(i + 1);
			const char* cname = name.c_str();
			cb.setName(cname);
			model.add(cb);
			constraint.end();
		};
	};
	//
	/*
	if(reopt_parameters.last_combined_node!=0){
		IloExpr constraintXX(env);
		constraintXX += Xo[Ledges_struct.list_of_Ledges.size()-1];
		IloConstraint cbXX(constraintXX >= 1);
		cbXX.setName("The artificial Ledge must be taken!");
		model.add(cbXX);
	}
	*/
	// 
	//there should be operations starting and ending in the depot

	IloExpr constraint1(env);

	for (int loc_m = 0; loc_m < (int)Ledges_struct.outgoing_Ledges[0].size(); loc_m++) {
		int ledge_number = Ledges_struct.outgoing_Ledges[0][loc_m];
		constraint1 += Xo[ledge_number];//add outgoing nodes
	};
	IloConstraint cb1(constraint1 >= 1);
	cb1.setName("there must be Ledges starting in 0");
	model.add(cb1);

	// dummy for break points
	IloExpr constraint2(env);

	for (int loc_m = 0; loc_m < (int)Ledges_struct.incoming_Ledges[0].size(); loc_m++) {
		int ledge_number = Ledges_struct.incoming_Ledges[0][loc_m];
		constraint2 += Xo[ledge_number];//add outgoing nodes
	};
	IloConstraint cb2(constraint2 >= 1);
	cb2.setName("there must be Ledges ending in 0");
	model.add(cb2);


	for (int i = 0; i < instance.n_nodes; ++i) {
		IloExpr constraint(env);

		for (int loc_m = 0; loc_m < (int)Ledges_struct.incoming_Ledges[i].size(); loc_m++) {
			int ledge_number = Ledges_struct.incoming_Ledges[i][loc_m];
			constraint += Xo[ledge_number];//add outgoing nodes
		};
		for (int loc_m = 0; loc_m < (int)Ledges_struct.outgoing_Ledges[i].size(); loc_m++) {
			int ledge_number = Ledges_struct.outgoing_Ledges[i][loc_m];
			constraint += (-Xo[ledge_number]);//substract outgoing nodes
		};
		IloConstraint cb(constraint == 0);
		string name = "flow_constraints_" + to_string(i + 1);
		const char* cname = name.c_str();
		cb.setName(cname);
		model.add(cb);
		constraint.end();
	};

	//identify nodes where operations end
	for (int i = 0; i < instance.n_nodes; ++i) {
		IloExpr constraint(env);

		for (int loc_m = 0; loc_m < (int)Ledges_struct.incoming_Ledges[i].size(); loc_m++) {
			int ledge_number = Ledges_struct.incoming_Ledges[i][loc_m];
			constraint += Xo[ledge_number];//add outgoing nodes
		};
		constraint += (-instance.n_nodes * Yv[i]);
		IloConstraint cb(constraint <= 0);
		string name = "identify_combined_nodes1_" + to_string(i + 1);
		const char* cname = name.c_str();
		cb.setName(cname);
		model.add(cb);
	};

	IloConstraint cb3(Yv[0] == 1);
	cb3.setName("depot_should be visited");
	model.add(cb3);


	//test
	/*
	vector<int> customer_set = vector<int>(0);
	for (int k = 1; k < customers_to_visit.size(); k++) {
		if (customers_to_visit[k]) {
			customer_set.push_back(k);
		}
	}
	std::vector<vector<int>> v_subsets(0);
	vector<int> subset;
	int num = 0;
	for (int loc_i = 0; loc_i < pow(2, instance.n_nodes); loc_i++)
	{
		subset.resize(0);
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++)
		{
			if (loc_i & (1 << loc_j))
				subset.push_back(loc_j);
		}
		std::vector<int> set_intersection;
		std::set_intersection(std::begin(subset), std::end(subset), // the first vector...
			std::begin(customer_set), std::end(customer_set), // ...minus the second...
			std::back_inserter(set_intersection));     // ...is stored into here
		if(set_intersection.size()>0){
			v_subsets.push_back(subset);
			num = num + 1;
		}
	}



	for (vector<int> sub : v_subsets) {
			for (int loc_v : sub) {//for each node from the subset
					IloExpr constraint(env);
					constraint += (-Yv[loc_v]);
					for (int loc_m :sub) {//for all ending Ledges of the subset that start elsewhere
							for (int loc_f = 0; loc_f < Ledges_struct.incoming_Ledges[loc_m][0]; loc_f++) {
								int ledge_number = Ledges_struct.incoming_Ledges[loc_m][loc_f + 1];
								int first_node_ledge = Ledges_struct.Ledges_from_end_to_start[ledge_number].back();
								if ((first_node_ledge == 0) || ((first_node_ledge > 0) && (std::find(sub.begin(), sub.end(), first_node_ledge) == sub.end()))) {
									constraint += Xo[ledge_number];//add to the constraint
								};
							};
					};
					IloConstraint cb(constraint >= 0);
					cb.setName("subset_elimination");
					model.add(cb);
					constraint.end();
			};
	};
	*/
	//for each subset of nodes, there should be possible to reach this subset from the outside
	std::vector<bool>selection_nodes = vector<bool>(instance.n_nodes - 1, false);
	bool loc_stop = false;//stops if all the subsets have been examined
	//int cs_n = 0;

	while (loc_stop == false) {
		int loc_i;
		for (loc_i = 0; loc_i < (instance.n_nodes - 1); loc_i++) {//first node is 1
			if (selection_nodes[loc_i] == false) {
				selection_nodes[loc_i] = true;
				loc_i = instance.n_nodes + 5;
			}
			else {
				selection_nodes[loc_i] = false;
			};
		};
		if (loc_i < (instance.n_nodes + 5)) {//no subset has been selected
			loc_stop = true;
		}
		else {
			std::vector<int> selection_subset(0); //
			for (int node_i = 0; node_i < (int)selection_nodes.size(); node_i++) {
				if (selection_nodes[node_i]) {
					selection_subset.push_back(node_i);
				};
			};
			for (int loc_v = 1; loc_v < instance.n_nodes; loc_v++) {//for each node from the subset
				if (selection_nodes[loc_v - 1] == true) {
					IloExpr constraint(env);


					constraint += (-Yv[loc_v]);
					for (int loc_m = 1; loc_m < instance.n_nodes; loc_m++) {//for all ending Ledges of the subset that start elsewhere
						if (selection_nodes[loc_m - 1] == true) {
							for (int loc_f = 0; loc_f < (int)Ledges_struct.incoming_Ledges[loc_m].size(); loc_f++) {
								int ledge_number = Ledges_struct.incoming_Ledges[loc_m][loc_f];
								int first_node_ledge = Ledges_struct.Ledges_from_end_to_start[ledge_number].back();
								if ((first_node_ledge == 0) || ((first_node_ledge > 0) && (selection_nodes[first_node_ledge - 1] == false))) {
									constraint += Xo[ledge_number];//add to the constraint
								};
							};
						};
					};
					IloConstraint cb(constraint >= 0);
					cb.setName("subset_elimination");
					model.add(cb);
					//cs_n = cs_n + 1;
					constraint.end();
				};
			};
		};
	};
	//cout << cs_n;
	IloExpr constraint_reopt(env);

	if (Ledges_struct.reopt_ct_ledges.size() > 0) {
		for (int ledge_number : Ledges_struct.reopt_ct_ledges) {
			constraint_reopt += Xo[ledge_number];//add outgoing nodes
		};
		IloConstraint cb_reopt(constraint_reopt >= 1);
		cb_reopt.setName("there must be one reopt closing operation");
		model.add(cb_reopt);
	}
};

void Model_EULER::add_Reopt_Operations() {
	Ledges_struct.reopt_ct_ledges.resize(0);
	for (int w = 0; w < instance.n_nodes; w++) {
		bool drone_cust = false;
		double loc_truck_dist = max_lim;
		double loc_drone_dist = max_lim;
		double truck_leg_distance = dp.distance_metric_naive[curr_truck_node][w]; //not adapted to the new definition of distances
		if (truck_leg_distance >= 0) {
			loc_truck_dist = truck_leg_distance;
		};
		//do we have a drone customer
		if (reopt_parameters.curr_drone_cust > -1) {
			loc_drone_dist = dp.reopt_drone_distances[curr_drone_node][reopt_parameters.curr_drone_cust] + dp.reopt_drone_distances[reopt_parameters.curr_drone_cust][w];
			drone_cust = true;
		}
		//if no drone customer
		else {
			double drone_leg_distance2 = dp.reopt_drone_distances[curr_drone_node][w];
			loc_drone_dist = drone_leg_distance2;
		};
		double z = max(loc_truck_dist, loc_drone_dist);
		if (z < max_lim) {
			vector<int>op(1,0);
			//op.push_back(curr_truck_node);
			//int loc_temp_truck_node = curr_truck_node;
			//int op_id = dp.subsets[op.size()].at(op);
			//op = backtrack_path(curr_truck_node, w, op);
			/*
			while (loc_temp_truck_node != dp.previous_node[loc_temp_truck_node][w]) {
				op.insert(op.begin(), loc_temp_truck_node);
				if (loc_temp_truck_node != w && loc_temp_truck_node!=instance.n_nodes) {
					Ledges_struct.nodes_on_Ledge[loc_temp_truck_node].push_back(Ledges_struct.n_Ledges);
				}
				loc_temp_truck_node = dp.previous_node[loc_temp_truck_node][w];
			};
			op.push_back(w);
			*/
			if (drone_cust) {
				Ledges_struct.Ledges_dronenode.push_back(reopt_parameters.curr_drone_cust);
				if (reopt_parameters.curr_drone_cust != w) {
					Ledges_struct.nodes_on_Ledge[reopt_parameters.curr_drone_cust].push_back(Ledges_struct.n_Ledges);
				}
			}
			else {
				//Ledges_struct.Ledges_dronenode.push_back(w);
				Ledges_struct.Ledges_dronenode.push_back(-1);//push back -1 
			}
			std::reverse(op.begin(), op.end()); //make it from end to start
			for (int node : op) {
				if (node<instance.n_nodes && node != w && node != 0) {
					Ledges_struct.nodes_on_Ledge[node].push_back(Ledges_struct.n_Ledges);
				}
			}
			Ledges_struct.list_of_Ledges.push_back(z);
			Ledges_struct.incoming_Ledges[w].push_back(Ledges_struct.n_Ledges);
			Ledges_struct.outgoing_Ledges[0].push_back(Ledges_struct.n_Ledges);
			
			Ledges_struct.Ledges_from_end_to_start.push_back(op);
			
			Ledges_struct.replan_nodes_Ledge.push_back({ last_combined_node,w });
			Ledges_struct.reopt_ct_ledges.push_back(Ledges_struct.n_Ledges);
			Ledges_struct.add_Ledge();
		}
	}
}


void Model_EULER::add_aritficial_operation()
{
	vector<int> op(0);
	op.push_back(reopt_parameters.last_combined_node);
	op.push_back(0);
	Ledges_struct.incoming_Ledges[reopt_parameters.last_combined_node].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.outgoing_Ledges[0].push_back(Ledges_struct.n_Ledges);
	Ledges_struct.list_of_Ledges.push_back(0);
	Ledges_struct.Ledges_from_end_to_start.push_back(op);
	Ledges_struct.Ledges_dronenode.push_back(-2);
	Ledges_struct.replan_nodes_Ledge.push_back({ 0,0 });
	Ledges_struct.add_Ledge();
}
;

void Model_EULER::putsolutionOnScreen() {
	cout << cplex.getStatus();
	cout << " ";
	cout << cplex.getObjValue();
	cout << '\n';
	//cout << cplex.getBestObjValue();


};

void Model_EULER::init_euler_model_struct()
{
	/*
	if (last_combined_node != 0 && (curr_drone_node != curr_truck_node)) {
		customers_to_visit[last_combined_node] = true;
		customer_set.resize(0);
		for (int k = 1; k < customers_to_visit.size(); k++) {
			if (customers_to_visit[k]) {
				customer_set.push_back(k);
			}
		}
	};
	*/
	if (curr_truck_node >= instance.n_nodes) {
		add_artificial_truck_node(reopt_parameters.artificial_node);
	}
	else if (curr_drone_node >= instance.n_nodes) {
		add_artificial_drone_node(reopt_parameters.artificial_node);
	}
	model_solution.truck_distances = dp.distance_metric_naive;
	model_solution.drone_distances = dp.reopt_drone_distances;
	model_solution.previous_drone_node = dp.reopt_drone_fromtriple_previousnode;
}

vector<int> Model_EULER::get_ledges_to_visit() {
	vector<int> loc_visited_ledges(0);
	for (int ledge = 0; ledge < Xo.getSize(); ledge++) {
		if (cplex.isExtracted(Xo[ledge]) == IloTrue) {
			double xx = cplex.getValue(Xo[ledge]);
			if (xx >= 0.5) {
				loc_visited_ledges.push_back(ledge);
			};
		};
	};
	return loc_visited_ledges;
};

void Model_EULER::read_solution_complete_Eulermodel(IloEnv& env) {
	for (int loc_i = 0; loc_i < instance.n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < instance.n_nodes; loc_j++) {
			if (edge_status[loc_i][loc_j] >= 1) {
				cout << "; edge";
				cout << (loc_i + 1);
				cout << "_";
				cout << (loc_j + 1);
				cout << '\n';
			};
		};
	};


	for (int k = 0; k < Xo.getSize(); ++k) {
		double newX = cplex.getValue(Xo[k]);
		if (newX > 0.49) {
			cout << k;
			cout << "th ledgee: ";
			cout << newX;
			cout << " distance: ";
			cout << Ledges_struct.list_of_Ledges[k];
			cout << '\n';
		};
	};

	for (int k = 0; k < Yv.getSize(); ++k) {
		double newX = cplex.getValue(Yv[k]);
		if (newX > 0.49) {
			cout << k;
			cout << "th node: ";
			cout << newX;
			cout << '\n';
		};
	};
};

vector<vector<int>> Model_EULER::mod_tour_reopt(vector<int> flag, vector<vector<int>>& truck_sorties) {
	if (flag.size() > 0) {
		vector<vector<int>> temp_truck_sorties(0);
		for (int i = 0; i < (int)truck_sorties.size(); i++) {
			if (flag[i] == 0) {
				temp_truck_sorties.push_back(truck_sorties[i]);
			}
		}
		for (int i = 0; i < (int)truck_sorties.size(); i++) {
			if (flag[i] == 1) {
				temp_truck_sorties.push_back(truck_sorties[i]);
			}
		}
		return temp_truck_sorties;
	}
	return truck_sorties;
}

vector<int> Model_EULER::get_indices_reopt() {
	//zero indices first
	//ones later
	vector<int> temp_truck_sorties_idx(0);
	bool starting_node = false;
	if (reopt_parameters.n_replan_reopt > 0) {
		for (int i = 0; i < (int)model_solution.truck_sorties.size(); i++) {
			if (model_solution.truck_sorties[i].front() > 0) {
				if (!starting_node) {
					temp_truck_sorties_idx.push_back(1);
					if (i == flag_reopt_start_idx) {
						temp_truck_sorties_idx.back() = 0;
						starting_node = true;
					}
				}
				else {
					temp_truck_sorties_idx.push_back(0);
				}
			}
			else {
				temp_truck_sorties_idx.push_back(-1);
			}
		}
	}
	return temp_truck_sorties_idx;
}

double Model_EULER::evaluate_reopt_distance(int v, int w, vector<vector<double>>& distance)
{
	double evaluated_distance_value;
	if (v >= instance.n_nodes) {
		v = instance.n_nodes;
	}
	evaluated_distance_value = distance[v][w];
	return evaluated_distance_value;
}