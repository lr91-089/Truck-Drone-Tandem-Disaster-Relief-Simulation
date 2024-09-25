#include "Model_op.h"


Model_OP::Model_OP(Instance& _instance,vector<vector<int>>& _arc_status_surveillance, vector<bool>_customers_to_visit, bool _manhattan_truck, int _curr_truck_node, int _curr_drone_node ,double _c_start,double _c_max, int _tilim) :
	Model(_instance, _arc_status_surveillance, _arc_status_surveillance, _customers_to_visit, _manhattan_truck, 0.0 ,  _curr_truck_node,_curr_drone_node) ,c_start(_c_start), c_max(_c_max) , tilim(_tilim), w_node(_instance.n_nodes,0), p_node(_instance.n_nodes,0.0)
{

}
void Model_OP::solve()
{
	IloEnv envtsp;
	int NUM_THREADS=1;
	modelop = IloModel(envtsp, "Model OP");
	cplexop = IloCplex(modelop);
	initialize();

	try {
		// Initialize variables
		buildVariables(envtsp);

		// Initialize objective
		buildObjective(envtsp);

		buildConstraints(envtsp);

		cplexop = IloCplex(modelop);
		cplexop.setParam(IloCplex::Param::TimeLimit, tilim);
		cplexop.setParam(IloCplex::Param::Threads, NUM_THREADS);
		
		//cplexop.exportModel("modelOP.lp");
		bool loc_problem_is_feasible;
		loc_problem_is_feasible = cplexop.solve();
		
		if (loc_problem_is_feasible == true) {
			putsolutionOnScreen();
			////read_solution(env);
			read_solution(envtsp);
			write_solution_TSP(envtsp);
		}
		else {
			cout << "infeasible OP solution" << '\n';
			mistake_reported = true;
			mistake_text = "infeasible_OP_solution, ";
		};
	}
	catch (IloException& ex) {
		cerr << "cplexop Error: " << ex << '\n';
	}
	catch (...) {
		cerr << "ErrorTSP " << '\n';
	}
	envtsp.end();
}
;

void Model_OP::buildVariables(IloEnv& env) {


	Xt = IloArray<IloBoolVarArray>(env, number_nodes_to_visit);
	for (int k = 0; k < number_nodes_to_visit; ++k) {
		IloBoolVarArray Xtemp(env, number_nodes_to_visit);
		string name = "Xtsp_" + to_string(k);
		Xtemp.setNames(name.c_str());
		Xt[k] = Xtemp;
	};


	Ut = IloIntVarArray(env, number_nodes_to_visit, 0, number_nodes_to_visit - 1);
	string name = "Utsp_";
	Ut.setNames(name.c_str());

};

void Model_OP::initialize() {
	number_nodes_to_visit = 0;
	references_to_customers = vector<int>(instance.n_nodes + 1);
	//if (curr_drone_node == 0){
	//starts_from_depot = true;//whether the start is from the depot
	references_to_customers[0] = curr_drone_node;
	number_nodes_to_visit = number_nodes_to_visit + 1;
	/*}
	else{
	starts_from_depot = false;*/
	//};
    int temp_prize = 0;
	for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
		if ((customers_to_visit[loc_i] == true) && (loc_i != curr_drone_node)) {//we have already taken the current position into account
			references_to_customers[number_nodes_to_visit] = loc_i;
			number_nodes_to_visit = number_nodes_to_visit + 1;
            temp_prize = 0;
            for(int loc_j = 1; loc_j < instance.n_nodes; loc_j++) {
                if(check_if_add_edge_to_prize(loc_i, loc_j)==true) {
                    temp_prize = temp_prize +1;
                 }
            }
            p_node[loc_i] = temp_prize;
		};
	};

	//if (curr_drone_node != 0){
	references_to_customers[number_nodes_to_visit] = -1;//artificial sink node. No cost to return to this node
	number_nodes_to_visit = number_nodes_to_visit + 1;
	//double w_max = *max_element(instance.drone_distances[0].begin(),instance.drone_distances[0].end());
	for(size_t node_idx=0;node_idx < w_node.size();node_idx++) {
		//w_node[node_idx] = instance.drone_distances[0][node_idx]/w_max;
		//no weights
		w_node[node_idx] = 1.0;
	}
	//};
    //compute the weight of each node
};

void Model_OP::buildObjective(IloEnv& env) {
	// Building objective
	IloExpr objective(env);
	// Adding coefficients to objective
	for (int i = 1; i < number_nodes_to_visit-1; ++i) {
		for (int j = 1; j < number_nodes_to_visit; ++j) {
			if(i!=j) {
				int first_node = references_to_customers[i];
				objective += w_node[first_node]*p_node[first_node] * Xt[i][j];
			}
		};
	};
	cout << objective << '\n';
	// Adding abjective to model
	modelop.add(IloMaximize(env, objective));
}

bool Model_OP::check_if_add_edge_to_prize(int i, int j)
{
    if(i==j){
        return false;
    }
    //edge should not be -1
    if(arc_status_after_surveillance[i][j]==0){
        return true;
    }
    return false;
};

void Model_OP::buildConstraints(IloEnv& env) {

	//one outgoing edge
	for (int i = 0; i < number_nodes_to_visit; ++i) {
		IloExpr constraint(env);
		if ((i != (number_nodes_to_visit - 1)) || (starts_from_depot == true)) {
			for (int j = 1; j < number_nodes_to_visit; j++) {//cannot retirn to source node (Hamiltonian path)
				if (i != j) {
					constraint += Xt[i][j];
				};
			};
			IloConstraint c(constraint <= 1);
			c.setName("at most 1 outgoing edge");
			modelop.add(c);
		};

	};

	//one incoming edge
	for (int i = 0; i < number_nodes_to_visit; ++i) {
		IloExpr constraint(env);
		if ((i != 0) || (starts_from_depot == true)) {
			for (int j = 0; j < (number_nodes_to_visit - 1); j++) {//cannot start from sink node (Hamiltonian path)
				if (i != j) {
					constraint += Xt[j][i];
				};
			};
			IloConstraint c(constraint <= 1);
			c.setName("at most 1 incoming edge");
			modelop.add(c);
		};

	};

    //Flow Conversion
	for (int i = 1; i < number_nodes_to_visit-1; ++i) {
		IloExpr constraint(env);
        for (int j = 0; j < number_nodes_to_visit-1; j++) {//visited node must be left};
			if(i != j ) {
				constraint += Xt[j][i];
			}
        };
		for (int j = 1; j < number_nodes_to_visit; j++) {//visited node must be left};
			if(i != j ) {
				constraint -= Xt[i][j];
			}
        };
        IloConstraint c(constraint == 0);
        c.setName("Flow Conversion");
        modelop.add(c);

	};

    //one outgoing edge for source
    int temp_i = 0;
    IloExpr constraint1(env);
    for (int j = 1; j < number_nodes_to_visit; j++) {//cannot retirn to source node (Hamiltonian path)
        constraint1 += Xt[temp_i][j];
    };
    IloConstraint c1(constraint1 == 1);
    c1.setName("1 outgoing edge source");
    modelop.add(c1);


	//one incoming edge for sink
    IloExpr constraint2(env);
    temp_i = number_nodes_to_visit - 1;
    for (int j = 1; j < (number_nodes_to_visit - 1); j++) {//cannot start from sink node (Hamiltonian path)
            constraint2 += Xt[j][temp_i];
    };
    IloConstraint c2(constraint2 == 1);
    c2.setName("1 incoming edge sink");
    modelop.add(c2);

    // max tour duration constraint
	IloExpr constraint3(env);
	// Adding coefficients to constraint
	for (int k = 0; k < number_nodes_to_visit; ++k) {
		for (int l = k + 1; l < number_nodes_to_visit; ++l) {
			if ((starts_from_depot == true) || ((k != 0) && (l != (number_nodes_to_visit - 1)))) {
				int first_node = references_to_customers[k];
				int second_node = references_to_customers[l];
				constraint3 += instance.drone_distances[first_node][second_node] * Xt[k][l];
				constraint3 += instance.drone_distances[second_node][first_node] * Xt[l][k];
			};
			if ((starts_from_depot == false) && (k == 0)) {
				int first_node = references_to_customers[k];
				if (references_to_customers[l] != -1) {
					int second_node = references_to_customers[l];
					constraint3 += instance.drone_distances[first_node][second_node] * Xt[k][l];
				}
				else {
					//since the cost is 0
				};
			}
			else {
				if ((starts_from_depot == false) && (l == (number_nodes_to_visit - 1))) {//since the cost is 0! (the last node is artificial node) Why is the cost zero for the last edge? Check it!
					int first_node = references_to_customers[k];
					int second_node = 0;
					constraint3 += instance.drone_distances[first_node][second_node] * Xt[k][l];
				};
			};
		};
	};
    constraint3 += c_start;
	IloConstraint c3(constraint3 <= c_max);
    c3.setName("max tour duration limit");
    modelop.add(c3);


	//Miller-Tucker-Zemlin condition
	for (int i = 1; i < number_nodes_to_visit; ++i) {
		for (int j = 1; j < number_nodes_to_visit; ++j) {
			if ((i != j) && ((starts_from_depot == true) || ((i != (number_nodes_to_visit - 1)) && (j != 0)))) {
				IloExpr constraint(env);
				constraint += number_nodes_to_visit * Xt[i][j] + Ut[i] - Ut[j];
				IloConstraint c(constraint <= number_nodes_to_visit - 1);
				c.setName("MTZ constraints");
				modelop.add(c);
			};
		};
	};
    //fix first node
    modelop.add(Ut[0]==0);


	////Technical: numbering of nodes only positive and up to N
	//for (int loc_i = 0; loc_i < number_nodes_to_visit; ++loc_i){
	//	IloConstraint c(Ut[loc_i] >= 0);
	//	c.setName("nonnegativity");
	//	modelop.add(c);
	//	IloConstraint cb(Ut[loc_i] <= number_nodes_to_visit-1);
	//	cb.setName("up to N");
	//	modelop.add(cb);
	//};

};

void Model_OP::putsolutionOnScreen() {
	cout << cplexop.getStatus();
	cout << " ";
	if(cplexop.getStatus()!=3){
		cout << cplexop.getObjValue();
		cout << '\n';
	} 
	else{
		cout << '\n';
	} 
	//cout << cplex.getBestObjValue();


};

void Model_OP::read_solution(IloEnv& env) {
	for (int loc_i = 0; loc_i < number_nodes_to_visit; loc_i++) {
		for (int loc_j = loc_i + 1; loc_j < number_nodes_to_visit; loc_j++) {
			if (cplexop.isExtracted(Xt[loc_i][loc_j]) == true) {
				double xx = cplexop.getValue(Xt[loc_i][loc_j]);
				if (xx >= 0.5) {
					cout << "; edge";
					cout << references_to_customers[loc_i];
					cout << "_";
					cout << references_to_customers[loc_j];
					cout << " " << Xt[loc_i][loc_j];
					cout << '\n';
				};
			};
			if ((starts_from_depot == true) || ((loc_i != 0) && (loc_j != (number_nodes_to_visit - 1)))) {
				if (cplexop.isExtracted(Xt[loc_j][loc_i]) == true) {
					double xx = cplexop.getValue(Xt[loc_j][loc_i]);
					if (xx >= 0.5) {
						cout << "; edge";
						cout << references_to_customers[loc_j];
						cout << "_";
						cout << references_to_customers[loc_i];
						cout << " " << Xt[loc_i][loc_j];
						cout << '\n';
					};
				};
			};
		};
	};

	for (int loc_i = 0; loc_i < number_nodes_to_visit; loc_i++) {
		if (cplexop.isExtracted(Ut[loc_i]) == true) {
			double xx = cplexop.getValue(Ut[loc_i]);
			cout << "; node";
			cout << references_to_customers[loc_i];
			cout << " numbering ";
			cout << xx;
			cout << '\n';
		};
	};


};

void Model_OP::write_solution_TSP(IloEnv& env) {
	if ((cplexop.getStatus() == 2) || (cplexop.getStatus() == 1)) {
		model_solution.obj_value = cplexop.getObjValue();
	}
	else {

		model_solution.obj_value = -1;

	};


	//initialize
	model_solution.drone_sorties.resize(0);
	model_solution.truck_sorties.resize(0);
	model_solution.drone_customers.resize(0);
	int loc_i = 0;
	bool all_visited = false;

	model_solution.n_stages = 0;
	vector<int>help_vector = vector<int>(0);
	while (all_visited == false) {
		for (int loc_j = 0; loc_j < (number_nodes_to_visit); loc_j++) {
			if ((loc_i != loc_j) && ((starts_from_depot == true) || ((loc_i != (number_nodes_to_visit - 1)) && (loc_j != 0)))) {
				if (cplexop.isExtracted(Xt[loc_i][loc_j]) == true) {
					double xx = cplexop.getValue(Xt[loc_i][loc_j]);
					if (xx >= 0.5) {
						int first_node = references_to_customers[loc_i];
						int second_node = references_to_customers[loc_j];
						if (second_node <= -1) {
							all_visited = true;
							second_node = 0;
						}
						help_vector = backtrack_path(first_node, second_node, instance.drone_fromtriple_previousnode, help_vector);
						
						loc_i = loc_j;
						loc_j = instance.n_nodes + 5;//since only one outgoing node
					};//if extracted and equal 1
				};
			};

		};//for each second node
	};//until all visited
	//check if tour starts with smaller node
	if(help_vector.front()==help_vector.back()) {
		vector<int> help_vector_orientation(0);
		for (int i: help_vector) {
			if(i<instance.real_n_nodes){
				help_vector_orientation.push_back(i);
			}
		}
		while(help_vector_orientation.size()>3 && help_vector_orientation.back()==help_vector_orientation.front()){
			help_vector_orientation.pop_back();
			help_vector_orientation.erase(help_vector_orientation.begin());
		}
		int second_node = help_vector_orientation.front();
		int vorletzte_node = help_vector_orientation.back();
		if(second_node<vorletzte_node){
			std::reverse(help_vector.begin(), help_vector.end());
		}
	}
	model_solution.drone_sorties.push_back(help_vector);
	vector<int> help_vector_drone_customers = { help_vector.begin() + 1, help_vector.end() - 1 };
	model_solution.drone_customers.push_back(help_vector_drone_customers);
	model_solution.list_of_op_customers = model_solution.drone_customers;
	model_solution.truck_sorties.push_back(vector<int>(1, curr_truck_node));
	model_solution.n_stages = model_solution.n_stages + 1;
};