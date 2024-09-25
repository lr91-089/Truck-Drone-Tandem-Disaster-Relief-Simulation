#include "Model_tsp.h"

//default values bool manhattan_truck=true, int curr_truck_node = 0,int curr_drone_node=0,int tilim=1800, bool arc_based_tsp=false
Model_TSP::Model_TSP(Instance& _instance,vector<vector<int>>& truck_edges_to_check, vector<vector<int>>& arc_status_surveillance, vector<bool>_customers_to_visit, bool _manhattan_truck, int _curr_truck_node, int _curr_drone_node, bool _arc_based_tsp, int _tilim) :
	Model(_instance, vector<vector<int>>(_instance.n_nodes,vector<int>(_instance.n_nodes, 1)), vector<vector<int>>(_instance.n_nodes,vector<int>(_instance.n_nodes, 1)), _customers_to_visit, _manhattan_truck, 0.0 ,  _curr_truck_node,_curr_drone_node), tilim(_tilim), arc_based_tsp(_arc_based_tsp) , unvisited_truck_edges(truck_edges_to_check)
{
	if(arc_based_tsp==true){
		customers_to_visit[_curr_truck_node] = true;
		//only needed for advanced arc based TSP
		unvisited_truck_edges= vector<vector<int>>(instance.n_nodes,vector<int>(instance.n_nodes, -1));
		int curr_node = truck_edges_to_check[0][0];
		int next_node = curr_node; 
		for(int i=0;i<(int) truck_edges_to_check.size();i++) {
			if(truck_edges_to_check[i].size()>1) {
				for(int j=1;j< (int) truck_edges_to_check[i].size();j++) {
					next_node = truck_edges_to_check[i][j];
					//start and end nodes needs to be included
					if(arc_status_surveillance[next_node][curr_node]==0 || next_node==0 || curr_node==0) {
						unvisited_truck_edges[next_node][curr_node] = 1;
					}
					curr_node = next_node;
				}
			}
			else{
				unvisited_truck_edges[curr_node][next_node] = 1;
				curr_node = next_node;
			}
		}
	}	
}

void Model_TSP::solve()
{
	IloEnv envtsp;
	int NUM_THREADS=1;
	modeltsp = IloModel(envtsp, "Model 2");
	cplextsp = IloCplex(modeltsp);
	initialize();

	try {
		// Initialize variables
		buildVariables(envtsp);

		// Initialize objective
		buildObjective(envtsp);


		//vector<vector<int>> temp = vector<vector<int>>(0, vector<int>(0))

		if(arc_based_tsp==true){
			buildConstraintsEdges(envtsp);
		}
		else{
			buildConstraintsNaive(envtsp);
		} 

		cplextsp = IloCplex(modeltsp);
		cplextsp.setParam(IloCplex::Param::TimeLimit, tilim);
		cplextsp.setParam(IloCplex::Param::Threads, NUM_THREADS);
		//cplextsp.setParam(IloCplex::Param::Feasopt::Mode, 2);
		//cplextsp.setParam(IloCplex::Param::Emphasis::MIP,4);
		//cplextsp.exportModel("modelTSP.lp");
		
		bool loc_problem_is_feasible;
		loc_problem_is_feasible = cplextsp.solve();
		//cplextsp.exportModel("modelTSP.lp");
		
		if (loc_problem_is_feasible == true) {
			putsolutionOnScreen();
			////read_solution(env);
			read_solution(envtsp);
			write_solution_TSP(envtsp);
		}
		else {
			cout << endl;
			cout << "infeasible TSP solution" << '\n';
			mistake_reported = true;
			mistake_text = "infeasible_TSP_solution, ";
		};
	}
	catch (IloException& ex) {
		cerr << "CplexTSP Error: " << ex << '\n';
	}
	catch (...) {
		cerr << "ErrorTSP " << '\n';
	}
	envtsp.end();
}
;

void Model_TSP::buildVariables(IloEnv& env) {

	
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

void Model_TSP::initialize() {
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

	for (int loc_i = 1; loc_i < instance.n_nodes; loc_i++) {
		if ((customers_to_visit[loc_i] == true) && (loc_i != curr_drone_node)) {//we have already taken the current position into account
			references_to_customers[number_nodes_to_visit] = loc_i;
			number_nodes_to_visit = number_nodes_to_visit + 1;
		};
	};


	//if (curr_drone_node != 0){
	references_to_customers[number_nodes_to_visit] = -1;//artificial sink node. No cost to return to this node
	number_nodes_to_visit = number_nodes_to_visit + 1;
	//};
	if(arc_based_tsp==true){
		//initialize visited truck edges
		for(int i=0;i<instance.n_nodes;i++) {
			for(int j=1;j<instance.n_nodes;j++) {
				if(customers_to_visit[i]==false) {
					unvisited_truck_edges[i][j] = -1;
					unvisited_truck_edges[j][i] = -1;
					unvisited_truck_edges[i][i] = -1;
				}
			}
		}
	}	
	
};

void Model_TSP::buildObjective(IloEnv& env) {
	// Building objective
	IloExpr objective(env);
	// Adding coefficients to objective
	for (int k = 0; k < number_nodes_to_visit; ++k) {
		for (int l = k + 1; l < number_nodes_to_visit; ++l) {
			if ((starts_from_depot == true) || ((k != 0) && (l != (number_nodes_to_visit - 1)))) {
				int first_node = references_to_customers[k];
				int second_node = references_to_customers[l];
				objective += instance.drone_distances[first_node][second_node] * Xt[k][l];
				objective += instance.drone_distances[second_node][first_node] * Xt[l][k];
			};
			if ((starts_from_depot == false) && (k == 0)) {
				int first_node = references_to_customers[k];
				if (references_to_customers[l] != -1) {
					int second_node = references_to_customers[l];
					objective += instance.drone_distances[first_node][second_node] * Xt[k][l];
				}
				else {
					//since the cost is 0
				};
			}
			else {
				if ((starts_from_depot == false) && (l == (number_nodes_to_visit - 1))) {//since the cost is 0! (the last node is artificial node) Why is the cost zero for the last edge? Check it!
					int first_node = references_to_customers[k];
					int second_node = 0;
					objective += instance.drone_distances[first_node][second_node] * Xt[k][l];
				};
			};
		};
	};
	cout << objective << '\n';
	// Adding abjective to model
	modeltsp.add(IloMinimize(env, objective));
};


void Model_TSP::buildConstraintsNaive(IloEnv& env) {

	//one outgoing edge
	for (int i = 0; i < number_nodes_to_visit; ++i) {
		IloExpr constraint(env);
		if ((i != (number_nodes_to_visit - 1)) || (starts_from_depot == true)) {
			for (int j = 1; j < number_nodes_to_visit; j++) {//cannot retirn to source node (Hamiltonian path)
				if (i != j) {
					constraint += Xt[i][j];
				};
			};
			IloConstraint c(constraint == 1);
			c.setName("1 outgoing edge");
			modeltsp.add(c);
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
			IloConstraint c(constraint == 1);
			c.setName("1 incoming edge");
			modeltsp.add(c);
		};

	};

	//Miller-Tucker-Zemlin condition
	for (int i = 1; i < number_nodes_to_visit; ++i) {
		for (int j = 1; j < number_nodes_to_visit; ++j) {
			if ((i != j) && ((starts_from_depot == true) || ((i != (number_nodes_to_visit - 1)) && (j != 0)))) {
				IloExpr constraint(env);
				constraint += number_nodes_to_visit * Xt[i][j] + Ut[i] - Ut[j];
				IloConstraint c(constraint <= number_nodes_to_visit - 1);
				c.setName("MTZ constraints");
				modeltsp.add(c);
			};
		};
	};

	////Technical: numbering of nodes only positive and up to N
	//for (int loc_i = 0; loc_i < number_nodes_to_visit; ++loc_i){
	//	IloConstraint c(Ut[loc_i] >= 0);
	//	c.setName("nonnegativity");
	//	modeltsp.add(c);
	//	IloConstraint cb(Ut[loc_i] <= number_nodes_to_visit-1);
	//	cb.setName("up to N");
	//	modeltsp.add(cb);
	//};

};



//Improved TSP
void Model_TSP::buildConstraintsEdges(IloEnv& env) {

	//one node visited of edge
	for (int i = 0; i < number_nodes_to_visit-1; ++i) {
		for (int j = i+1; j < number_nodes_to_visit; j++) {//cannot return to source node (Hamiltonian path)
			int ref_node_i = references_to_customers[i];
			int ref_node_j;
			if(j==number_nodes_to_visit-1) {
				ref_node_j=curr_truck_node;
			}
			else{
				ref_node_j=references_to_customers[j];
			}
			if (unvisited_truck_edges[ref_node_i][ref_node_j]==1 || unvisited_truck_edges[ref_node_j][ref_node_i] == 1) {
				IloExpr constraint(env);
				for (int k = 0; k < number_nodes_to_visit-1; k++) {
					if(k!=i) {
						constraint += Xt[k][i];
					}
					if(k!=j) {
						constraint +=  Xt[k][j];
					}
				}
				IloConstraint c(1 <= constraint);
				c.setName(("visited one node of the edge (" + to_string(ref_node_i)+ "," + to_string(ref_node_j)+")").c_str());
				modeltsp.add(c);
			}
		}
	}

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
        modeltsp.add(c);

	};

	//one outgoing edge for source
    int temp_i = 0;
    IloExpr constraint1(env);
    for (int j = 1; j < number_nodes_to_visit; j++) {//cannot retirn to source node (Hamiltonian path)
        constraint1 += Xt[temp_i][j];
    };
    IloConstraint c1(constraint1 == 1);
    c1.setName("1 outgoing edge source");
    modeltsp.add(c1);


	//no incoming edge for source
	//redundant constraint
    temp_i = 0;
    IloExpr constraint2(env);
    for (int j = 1; j < number_nodes_to_visit; j++) {//cannot retirn to source node (Hamiltonian path)
        constraint2 += Xt[j][temp_i];
    };
    IloConstraint c2(constraint2 == 0);
    c2.setName("1 outgoing edge source");
    modeltsp.add(c2);

	//one incoming edge for sink
    IloExpr constraint3(env);
    temp_i = number_nodes_to_visit - 1;
    for (int j = 1; j < (number_nodes_to_visit - 1); j++) {//cannot start from sink node (Hamiltonian path)
            constraint3 += Xt[j][temp_i];
    };
    IloConstraint c3(constraint3 == 1);
    c3.setName("1 incoming edge sink");
    modeltsp.add(c3);


	//no outgoing edge for sink
	//redundant constraint
    IloExpr constraint4(env);
    temp_i = number_nodes_to_visit - 1;
    for (int j = 1; j < (number_nodes_to_visit - 1); j++) {//cannot start from sink node (Hamiltonian path)
            constraint4 += Xt[temp_i][j];
    };
    IloConstraint c4(constraint4 == 0);
    c4.setName("1 incoming edge sink");
    modeltsp.add(c4);
	

	//Miller-Tucker-Zemlin condition
	for (int i = 1; i < number_nodes_to_visit; ++i) {
		for (int j = 1; j < number_nodes_to_visit; ++j) {
			if ((i != j) && ((starts_from_depot == true) || ((i != (number_nodes_to_visit - 1)) && (j != 0)))) {
				IloExpr constraint(env);
				constraint += number_nodes_to_visit * Xt[i][j] + Ut[i] - Ut[j];
				IloConstraint c(constraint <= number_nodes_to_visit - 1);
				c.setName("MTZ constraints");
				modeltsp.add(c);
			};
		};
	};

	////Technical: numbering of nodes only positive and up to N
	//for (int loc_i = 0; loc_i < number_nodes_to_visit; ++loc_i){
	//	IloConstraint c(Ut[loc_i] >= 0);
	//	c.setName("nonnegativity");
	//	modeltsp.add(c);
	//	IloConstraint cb(Ut[loc_i] <= number_nodes_to_visit-1);
	//	cb.setName("up to N");
	//	modeltsp.add(cb);
	//};

};


void Model_TSP::putsolutionOnScreen() {
	cout << cplextsp.getStatus();
	cout << " ";
	cout << cplextsp.getObjValue();
	cout << '\n';
	//cout << cplex.getBestObjValue();


};

void Model_TSP::read_solution(IloEnv& env) {
	for (int loc_i = 0; loc_i < number_nodes_to_visit; loc_i++) {
		for (int loc_j = 1; loc_j < number_nodes_to_visit; loc_j++) {
			if(loc_i!=loc_j) {
				if (cplextsp.isExtracted(Xt[loc_i][loc_j]) == true) {
					double xx = cplextsp.getValue(Xt[loc_i][loc_j]);
					if (xx > 0.5) {
						cout << "; edge";
						cout << references_to_customers[loc_i];
						cout << "_";
						cout << references_to_customers[loc_j];
						cout << "   " << Xt[loc_i][loc_j];
						cout << '\n';
					};
				};
			};
		};
	};

	for (int loc_i = 0; loc_i < number_nodes_to_visit; loc_i++) {
		if (cplextsp.isExtracted(Ut[loc_i]) == true) {
			double xx = cplextsp.getValue(Ut[loc_i]);
			cout << "; node";
			cout << references_to_customers[loc_i];
			cout << " numbering ";
			cout << xx;
			cout << '\n';
		};
	};


};

void Model_TSP::write_solution_TSP(IloEnv& env) {
	if ((cplextsp.getStatus() == 2) || (cplextsp.getStatus() == 1)) {
		model_solution.obj_value = cplextsp.getObjValue();
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
				if (cplextsp.isExtracted(Xt[loc_i][loc_j]) == true) {
					double xx = cplextsp.getValue(Xt[loc_i][loc_j]);
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