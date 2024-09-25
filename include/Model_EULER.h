#ifndef MODEL_EULER_HEADER
#define MODEL_EULER_HEADER

#include "Instance.h"
#include "Solution.h"
#include "Model.h"
#include "Simulation.h"
#include "Utilities.h"
#include <ilcplex/ilocplex.h>
#include <deque>
#include <set>
#include <unordered_map>
#include <iostream>
#include <map>

ILOSTLBEGIN

using namespace std;

/*
template <typename Container> // we can make this generic for any container [1]
struct container_hash {
	std::size_t operator()(Container const& c) const {
		return boost::hash_range(c.begin(), c.end());
	}
};
*/



class Model_EULER: public Model {
public:
	IloModel model;
	IloCplex cplex;
	int tilim;//1800-->30 minutes naive ub
	reoptParams& reopt_parameters;
	Solution reopt_solution;
	bool curr_drone_cust_delivered;
	//FOR THE EULER MODEL==> required sbs with loops for initialization
	IloBoolVarArray Xo;//if operation o is used or not
	IloBoolVarArray Yv;//if node v is used or not

	vector<vector<int>>temp_adjacent_nodes;//adjacent nodes for the truck in this input data, first number=the number of adjacent nodes
	//NOTE: for the sake of the model (truck stay at the depot as soon as it arrives there), temp_adjacent_nodes: node 0 has a neighbor 0
private:


public:
	//NOTE: curr_position - is position of a vehicle AND a drone. Hence, replanning 
	//n_stage : number of the stages, which influence the number of decision variables in the model
	Model_EULER(Instance& instance, vector<vector<int>>edge_status, vector<bool>customers_to_visit, bool manhattan_truck, reoptParams& reopt_parameters, int tilim=1800);
	void solve(int setting);
	void reset_euler_model(vector<vector<int>>edge_status, vector<bool>_customers_to_visit, reoptParams& reopt_parameters);
	double evaluate_reopt_distance(int v, int w, vector<vector<double>>& distance);

private:
	
	void putsolutionOnScreen();
	void init_euler_model_struct();
	void add_Reopt_Operations();
	void add_aritficial_operation();
	void buildVariables_Eulermodel(IloEnv& env);
	void buildObjective_Eulermodel(IloEnv& env);
	void buildConstraints_Eulermodel(IloEnv& env);
	void read_solution_complete_Eulermodel(IloEnv& env);
	vector<int> get_ledges_to_visit();
	vector<vector<int>> mod_tour_reopt(vector<int> flag, vector<vector<int>>& truck_sorties);
	vector<int> get_indices_reopt();
	
};




#endif