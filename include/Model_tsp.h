#ifndef MODEL_TSP_HEADER
#define MODEL_TSP_HEADER

#include "Instance.h"
#include "Model.h"
#include "Solution.h"
#include <ilcplex/ilocplex.h>
#include <deque>
ILOSTLBEGIN

using namespace std;



class Model_TSP : public Model {
public:
	IloModel modeltsp;
	IloCplex cplextsp;
	int tilim;
	bool arc_based_tsp;
	vector<int>references_to_customers;//since not all customers have to be visited, store here actual numbering of customers to be visited
	int number_nodes_to_visit;
	bool starts_from_depot = false;//whether the start is from the depot	//!!!!Instruental, because return to deport not needed!!!!
	vector<vector<int>> unvisited_truck_edges;
	IloArray<IloBoolVarArray> Xt;//arcs in the drone surveillance tour
	IloIntVarArray Ut;//the order of visiting the nodes by the drone



public:
	//NOTE: curr_position - is position of a vehicle AND a drone. Hence, replanning 
	//!!!!return to deport not needed!!!!
	//n_stage : number of the stages, which influence the number of decision variables in the model
	Model_TSP(Instance& instance,vector<vector<int>>& truck_edges_to_check, vector<vector<int>>& arc_status_surveillance, vector<bool>customers_to_visit, bool manhattan_truck, int curr_truck_node,int curr_drone_node, bool arc_based_tsp, int tilim); //default values bool manhattan_truck=true, int curr_truck_node = 0,int curr_drone_node=0,int tilim=1800, bool arc_based_tsp=false
	void solve();


private:
	void initialize();
	void buildVariables(IloEnv& env);
	// Initialize objective
	void buildObjective(IloEnv& env);

	void buildConstraintsNaive(IloEnv& env);// short model: the operation (drone sortee) cannot extend over several truck deliveries.
	void buildConstraintsEdges(IloEnv& env);// short model: the operation (drone sortee) cannot extend over several truck deliveries.
	void putsolutionOnScreen();
	void read_solution(IloEnv& env);
	void write_solution_TSP(IloEnv& env);
};
#endif