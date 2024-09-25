#ifndef MODEL_OP_HEADER
#define MODEL_OP_HEADER

#include "Instance.h"
#include "Model.h"
#include "Solution.h"
#include <ilcplex/ilocplex.h>
#include <deque>
ILOSTLBEGIN

using namespace std;



class Model_OP : public Model {
public:
	IloModel modelop;
	IloCplex cplexop;
    double c_start; //elapsed surveillance time after replanning
    double c_max; //max surveillance duration
	int tilim;
    vector<double>w_node; //weight of the prize depending on the distance to the depot, normalized by max distance 
	vector<int> p_node; //the price depending on the outgoing edges of the node
	vector<int>references_to_customers;//since not all customers have to be visited, store here actual numbering of customers to be visited
	int number_nodes_to_visit;
	bool starts_from_depot = false;//whether the start is from the depot	//!!!!Instruental, because return to deport not needed!!!!

	IloArray<IloBoolVarArray> Xt;//arcs in the drone surveillance tour
	IloIntVarArray Ut;//the order of visiting the nodes by the drone



public:
	//NOTE: curr_position - is position of a vehicle AND a drone. Hence, replanning 
	//!!!!return to deport not needed!!!!
	//n_stage : number of the stages, which influence the number of decision variables in the model
	Model_OP(Instance& instance, vector<vector<int>>& arc_status_surveillance, vector<bool>customers_to_visit, bool manhattan_truck, int curr_truck_node,int curr_drone_node,double c_start, double c_max,int tilim);
	void solve();


private:
	void initialize();
	void buildVariables(IloEnv& env);
	// Initialize objective
	void buildObjective(IloEnv& env);
    bool check_if_add_edge_to_prize(int i, int j);

	void buildConstraints(IloEnv& env);// short model: the operation (drone sortee) cannot extend over several truck deliveries.
	void putsolutionOnScreen();
	void read_solution(IloEnv& env);
	void write_solution_TSP(IloEnv& env);
};
#endif