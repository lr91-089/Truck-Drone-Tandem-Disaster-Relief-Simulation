#ifndef ARTIFICIALNODE_HEADER
#define ARTIFICIALNODE_HEADER

#include <ilcplex/ilocplex.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <utility>
#include <map>
ILOSTLBEGIN

using namespace std;


class ArtificialNode {

public:



	// id of the artificial node
	int id = 0;
	//an artificial node has 3 edges, a real node has 4
	int n_nodes;
	int first_node;
	int second_node;
	int outgoing_node;
	pair<int, int> real_edge;
	pair<pair<int,int>,double> real_incoming_edge; // <edge(node1,id), distance>
	pair<pair<int,int>,double> real_outgoing_edge; // <edge(id,node2), distance>
	pair<pair<int,int>,double> artificial_outgoing_edge; // <edge(id,outgoing_node), distance>
	double service_time;
    bool service_node; //true if service delivery node
	bool truck_sichtfeld; //true if truck sichtfeld node

public:

	/**
	* Constructor for the ArtificialNode::ArtificialNode class to build the node data structure and prepare distances. The standard class is always a Truck node.
	* @method ArtificialNode::ArtificialNode
	* @param  id           node id
	* @param first_node			first node of edge of artificial edge
	* @param second_node 		second node of edge of artificial edge
	* @param service_node true if service_node
	*/
	ArtificialNode(int id, int n_nodes, int first_node, int second_node, double distance_move, double traveled_duration, double stopped_duration, double node_service_time, bool service_node, map<int, ArtificialNode> art_node_map, vector<vector<double>> distance);
	ArtificialNode();
    /**
	* Initialize the distances used for the DP algorithm
	* @method truck_distance
	* @param truck_distances // vector of distances to expand by artificial node
	* @param pred_drone // predecessors of drone nodes, if bool truck_node==false
	*/
    bool areEqual(double a, double b);
    // Method to check if the object is empty
    // is empty if id is equal to depot node
    bool isEmpty() const {
        return id==0;
    }

};


#endif

