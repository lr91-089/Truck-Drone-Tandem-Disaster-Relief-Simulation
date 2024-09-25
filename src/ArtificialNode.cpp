#include "ArtificialNode.h"

ArtificialNode::ArtificialNode(int _id, int _n_nodes ,int _first_node, int _second_node, double distance_move, double traveled_duration, double stopped_duration, double _node_service_time, bool _service_node, map<int, ArtificialNode> art_node_map, vector<vector<double>> distance) :
            id(_id), n_nodes(_n_nodes), first_node(_first_node), second_node(_second_node), outgoing_node(_first_node), real_edge({first_node,second_node}), service_time(_node_service_time) ,service_node(_service_node)
{       
        double d_a_second_node = traveled_duration - stopped_duration;
        double d_first_node_a = distance_move - d_a_second_node;
        //if artificial node then create an artificial outgoing edge
        double d_a_out_going_node = std::numeric_limits<double>::infinity();
        if(first_node>=n_nodes) {
                //by definition second node must be real, cannot move to artificial node
                int temp_check_node = second_node;
                if(art_node_map[first_node].artificial_outgoing_edge.first.second!=temp_check_node) {
                        //case same direction as previous node
                        outgoing_node = art_node_map[first_node].artificial_outgoing_edge.first.second;
                }
                else {  
                        //case different direction than previous node
                        outgoing_node = art_node_map[first_node].real_outgoing_edge.first.second;
                }
                double d_pred = distance[first_node][outgoing_node];
                d_a_out_going_node = d_first_node_a+d_pred;
        }
        else {
                //else outgoing_node==first_node
                d_a_out_going_node = d_first_node_a;
        }
        
        real_incoming_edge = {{first_node,id},d_first_node_a}; // <edge(node1,id), distance>
	real_outgoing_edge = {{id, second_node}, d_a_second_node}; ; // <edge(id,node2), distance>
	artificial_outgoing_edge = {{id, outgoing_node},d_a_out_going_node}; // <edge(id,outgoing_node), distance>
        
        //check if already reached second node and begun service time
        if(service_node) {
                //only recompute the incoming edge
                //real_outgoing_edge.second<=service_time --> elapsed service time
                real_incoming_edge.second = service_time- real_outgoing_edge.second;
                //change the edge	
                real_outgoing_edge.first = {id, first_node};
                //no outgoing edge will be needed
                artificial_outgoing_edge.second = std::numeric_limits<double>::infinity();
                second_node = first_node;
                service_time = -service_time;
        }
        else {
                //no service time accounting needed
                service_time = 0.0;
        }
}

ArtificialNode::ArtificialNode() {}

bool ArtificialNode::areEqual(double a, double b)
{
    return fabs(a-b)<0.1;
}


