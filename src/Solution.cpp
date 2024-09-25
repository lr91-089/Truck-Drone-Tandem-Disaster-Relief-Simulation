#include "Solution.h"



Solution::Solution(double _obj_value, int n_nodes) :
	obj_value(_obj_value),number_of_nodes(n_nodes)
{
	;
}

//if sortie=true return the sortie time of drone
//if sortie=false return the travel time of drone without sorties
double Solution::get_route_travel_time(bool for_truck, bool sortie)//sortie=false)
{
    double dist_truck = 0.0;
	double dist_drone = 0.0;
	vector<bool> drone_cust_visited = vector<bool>(number_of_nodes,false);
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			double dist_truck_op = 0.0;
			int first_node = truck_sorties[loc_i][0];
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					dist_truck_op = dist_truck_op + truck_distances[first_node][second_node];
					first_node = second_node;
				}
			}
			dist_truck = dist_truck + dist_truck_op;
			first_node = drone_sorties[loc_i][0];
			if ((int) drone_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)drone_sorties[loc_i].size(); loc_j++) {
					int second_node = drone_sorties[loc_i][loc_j];
					if(dist_truck_op>0.0) {
						if(sortie==false) {
							//only track duration if customer not visited
							if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
								if(drone_cust_visited[drone_customers[loc_i].front()]==false){
									dist_drone = dist_drone + drone_distances[first_node][second_node];
								}
							}
							else{
								dist_drone = dist_drone + drone_distances[first_node][second_node];
							}
						}
					}
					else{
						if(sortie==true) {
							if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
								if(drone_cust_visited[drone_customers[loc_i].front()]==false){
									dist_drone = dist_drone + drone_distances[first_node][second_node];
								}
							}
							else{
								dist_drone = dist_drone + drone_distances[first_node][second_node];
							}
						}
					}
					drone_cust_visited[second_node] = true;
					first_node = second_node;
				}
			}
		}
	};
	if(for_truck==true){
		return dist_truck;
	}			 
	return dist_drone;
}

double Solution::get_route_tandem_travel_time()//sortie=false)
{
    double dist_truck = 0.0;
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			double dist_truck_op = 0.0;
			int first_node = truck_sorties[loc_i][0];
			if ((int)drone_customers[loc_i].front() < 0){
				if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
					for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
						int second_node = truck_sorties[loc_i][loc_j];
						dist_truck_op = dist_truck_op + truck_distances[first_node][second_node];
						first_node = second_node;
					}
				}
			}
			dist_truck = dist_truck + dist_truck_op;

		}
	};
	return dist_truck;
}

double Solution::get_route_waiting_time(bool for_truck)
{
	double wait_truck = 0.0;
	double wait_drone = 0.0;
	vector<bool> drone_cust_visited = vector<bool>(number_of_nodes,false);
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			int first_node = truck_sorties[loc_i][0];
			double truck_op_time = 0.0;
			double drone_op_time = 0.0;
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					truck_op_time = truck_op_time + truck_distances[first_node][second_node];
					first_node = second_node;
				}
			}
			first_node = drone_sorties[loc_i][0];
			if ((int)drone_sorties[loc_i].size() > 1)  {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)drone_sorties[loc_i].size(); loc_j++) {
					int second_node = drone_sorties[loc_i][loc_j];
					//only track duration if customer not visited
					if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
						if(drone_cust_visited[drone_customers[loc_i].front()]==false){
							drone_op_time = drone_op_time + drone_distances[first_node][second_node];
						}
					}
					else{
						drone_op_time = drone_op_time + drone_distances[first_node][second_node];
					}
					drone_cust_visited[second_node] = true;
					first_node = second_node;
				}
				//only update drone waiting time, if travel stage
				if (truck_op_time>drone_op_time) {
					wait_drone = wait_drone-drone_op_time+truck_op_time;
				};
			}
			if (truck_op_time<drone_op_time){
				wait_truck = wait_truck+drone_op_time-truck_op_time;
			};
		}
	};
	if(for_truck==true){
		return wait_truck;
	}			 
	return wait_drone;
    
}




vector<double> Solution::get_op_travel_time_vec(bool for_truck)
{
    vector<double> dist_truck = vector<double>();
	vector<double> dist_drone = vector<double>();
	vector<bool> drone_cust_visited = vector<bool>(number_of_nodes,false);
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			int first_node = truck_sorties[loc_i][0];
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				double dist_truck_op = 0.0;
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					dist_truck_op = dist_truck_op + truck_distances[first_node][second_node];
					first_node = second_node;
				}
				dist_truck.push_back(dist_truck_op);
			}
			first_node = drone_sorties[loc_i][0];
			if ((int)drone_sorties[loc_i].size() > 1) {//it is a traveling stage
				double dist_drone_op= 0.0;
				for (int loc_j = 1; loc_j < (int)drone_sorties[loc_i].size(); loc_j++) {
					int second_node = drone_sorties[loc_i][loc_j];
					//only track duration if customer not visited
					if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
						if(drone_cust_visited[drone_customers[loc_i].front()]==false){
							dist_drone_op = dist_drone_op + drone_distances[first_node][second_node];
						}
					}
					else{
						dist_drone_op = dist_drone_op + drone_distances[first_node][second_node];
					}
					drone_cust_visited[second_node] = true;
					first_node = second_node;
				}
				dist_drone.push_back(dist_drone_op);
			}
		}
	};
	if(for_truck==true){
		return dist_truck;
	}			 
	return dist_drone;
}



vector<double> Solution::get_op_waiting_time_vec(bool for_truck)
{
    vector<double> wait_truck = vector<double>();
	vector<double> wait_drone = vector<double>();
	vector<bool> drone_cust_visited = vector<bool>(number_of_nodes,false);
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			int first_node = truck_sorties[loc_i][0];
			double truck_op_time = 0.0;
			double drone_op_time = 0.0;
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					truck_op_time = truck_op_time + truck_distances[first_node][second_node];
					first_node = second_node;
				}
			}
			first_node = drone_sorties[loc_i][0];
			if ((int)drone_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)drone_sorties[loc_i].size(); loc_j++) {
					int second_node = drone_sorties[loc_i][loc_j];
					if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
						if(drone_cust_visited[drone_customers[loc_i].front()]==false){
							drone_op_time = drone_op_time + drone_distances[first_node][second_node];
						}
					}
					else{
						drone_op_time = drone_op_time + drone_distances[first_node][second_node];
					}
					drone_cust_visited[second_node] = true;
					first_node = second_node;
				}
				//only update drone waiting time, if travel stage
				if (truck_op_time>drone_op_time) {
					wait_drone.push_back(truck_op_time-drone_op_time);
				}
				else{
					wait_drone.push_back(0.0);
				}
			}
			if (truck_op_time<drone_op_time) {
				wait_truck.push_back(drone_op_time-truck_op_time);
			}
			else{
				wait_truck.push_back(0.0);
			}
		}
	}
	if(for_truck==true){
		return wait_truck;
	}			 
	return wait_drone;
}


vector<double> Solution::get_op_travel_and_waiting_time_vec(bool for_truck)
{
    vector<double> truck_vec = vector<double>();
	vector<double> drone_vec = vector<double>();
	vector<bool> drone_cust_visited = vector<bool>(number_of_nodes,false);
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			int first_node = truck_sorties[loc_i][0];
			double truck_op_time = 0.0;
			double drone_op_time = 0.0;
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					truck_op_time = truck_op_time + truck_distances[first_node][second_node];
					first_node = second_node;
				}
			}
			first_node = drone_sorties[loc_i][0];
			if ((int)drone_sorties[loc_i].size() > 1) {//it is a traveling stage
				for (int loc_j = 1; loc_j < (int)drone_sorties[loc_i].size(); loc_j++) {
					int second_node = drone_sorties[loc_i][loc_j];
					//only track duration if customer not visited
					if(last_delivered_customer_obj2 >-1 && loc_i==(int) drone_sorties.size()-1){
						if(drone_cust_visited[drone_customers[loc_i].front()]==false){
							drone_op_time = drone_op_time + drone_distances[first_node][second_node];
						}
					}
					else{
						drone_op_time = drone_op_time + drone_distances[first_node][second_node];
					}
					drone_cust_visited[second_node] = true;
					first_node = second_node;
				}
				//only update drone waiting time, if travel stage
				drone_vec.push_back(max(truck_op_time,drone_op_time));
			}
			truck_vec.push_back(max(truck_op_time,drone_op_time));
		}
	};
	if(for_truck==true){
		return truck_vec;
	}			 
	return drone_vec;
}

int Solution::get_damaged_trucks_arcs(vector<vector<int>> current_damages)
{
	int damages = 0;
	if ((int)truck_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)truck_sorties.size(); loc_i++) {
			if ((int)truck_sorties[loc_i].size() > 1) {//it is a traveling stage
				int first_node = truck_sorties[loc_i][0];
				for (int loc_j = 1; loc_j < (int)truck_sorties[loc_i].size(); loc_j++) {
					int second_node = truck_sorties[loc_i][loc_j];
					//check if it is an arc with an artificial node
					if(first_node<(int)current_damages.size() && second_node<(int)current_damages.size()){
						if(current_damages[first_node][second_node]<= -1) {
							damages = damages + 1;
						}
					} 
					first_node = second_node;
				}
			}
		}
	};
	return damages;
}

int Solution::get_drone_riding_truck_ops()
{
    int ops= 0;
	if ((int)drone_sorties.size() > 0) {
		for (int loc_i = 0; loc_i < (int)drone_sorties.size(); loc_i++) {
			if ((int)drone_sorties[loc_i].size() <= 1 && drone_customers[loc_i].back()==-1) {//it is a truck riding stage
				ops = ops+1;
			}
		}
	};		 
	return ops;
}

;