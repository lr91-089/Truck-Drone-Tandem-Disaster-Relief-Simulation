#include "Instance.h"
#include "Utilities.h"




Instance::Instance(const string& filename, int ins_num, int _k_arc_damages, double s_drone_speed, double _const_service_time , bool _shortcuts) :
	n_nodes(0), real_n_nodes(0), k_arc_damages(_k_arc_damages),s_drone(s_drone_speed), const_service_time(_const_service_time) , shortcuts(_shortcuts), ins_number(ins_num)
{
	fillFromFile(filename);
	initialize_drone_distances();
	compute_prio_values_of_arcs();
	compute_maxn_stages();
	initialize_customers_bool_and_const_delivery_service_times();
}

Instance::Instance() {}


void Instance::fillFromFile(const string& filename) {
	// Opening the file
	ifstream file;
	//string file2 = "C:\\Users\\un_po\\source\\repos\\drone_truck_disaster\\x64\\Debug\\data\\2001";
	file.open(filename);

	// Processing initial values - header of the file
	std::string s;
	while (!file.eof()) {
		string l;
		getline(file, l);
		trim(l);
		if (l == "<number of nodes>")
			file >> n_nodes;
		//else if (l == "<probability of arc damage, percent>")
		//{
		//	file >> p_damage;

		//}
		else if (l == "<number of real nodes>")
		{
			file >> real_n_nodes;

		}
		else if (l == "<how much the drone is faster than the truck, percent>")
		{
			file >> s_drone;

		}
		else if (l == "<max length of the drone flight>")
		{
			file >> max_drone_sortie;
		}
		else if(l == "<D 2/3>") {
			parking_nodes.resize(0);
			getline(file, l);
			std::stringstream string_stream_line( l );
			int temp_pos;

			while ( string_stream_line >> temp_pos){
  				parking_nodes.push_back( temp_pos -1);
				if (string_stream_line.peek() == ','){
            		string_stream_line.ignore();
				}
			}

		}
		else if(l == "<V>") {
			customer_nodes.resize(0);
			getline(file, l);
			std::stringstream string_stream_line( l );
			int temp_pos;

			while ( string_stream_line >> temp_pos){
  				customer_nodes.push_back( temp_pos -1 );
				if (string_stream_line.peek() == ','){
            		string_stream_line.ignore();
				}
			}
		}
		else if (l == "<coordinates X,Y>")
		{
			coordinates.resize(0);
			bool all_read = false;
			float x = 0.0;
			float y = 0.0;

			while (all_read == false) {
				getline(file, l);
				trim(l);
				if (l == "") {
					all_read = true;
				}
				else
				{
					string temp_string = l;
					l = l.substr(0, l.find(","));
					x = stod(l);
					//x = roundf(x*100) / 100;
					temp_string = temp_string.substr(temp_string.find(",") + 1);
					l = temp_string;
					y = stod(l);
					coordinates.push_back({x,y});
				}
			}
		}
		else if (l == "<adjacency matrix with truck distances>")
		{
			//initialize matrices
			vector<int> temp_vector(n_nodes, -1);
			euclidean_distance_matrix.resize(n_nodes, vector<double>(n_nodes, max_lim));
			manhattan_distance_matrix.resize(n_nodes, vector<double>(n_nodes, max_lim));
			arc_status.resize(0);
			adjacent_nodes.resize(0);

			for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
				//euclidean_distance_matrix.push_back(temp_vector);
				arc_status.push_back(temp_vector);
			};
			arc_status_conservative = arc_status;
			//seems to be useless, just so index+1 loops are possible
			temp_vector.push_back(-1);
			temp_vector[0] = 0;
			for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
				adjacent_nodes.push_back(temp_vector);
			};

			//read the info on the arcs
			bool all_arcs_read = false;

			while (all_arcs_read == false) {
				getline(file, l);
				trim(l);
				if (l == "") {
					all_arcs_read = true;
				}
				else
				{
					string temp_string = l;
					string temp_euclidean_string = l;
					string temp_manhattan = "";
					l = l.substr(0, l.find(","));
					int left_node = stoi(l) - 1;
					temp_string = temp_string.substr(temp_string.find(",") + 1);
					l = temp_string;
					l = l.substr(0, l.find(","));
					int right_node = stoi(l) - 1;
					temp_string = temp_string.substr(temp_string.find(",") + 1);
					temp_euclidean_string = temp_string.substr(0, temp_string.find(","));
					temp_string = temp_string.substr(temp_string.find(",") + 1);
					//Manhattan distance, second value
					temp_manhattan = temp_string.substr(0,temp_string.find(","));
					temp_string = temp_string.substr(temp_string.find(",") + 1);
					l = temp_string;
					int temp_arc_status = stoi(l);
					if (left_node != right_node) {
						euclidean_distance_matrix[left_node][right_node] = stoi(temp_euclidean_string);
						manhattan_distance_matrix[left_node][right_node] = stoi(temp_manhattan);
						//euclidean_distance_matrix[right_node][left_node] = euclidean_distance_matrix[left_node][right_node];
						if (left_node == 0) {
							arc_status[left_node][right_node] = 1; //if connected to depot
							arc_status[right_node][left_node] = 1;
						}
						else {
							arc_status[left_node][right_node] = temp_arc_status;
							arc_status[right_node][left_node] = temp_arc_status;
						};
					}
					//we assume that same node has a distance of zero
					else {
						euclidean_distance_matrix[left_node][right_node] = 0;
						manhattan_distance_matrix[left_node][right_node] = 0;
						arc_status[left_node][right_node] = temp_arc_status;
						arc_status[right_node][left_node] = temp_arc_status;
					};

				}
			};
		}
	}

	// Closing file
	file.close();

	// fill_in the list of adjacent nodes
	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {
			if (arc_status[loc_i][loc_j] >= 0) {
				adjacent_nodes[loc_i][0] = adjacent_nodes[loc_i][0] + 1;
				int temp_int = adjacent_nodes[loc_i][0];
				adjacent_nodes[loc_i][temp_int] = loc_j;
			};
		}
	}


};

void Instance::initialize_drone_distances() {
	vector<double>temp_vector = vector<double>(n_nodes);
	vector<vector<int>> temp_drone_arc_status = arc_status;
	if(shortcuts){
		for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
			for (int loc_j = 0; loc_j < n_nodes; loc_j++) {
				temp_drone_arc_status[loc_i][loc_j] = 1;
			}
		}
	}
	drone_distances.resize(0);
	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
		drone_distances.push_back(temp_vector);
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {
			if(temp_drone_arc_status[loc_i][loc_j]>-1){
				drone_distances[loc_i][loc_j] = (double) euclidean_distance_matrix[loc_i][loc_j] / s_drone;
			}
			else{
				drone_distances[loc_i][loc_j] = max_lim;
			}
		};
	};

	drone_fromtriple_previousnode = vector<vector<int>>(n_nodes, vector<int>(n_nodes));

	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {
			drone_fromtriple_previousnode[loc_i][loc_j] = loc_i;
		};
		drone_distances[loc_i][loc_i] = 0;
		drone_fromtriple_previousnode[loc_i][loc_i] = loc_i;

	};

	//COMPUTATION
	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {//transitknoten
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {//startknoten
			if ((loc_i != loc_j) && (drone_distances[loc_j][loc_i] < max_lim)) {
				for (int loc_k = 0; loc_k < n_nodes; loc_k++) {//endknoten
					if ((drone_distances[loc_i][loc_k] < max_lim) && (drone_distances[loc_j][loc_i] + drone_distances[loc_i][loc_k] < drone_distances[loc_j][loc_k])) {
						drone_distances[loc_j][loc_k] = drone_distances[loc_j][loc_i] + drone_distances[loc_i][loc_k];
						drone_fromtriple_previousnode[loc_j][loc_k] = drone_fromtriple_previousnode[loc_i][loc_k];
					};
				};
			};
		};
	};


	//TEMPORARY
	double loc_max_drone_sortie = 0;
	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {//transitknoten
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {//startknoten
			if ((drone_distances[loc_j][loc_i] >= 0) && (loc_i != loc_j)) {

				if (drone_distances[loc_i][loc_j] > loc_max_drone_sortie) {
					loc_max_drone_sortie = drone_distances[loc_i][loc_j];
				};
			};
		};
	};
	/*
	if (s_drone > 1){
		loc_max_drone_sortie = 2 * loc_max_drone_sortie *s_drone+ 0.5; //check this definition
	}
	else{
		loc_max_drone_sortie = 2 * loc_max_drone_sortie + 0.5;
	};
	*/
	//Annahme drohne ist schneller als Truck und kann alle Knoten besuchen
	// Check if needed or useless?!
	loc_max_drone_sortie = 2 * loc_max_drone_sortie * n_nodes + 0.5;
	if (max_drone_sortie < loc_max_drone_sortie) {
		max_drone_sortie = loc_max_drone_sortie;
	};

};

void Instance::compute_prio_values_of_arcs() {
	vector<int>temp_vector = vector<int>(n_nodes);
	priovalues_arcs.resize(0);
	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
		priovalues_arcs.push_back(temp_vector);
		for (int loc_j = 0; loc_j < n_nodes; loc_j++) {
			priovalues_arcs[loc_i][loc_j] = 0;
		};
	};


	for (int loc_i = 0; loc_i < n_nodes; loc_i++) {
		for (int loc_j = loc_i + 1; loc_j < n_nodes; loc_j++) {
			if (arc_status[loc_i][loc_j] >= 0) {
				priovalues_arcs[loc_i][loc_j] = adjacent_nodes[loc_i][0] + adjacent_nodes[loc_j][0];
				priovalues_arcs[loc_j][loc_i] = priovalues_arcs[loc_i][loc_j];
			};
		};
	};
};

void Instance::compute_maxn_stages() {
	maxn_stages_long = 2 * n_nodes + 1;
	maxn_stages_short = 4 * n_nodes - 3;//Must be nicht ganzzahlig (last stage: truck waiting in the depot). [Justification for 4 * n_nodes-3: in the worst case, the truck delivers to each node hin-zurueck]
}
void Instance::initialize_customers_bool_and_const_delivery_service_times(){
	customers_to_visit = vector<bool>(n_nodes, false);
	for (int loc_i = 0; loc_i < (int) customer_nodes.size(); loc_i++) {
		int temp_customer = customer_nodes[loc_i];
		customers_to_visit[temp_customer] = true;
	}
	
	for (int i = 0; i < n_nodes; ++i) {
		if(customers_to_visit[i]) {
			service_time_map.emplace(i, const_service_time);
		}
		else {
			service_time_map.emplace(i, 0.0);
		}
    }
};