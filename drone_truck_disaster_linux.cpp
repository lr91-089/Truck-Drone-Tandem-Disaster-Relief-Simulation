// drone_truck_diaster_refactored.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>
#include "Instance.h"
#include "Simulation.h"

//#pragma warning(disable : 4996)
//ILOSTLBEGIN

using namespace std;



int main(int argc, char** argv)
{
		
		cmd_inputs_struct cmd_inputs = {};
		cmd_inputs.path_to_folder = "results/";
		cmd_inputs.path_to_results = cmd_inputs.path_to_folder;
		cmd_inputs.path_to_data = "data/";
		cmd_inputs.zusatz_ausgabe_filename = "";
		cmd_inputs.number_inputs = argc;
		cmd_inputs.input_filename = argv[0];
		int temp_int=1;
		double temp_double=0.0;
		string project_name = "drone_truck_disaster_linux";
	
		
		std::string arg;
		cmd_inputs.n_simulations = 1;
		//number of iterations
		try{ 
			if (cmd_inputs.number_inputs > 2) {
				arg = argv[2];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for iterations: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.n_simulations = temp_int;
			}

			//How many arc damages
			if (cmd_inputs.number_inputs > 3) {
				cmd_inputs.input_k_arc_damages = 0;
				arg = argv[3];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for arc damages: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.input_k_arc_damages = temp_int;
			};
			//clustered damage
			if (cmd_inputs.number_inputs > 4) {
				arg = argv[4];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for clusterd damages: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_int > 0 && temp_int < 4) {
					cmd_inputs.input_clustered_damages = temp_int;
				}
				else {
					if (temp_int != 0) {
						cout << "Error: Invalid argument for clustered damages. \n";
						cout << temp_int;
						throw std::invalid_argument("Invalid argument for clustered_damages.");
					}
				}
			};

			//required nodes. 1.0 if all nodes are required
			if (cmd_inputs.number_inputs > 5) {
				temp_double = 0.0;
				try {
					temp_double = atof(argv[5]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.input_required_nodes_proportion = temp_double;
				if (temp_double  > 1.0 || temp_double  < 0.0) {
					cout << "Error: Invalid argument for required nodes proportion. \n";
					throw std::invalid_argument("Invalid argument for required nodes proportion.");
				}
			};
			//drone speed
			if (cmd_inputs.number_inputs > 6) {
				temp_double = 1.0;
				try {
					temp_double = atof(argv[6]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.input_howmuch_drone_is_faster = temp_double;
				if (temp_double  > 3.5 || temp_double  < 0.0) {
					cout << "Error: Invalid argument for drone speed. \n";
					throw std::invalid_argument("Invalid argument for drone speed.");
				}
			}
			else {
				throw std::invalid_argument("Missing drone argument!");
			};
			//cmax
			if (cmd_inputs.number_inputs > 7) {
				temp_double = 1000.0;
				try {
					temp_double = atof(argv[7]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.cmax = temp_double;
				if (temp_double  > 100000.0 || temp_double  < -2.0) {
					cout << "Error: Invalid argument for drone speed. \n";
					throw std::invalid_argument("Invalid argument for cmax.");
				}
			}
			else {
				throw std::invalid_argument("Missing cmax!");
			};
			//ran seed
			long temp_long = 0;
			if (cmd_inputs.number_inputs > 8) {
				arg = argv[8];
				try {
					std::size_t pos;
					temp_long = std::stoll(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for seed argument: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (const std::exception &e) {
					std::cerr << "ERROR " << e.what() << '\n';
				}
				cmd_inputs.ranseed = temp_long;
			}
			else {
				throw std::invalid_argument("Missing seed arguments!");
			};


			//Simulation settings: full_info_settings,surveillance_first_settings,reopt_settings, conservative_settings
			//setting fullinfo
			if (argc > 9) {
				arg = argv[9];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for setting full info: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_full_info = temp_int;
			}
			//setting surveillance
			if (argc > 10) {
				arg = argv[10];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for setting surveillance first: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_surv_first = temp_int;
			}
			//setting surveillance orienteering
			if (argc > 11) {
				arg = argv[11];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for setting surveillance first: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_surv_op = temp_int;
			}
			//setting reopt
			if (argc > 12) {
				arg = argv[12];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for setting reopt: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_reopt = temp_int;
			}
			//setting conservative
			if (argc > 13) {
				arg = argv[13];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number setting conservative: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_conservative = temp_int;
			}
			//shortcuts
			if (argc > 14) {
				arg = argv[14];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for shortcuts: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if(temp_int == 0){
					cmd_inputs.shortcuts = false;
				}
				else{
					cmd_inputs.shortcuts = true;		
				}
				
			}
			//manhattan distance
			if (argc > 15) {
				temp_int = 1;
				arg = argv[15];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for shortcuts: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_int == 1) {	
					cmd_inputs.truck_distance_metric_manhattan = true;
				}
				else {
					cmd_inputs.truck_distance_metric_manhattan = false;		
				}
				
			}

			if (argc > 16) {
				arg = argv[16];
				cmd_inputs.path_to_results = cmd_inputs.path_to_results+arg+"/";
				cmd_inputs.zusatz_ausgabe_filename = arg;
			}
			//drone parking nodes. 1.0 if all nodes are parking nodes
			// customer ratio must be smaller or equal to drone parking nodes!
			if (cmd_inputs.number_inputs > 17) {
				temp_double = 1.0;
				try {
					temp_double = atof(argv[17]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_double  > 1.0 || temp_double  < 0.0 || temp_double < cmd_inputs.input_required_nodes_proportion) {
					cout << "Error: Invalid argument for drone parking nodes! \n";
					throw std::invalid_argument("Invalid argument for drone parking nodes.");
				}
				cmd_inputs.input_parking_nodes = temp_double;
				
			};

			//penalty_factor
			cmd_inputs.penalty_factor = 1.0;
			if (argc > 18) {
				temp_double = 1.0;
				try {
					temp_double = atof(argv[18]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_double  < 1.0 || temp_double  > 10.0) {
					cout << "Error: Invalid argument for penalty_factor value! \n";
					throw std::invalid_argument("Invalid argument for penalty_factor value.");
				}
				cmd_inputs.penalty_factor = temp_double;
			}

			//service time
			cmd_inputs.service_time = 0.0;
			if (argc > 19) {
				temp_double = 0.0;
				try {
					temp_double = atof(argv[19]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_double  < 0.0) {
					cout << "Error: Invalid argument for service time value! \n";
					throw std::invalid_argument("Invalid argument for service time value.");
				}
				cmd_inputs.service_time = temp_double;
			}

			//service time
			cmd_inputs.cmax_ratio = 0.0;
			if (argc > 20) {
				temp_double = 0.75;
				try {
					temp_double = atof(argv[20]);
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_double  < 0.0 || temp_double  > 1.0) {
					cout << "Error: Invalid argument for cmax ratio value! \n";
					throw std::invalid_argument("Invalid argument for cmax ratio value.");
				}
				cmd_inputs.cmax_ratio = temp_double;
			}

			//sichtfeld_setting
			//0 no sichtfeld
			//1 uniform
			//2 midpoint
			//3 fourth
			cmd_inputs.setting_sichtfeld = 0;
			if (argc > 21) {
				temp_int = 0;
				arg = argv[21];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for shortcuts: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				cmd_inputs.setting_sichtfeld = temp_int;
				
			}

			//output tours
			if (argc > 22) {
				temp_int = 1;
				arg = argv[22];
				try {
					std::size_t pos;
					temp_int = std::stoi(arg, &pos);
					if (pos < arg.size()) {
						std::cerr << "Trailing characters after number for shortcuts: " << arg << '\n';
					}
				}
				catch (std::invalid_argument const& ex) {
					std::cerr << "Invalid number: " << arg << '\n';
				}
				catch (std::out_of_range const& ex) {
					std::cerr << "Number out of range: " << arg << '\n';
				}
				if (temp_int == 1) {	
					cmd_inputs.setting_debug = true;
				}
				else {
					cmd_inputs.setting_debug = false;		
				}
				
			}

			cmd_inputs.ins_num = atoi(argv[1]);
			//std::cout << "start of run!\n";
			string loc_filename = cmd_inputs.path_to_data + to_string(cmd_inputs.ins_num);
			//loc_filename = loc_filename + ".tspd";
			Instance instance(loc_filename, cmd_inputs.ins_num, cmd_inputs.input_k_arc_damages, cmd_inputs.input_howmuch_drone_is_faster, cmd_inputs.service_time, cmd_inputs.shortcuts);


			/*
			Optimistic case
			default deactivated e.g. -1 or other number
			0 = model_tsp
			1 = model_relaxed
			2 = model
			*/



			/*
			*Naive solution method heuristic is activated default case!
			default: deactivate naive solution e.g. -1 or other number than 0 or 1
			0 = Naïve solution heuristic
			1 = Naïve solution Cplex model
			*/

			/*
			*Full info solution
			0 = Full-info heuristic
			1 = Full-info Cplex model
			default deactivated e.g. -1 or other number
			*/

			/*
			* Surveillance first
			0 = Surveillance-first heuristic
			1 = Surveillance-first Cplex model
			default deactivated e.g. -1 or other number
			*/

			/*Reopt settings
			0 = Model_heuristic_smallsorties Model_heuristic_smallsorties
			1 = Model_SMALLSORTIES Model_SMALLSORTIES
			default deactivated e.g. -1 or other number
			*/

			/*Truck and drone delivery conservative  (probably not done yet)
			1 = Model
			default deactivated e.g. -1 or other number
			*/

			/*
			Surveillance-first with ADP (irrelevant for the current project)
			*/

			//AlWAYS check the globparameters. Some of them are initialized in instance.cpp
			

			Simulation simulation(instance, cmd_inputs);
			
	}
	catch(const std::exception &e){
		std::ofstream OFile;
		string locfilename = cmd_inputs.path_to_folder+"error_log.csv";

		OFile.open(locfilename, std::ios_base::app);


		if (OFile.is_open()) {

			OFile << e.what()<< '\n';
			};//if file is open

		OFile.close();
		std::cout << "ERROR OCCURED! " << e.what() << "\n";	
		return 1;
	}
	std::cout << "End of the run!\n";
	return 0;	
} 	