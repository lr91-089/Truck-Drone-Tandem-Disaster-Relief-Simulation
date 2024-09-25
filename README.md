# Truck-Drone-Tandem-Disaster-Relief-Simulation
A repository with the simulation framework and instances used for the submitted paper "On delivery policies for a truck-and-drone tandem in disaster relief".

You are kindly requested to cite this paper if these code or instances are relevant to your research. 

Shield: [![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg

## Instance generation
We generate 200 graphs $G=(L,E)$ with $|L|=16$ nodes, by placing the nodes randomly in a $1000 \times 1000$ square. Afterward, we randomly assign $|V|=5$ nodes to be delivery addresses and one node to be the depot.  
We create sparse graphs since they most resemble real road networks. We keep the graphs connected to ensure feasibility: There is a road between each delivery address and the depot. 
We start with a complete graph and gradually reduce the number of edges to $|E|=36$ by randomly deleting edges that do not disconnect the graph. 

The instances 1601-1700 are used for the main experiments, while the instances 6001-6200 are used for the calibration of the hybrid policy.

## Random seeds

For each graph, we randomly generate 20 instances by setting 22 edges (or $\approx 60$% of $|E|$) to be damaged. The file ```random_seed_list_per_instance.csv``` lists each random seed used for each graph. The first column lists the random seed index. Each column lists all random seeds for the specific graph, e.g. "1601" (see column header).

## Settings analyzed
In the computational experiments we defined 8 settings. In the basic setting (Base), the drone is twice as fast as the truck ($\alpha=2$) and $D:=V\cup\{v_0\}$, i.e., the remaining nodes serve exclusively as surveillance nodes. We set the truck speed to 1.
We examine seven additional settings by changing one factor at a time in Base. In four settings, we vary  the drone speed ($\alpha=1$, $\alpha=3$) and the incidence of the parking lots $(|D|=11, |D|=16)$. 
Three further settings refer to widespread network characteristics in the TSP-D literature: the Euclidean metric ($L_2$) for the truck with drone shortcuts as well as the Manhattan metric ($L_1$) for the truck with and without drone shortcuts.  

We also investigate the influence of a slow drone ($\alpha \leq 1$), influence of the number of damaged edges in the graph and different field of views for truck an drone.

For more information, please see our paper: <insert paper link here>

## Instance usage
Comments within an instance file start with ```<``` and end with ```>``` and have to be ignored by a parser.

Node 1 is the depot ($v_0$) for the truck and drone.
The set of delivery adresses $V$ is defined in the line after ```<V>```.
In the setting where we vary the number of parking lots $|D|=11$, we use the set of nodes defined after ```<D 2/3>``` for $D$. These nodes are randomly assigned with uniform probability and $V \subseteq D$.
The edges of the graph are provided in the ```<adjacency matrix>```, as well as the euclidean distance (ceiling) and manhattan distance (ceiling) of each edge.

## Simulation usage

To replicate the experiments of the paper, please run the three bash files. The input of the bash files is given in the folders: ```paper_bash_files_experiments```, ```paper_calibration_bash_files```, and ```paper_sichtfeld_experiments_bash```. To speed up the computation, the experiments are executed in 20 parallel threads.


