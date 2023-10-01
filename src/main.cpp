#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <time.h>
#include <fstream>

#include "local_search.h"

#include "problem.h"
#include "ant_colony.h"

unsigned int T_wd = 100;
unsigned int n_ts = 50;
double t_ts = (double)T_wd / (double)n_ts;

bool timeslice_over(const std::chrono::high_resolution_clock::time_point &time_0, unsigned int timeslice, double t_ts)
{
    // We compute the time elapsed since the begining of the working day to find if we are still in the timeslice
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - time_0;

    return elapsed.count() >= timeslice * t_ts;
}

double elapsed_since(const std::chrono::high_resolution_clock::time_point &time)
{
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - time;

    return elapsed.count();
}

int main()
{

    srand(time(NULL));

    std::string filepath = "../benchmarks/vanveen/rc101-0.7.txt";
    Problem problem = Problem(filepath, T_wd, n_ts);
    auto diff = problem.update(0);

    // For plotting
    // problem.visual_dump_data();
    problem.dump_to_file("../data/problem_data.txt");

    AntColony ant_colony = AntColony(&problem, 10, 1, 1, 0.9, 0.1);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    unsigned int counter = 0;

    auto time_0 = std::chrono::high_resolution_clock::now();
    unsigned int timeslice = 1;

    while (elapsed_since(time_0) < 75)
    {

        std::cout << "Starting timeslice " << timeslice << "." << std::endl;

        unsigned int ant_colony_steps_counter = 0;
        while (!timeslice_over(time_0, timeslice, t_ts))
        {
            ant_colony.step();
            ant_colony_steps_counter++;
        }

        std::cout << "Ant Colony stepped " << ant_colony_steps_counter << " times." << std::endl;

        auto best_solution = ant_colony.get_best_solution();
        auto best_solution_score = ant_colony.get_best_solution_score();

        std::cout << "The current best solution score is " << best_solution_score << "." << std::endl;

        // for (auto &tour_atom : best_solution)
        // {
        //     std::cout << "( " << tour_atom.node_id << ", " << tour_atom.load << ", " << tour_atom.end_of_service << ", " << tour_atom.distance << " )" << std::endl;
        // }

        // Here we compute which nodes from the current solution are to be committed
        // A node from the best solution is committed if the servicing time of the vehicle serving it falls within the next t_ts seconds
        unsigned int current_vehicle_number = best_solution[0].node_id - problem.get_num_customers();
        for (auto i = 1; i < best_solution.size(); i++)
        {
            unsigned int node_id = best_solution[i].node_id;

            if (problem.is_node_depot(node_id))
            {
                current_vehicle_number = node_id - problem.get_num_customers();
            }
            else
            {
                // We check the servicing time of the last commitment ;
                // it needs to fall in the next step for the node to be committed
                if (best_solution[i - 1].end_of_service < (timeslice + 1) * t_ts)
                {
                    // We check if the node has not already been committed
                    if (!problem.has_c_node_been_committed(node_id))
                    {
                        problem.commit(node_id, current_vehicle_number);
                        std::cout << "Commitment of node " << node_id << " to vehicle " << current_vehicle_number << std::endl;
                    }
                }
            }
        }

        // <DEBUG>
        // Dump the data for this timeslice
        std::ofstream timeslice_data_file;
        std::string filename = "data/timeslice_" + std::to_string(timeslice) + ".txt";
        timeslice_data_file.open(filename);
        auto available_c_nodes_ids = problem.get_available_c_nodes_ids();
        for (auto &available_c_node_id : available_c_nodes_ids)
        {
            timeslice_data_file << available_c_node_id << ", ";
        }
        timeslice_data_file << std::endl;
        auto committed_c_nodes_ids = problem.get_committed_c_nodes_ids();
        for (auto &committed_c_node_id : committed_c_nodes_ids)
        {
            timeslice_data_file << committed_c_node_id << ", ";
        }
        timeslice_data_file << std::endl;
        for (auto &tour_atom : best_solution)
        {
            timeslice_data_file << tour_atom.node_id << ", " << tour_atom.load << ", " << tour_atom.end_of_service << ", " << tour_atom.distance << std::endl;
        }
        timeslice_data_file.close();
        // </ DEBUG>

        //std::cout << "Before insertion of new nodes " << ant_colony.get_best_solution_score() << std::endl;

        // We update the problem for the new timeslice
        diff = problem.update((timeslice + 1) * t_ts);
        std::cout << "There are " << diff.size() << " new customers available." << std::endl;

        if (diff.size() != 0)
        {
            // std::cout << diff.size() << " new nodes" << std::endl;
            ant_colony.update_solution();
        }

        //std::cout << "After insertion of new nodes : " << ant_colony.get_best_solution_score() << std::endl;

        std::cout << "Ending timeslice " << timeslice << "." << std::endl;

        timeslice++;

        // For plotting
        // ant_colony.visual_dump_data();
    }

    std::cout << "Score of working day's solution : " << ant_colony.get_best_solution_score() << std::endl;

    // We can scale it back like that because of norms properties ( || \alpha x|| = |\alpha| ||x||)
    std::cout << "Scaled back : " << ant_colony.get_best_solution_score() / problem.get_scaling_factor() << std::endl;
}
