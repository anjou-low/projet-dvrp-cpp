#include <vector>
#include <iostream>
#include "ant_colony.h"
#include "tour_atom.h"
#include "ant.h"
#include <algorithm>
#include "local_search.h"
AntColony::AntColony(Problem *problem, unsigned int num_ants, float alpha, float beta, float q_0, float rho) : problem{problem}, num_ants{num_ants}, alpha{alpha}, beta{beta}, q_0{q_0}, rho{rho}
{
    // We create an initial solution using Nearest Neighbour to get tau_0
    Ant ant = Ant(problem);
    std::vector<TourAtom> initial_solution = ant.construct_solution_nn();
    float initial_solution_score = compute_solution_score(initial_solution);
    best_solution_score = initial_solution_score;

    // We compute tau_0 following Gambardella 1999
    tau_0 = (float)1 / ((float)problem->get_num_available_nodes() * initial_solution_score);

    // <DEBUG>
    // Print initial solution
    std::cout << "Solution @ AntColony initialization: " << std::endl;
    for (auto &tour_atom : initial_solution)
    {
        std::cout << "(" << tour_atom.node_id << ", " << tour_atom.load << ", " << tour_atom.end_of_service << ", " << tour_atom.distance << ")" << std::endl;
    }

    std::cout << "Score @ AntColony initialization: " << initial_solution_score << std::endl;
    // </ DEBUG>

    // We initialize the pheromons matrix to tau_0
    auto flat_matrix_size = (problem->get_num_nodes() + 1) * (problem->get_num_nodes() + 1);
    pheromon_matrix = std::vector<float>(flat_matrix_size, tau_0);
}

void AntColony::step()
{
    // Create a copy of the global pheromon matrix for local updates
    std::vector<float> local_pheromon_matrix = pheromon_matrix;

    std::vector<std::vector<TourAtom>> solutions;
    std::vector<float> solutions_scores;

    // Try to construct a solution for num_ants ants
    for (auto i = 0; i < num_ants; i++)
    {
        Ant ant = Ant(problem);
        std::vector<TourAtom> acs_solution = ant.construct_solution_acs(this, alpha, beta, q_0, rho);

        // The ant got stuck and wasn't able to complete its solution
        // TODO : Maybe we should still update locally to prevent other ants from following the same path
        if (acs_solution.empty())
        {
            continue;
        }

        float acs_solution_score = compute_solution_score(acs_solution);

        // Update locally
        for (auto i = 1; i < acs_solution.size(); i++)
        {
            unsigned int node_id_i = acs_solution[i - 1].node_id;
            unsigned int node_id_j = acs_solution[i].node_id;

            unsigned int index = node_id_i * (problem->get_num_nodes() + 1) + node_id_j;
            local_pheromon_matrix[index] *= (1. - rho);
            local_pheromon_matrix[index] += rho * tau_0;
        }

        // Local search didn't improve the solution, so the next lines are commented out.
        //Local_search ls = Local_search(*problem, acs_solution);
        // Here we lunch the local search for 30 seconds max.
        //ls.search(30);
        //auto ls_solution = ls.solution_from_search();
        //auto ls_solution_score = ls.compute_solution_score(ls_solution);
        //solutions.push_back(ls_solution);
        //solutions_scores.push_back(ls_solution_score);

        solutions.push_back(acs_solution);
        solutions_scores.push_back(acs_solution_score);
    }

    // If the ants have not found a single feasible solution
    if (solutions.empty())
    {
        return;
    }

    // Find best solution among the constructed ones
    auto index_of_min = std::distance(solutions_scores.begin(), std::min_element(solutions_scores.begin(), solutions_scores.end()));

    // If it is better than the current best solution we should update it
    if (solutions_scores[index_of_min] < best_solution_score)
    {
        best_solution = solutions[index_of_min];
        best_solution_score = solutions_scores[index_of_min];
        tau_0 = best_solution_score;
    }

    // Update globally
    for (auto i = 1; i < best_solution.size(); i++)
    {
        unsigned int node_id_i = best_solution[i - 1].node_id;
        unsigned int node_id_j = best_solution[i].node_id;

        unsigned int index = node_id_i * (problem->get_num_nodes() + 1) + node_id_j;
        pheromon_matrix.at(index) *= (1. - rho);
        pheromon_matrix.at(index) += rho * (1. / best_solution_score);
    }
}

void AntColony::update_solution()
{
    // This method is called after the problem has been updated to insert the new available nodes.
    // We simply use an ant to construct a NN tour (this will take into account the nodes that have just been committed)
    // and set the current best_solution to this construction
    // This method is only call when the diff returned by problem.update is not empty (otherwise we would lose the current best solution for nothing)

    // Initialize an ant
    // ant = Ant(problem);

    auto counter = 1;
    // We try to find an ACS solution
    while (true)
    {
        Ant ant_2 = Ant(problem);
        auto solution = ant_2.construct_solution_acs(this, alpha, beta, q_0, rho);
        if (!solution.empty())
        {
            // We have no choice but to update the current best solution because there are new nodes that we have to take into account
            best_solution = solution;
            best_solution_score = compute_solution_score(solution);
            break;
        }
        counter++;
    }

    // <DEBUG>
    std::cout << "It took " << counter << " iterations to find a feasible solution accounting for the new nodes." << std::endl;
    std::cout << "The score of the initial solution accounting for the new nodes is " << best_solution_score << std::endl;
    // </ DEBUG>

    tau_0 = 1. / ((float)problem->get_num_available_nodes() * best_solution_score);

    // TODO : Do we override the full matrix with the new tau_0 or only the new available nodes arcs ?

    // We have two options here
    // 1. Override the whole pheromon matrix with the new tau_0 -> losing information about previous problems which had less customers
    // 2. Evaporate them

    // Option 1 is better for problems with low dynamicity
    // Option 2 is better for problems with high dynamicity

    // For now we take option 2
    for (auto i = 0; i < pheromon_matrix.size(); i++)
    {
        pheromon_matrix[i] *= (1. - rho);
        pheromon_matrix[i] += rho * tau_0;
    }

    // We override the pheromons matrix to tau_0
    // auto flat_matrix_size = (problem->get_num_nodes() + 1) * (problem->get_num_nodes() + 1);
    // pheromon_matrix = std::vector<float>(flat_matrix_size, tau_0);
}

float AntColony::get_pheromons(unsigned int node_id_i, unsigned int node_id_j)
{
    return pheromon_matrix.at(node_id_i * (problem->get_num_nodes() + 1) + node_id_j);
}

const std::vector<TourAtom> &AntColony::get_best_solution() const
{
    return best_solution;
}

float AntColony::get_best_solution_score() const
{
    return best_solution_score;
}

float AntColony::compute_solution_score(const std::vector<TourAtom> &solution) const
{
    float score = 0;
    float last_distance = 0;

    for (const auto &tour_atom : solution)
    {
        if (problem->is_node_depot(tour_atom.node_id))
        {
            score += last_distance;
        }
        last_distance = tour_atom.distance;
    }

    score += last_distance;

    return score;
}

void AntColony::visual_dump_data() const
{
    for (auto &tour_atom : best_solution)
    {
        std::cout << tour_atom.node_id << std::endl;
    }
}