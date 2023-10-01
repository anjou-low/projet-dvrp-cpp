#include <vector>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <random>
#include "ant.h"
#include "ant_colony.h"
#include "tour_atom.h"

Ant::Ant(Problem *problem) : problem{problem}, num_visited_customers{0}, current_vehicle_number{0}, current_time{0}, current_load{0}, current_distance{0}
{
}

std::vector<TourAtom> Ant::construct_solution_nn()
{
    // We first initialize the Ant to a random starting depot
    initialize_tour();

    // While not all customers have been visited we keep adding arcs
    while (num_visited_customers < problem->get_num_available_customers())
    {
        std::vector<unsigned int> candidate_nodes_ids = compute_candidate_arcs();

        if (candidate_nodes_ids.size() == 0)
        {
            //std::cout << "No feasible solution die!" << std::endl;
            return {};
        }

        unsigned int selected_node_id = select_arc_nn(candidate_nodes_ids);

        insert_selected_arc(selected_node_id);
    }

    return solution;
}

std::vector<TourAtom> Ant::construct_solution_acs(AntColony *ant_colony, float alpha, float beta, float q_0, float rho)
{
    initialize_tour();

    while (num_visited_customers < problem->get_num_available_customers())
    {
        std::vector<unsigned int> candidate_nodes_ids = compute_candidate_arcs();

        if (candidate_nodes_ids.size() == 0)
        {
            //std::cout << "No feasible solution die!" << std::endl;
            return {};
        }

        unsigned int selected_node_id = select_arc_acs(candidate_nodes_ids, ant_colony, alpha, beta, q_0, rho);

        insert_selected_arc(selected_node_id);
    }

    return solution;
}

void Ant::initialize_tour()
{
    // Pick a random depot as starting node
    // Generate integer in the range [1, num_vehicles]
    // And set it as current vehicle
    current_vehicle_number = rand() % problem->get_num_vehicles() + 1;

    // Add num_customers to get the node_id
    current_node_id = current_vehicle_number + problem->get_num_customers();

    // The node has been visited, can't be visited  later
    visited_nodes.push_back(current_node_id);

    // Add it to the solution
    solution.push_back(TourAtom(current_node_id, current_load, current_time, current_distance));

    // Add the committed customers of the selected vehicle to the solution
    insert_committed_customers(current_vehicle_number);
}

void Ant::insert_committed_customers(unsigned int vehicle_number)
{
    // Iterate over committed customers for the vehicle_number to add them to the tour
    for (auto &committed_c_node_id : problem->get_vehicle_commitments(vehicle_number))
    {
        current_load += problem->get_customer_demand(committed_c_node_id);
        current_time += problem->get_distance(current_node_id, committed_c_node_id);
        current_time += problem->get_customer_service_time(committed_c_node_id);
        current_distance += problem->get_distance(current_node_id, committed_c_node_id);

        current_node_id = committed_c_node_id;

        solution.push_back(TourAtom(current_node_id, current_load, current_time, current_distance));

        visited_nodes.push_back(committed_c_node_id);
        num_visited_customers++;
    }
}

std::vector<unsigned int> Ant::compute_candidate_arcs() const
{
    // Given the current state of the ant (current_load, current_vehicle, visited_nodes, ...)
    // compute the candidate nodes_ids for the next move

    // We do this by iteratively removing nodes_ids from all the available nodes of the problem
    std::vector<unsigned int> candidate_nodes_ids = problem->get_available_nodes_ids();

    // We remove the nodes that have already been visited
    candidate_nodes_ids.erase(std::remove_if(candidate_nodes_ids.begin(),
                                             candidate_nodes_ids.end(),
                                             [this](unsigned int node_id) { return has_node_been_visited(node_id); }),
                              candidate_nodes_ids.end());

    // If we are at a depot we can't go to a depot
    // So we filter the depot nodes from the candidates
    if (problem->is_node_depot(current_node_id))
    {
        candidate_nodes_ids.erase(std::remove_if(candidate_nodes_ids.begin(),
                                                 candidate_nodes_ids.end(),
                                                 [this](unsigned int node_id) { return problem->is_node_depot(node_id); }),
                                  candidate_nodes_ids.end());
    }

    // We remove the nodes that have been committed to other vehicles
    // Note : The nodes committed to the current vehicle have already been added to the tour when this function is called
    //        so they can't appear here
    candidate_nodes_ids.erase(std::remove_if(candidate_nodes_ids.begin(),
                                             candidate_nodes_ids.end(),
                                             [this](unsigned int node_id) { return problem->has_c_node_been_committed(node_id); }),
                              candidate_nodes_ids.end());

    // We remove nodes whose demand it too high given the current_load
    candidate_nodes_ids.erase(std::remove_if(candidate_nodes_ids.begin(),
                                             candidate_nodes_ids.end(),
                                             [this](unsigned int node_id) { return problem->get_customer_demand(node_id) + current_load > problem->get_vehicle_capacity(); }),
                              candidate_nodes_ids.end());

    return candidate_nodes_ids;
}

unsigned int Ant::select_arc_nn(std::vector<unsigned int> candidate_nodes_ids) const
{
    // Simply pick the arc with the least distance
    unsigned int node_id = 0;
    float distance = std::numeric_limits<float>::infinity();

    for (auto &candidate_node_id : candidate_nodes_ids)
    {
        if (problem->get_distance(current_node_id, candidate_node_id) < distance)
        {
            node_id = candidate_node_id;
            distance = problem->get_distance(current_node_id, candidate_node_id);
        }
    }

    return node_id;
}

unsigned int Ant::select_arc_acs(std::vector<unsigned int> candidate_nodes_ids, AntColony *ant_colony, float alpha, float beta, float q_0, float rho) const
{
    // We compute the weights
    std::vector<float> weights;
    for (auto &candidate_node_id : candidate_nodes_ids)
    {
        float eta_ij = (float)1 / problem->get_distance(current_node_id, candidate_node_id);
        float tau_ij = ant_colony->get_pheromons(current_node_id, candidate_node_id);
        float weight = pow(tau_ij, alpha) * pow(eta_ij, beta);

        weights.push_back(weight);
    }
    // TODO: Move this somewhere else
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform(0, 1);
    float sample = uniform(gen);

    bool exploitation = sample <= q_0 ? true : false;
    unsigned int selected_node_id;

    if (exploitation)
    {
        auto index_of_max = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
        selected_node_id = candidate_nodes_ids[index_of_max];
    }
    else
    {
        float normalization_constant = 0;
        for (auto &weight : weights)
        {
            normalization_constant += weight;
        }

        std::vector<float> distribution;
        for (auto &weight : weights)
        {
            float normalized_weight = weight / normalization_constant;
            distribution.push_back(normalized_weight);
        }

        auto sampled_index = sample_from_categorical(distribution);
        selected_node_id = candidate_nodes_ids[sampled_index];
    }

    return selected_node_id;
}

void Ant::insert_selected_arc(unsigned int selected_node_id)
{
    if (problem->is_node_depot(selected_node_id))
    {
        current_node_id = selected_node_id;
        current_vehicle_number = selected_node_id - problem->get_num_customers();
        current_load = 0;
        current_time = 0;
        current_distance = 0;

        solution.push_back(TourAtom(selected_node_id, 0, 0, 0));

        visited_nodes.push_back(selected_node_id);

        insert_committed_customers(current_vehicle_number);
    }
    else
    {
        current_load += problem->get_customer_demand(selected_node_id);
        current_time += problem->get_distance(current_node_id, selected_node_id);
        current_time += problem->get_customer_service_time(selected_node_id);
        current_distance += problem->get_distance(current_node_id, selected_node_id);

        current_node_id = selected_node_id;

        solution.push_back(TourAtom(selected_node_id, current_load, current_time, current_distance));

        visited_nodes.push_back(selected_node_id);
        num_visited_customers++;
    }
}

bool Ant::has_node_been_visited(unsigned int node_id) const
{
    std::vector<unsigned int>::const_iterator it;
    it = std::find(visited_nodes.begin(), visited_nodes.end(), node_id);

    return (it != visited_nodes.end());
}

unsigned int Ant::sample_from_categorical(const std::vector<float> &distribution) const
{
    // Given a vector of float whose sum is 1
    // Return a random index sampled according to the distribution
    // specified by the vector

    unsigned int num_bins = distribution.size();
    std::vector<std::pair<float, float>> bins;

    float acc = 0;

    for (auto i = 0; i < num_bins; i++)
    {
        bins.push_back(std::make_pair(acc, acc + distribution[i]));
        acc += distribution[i];
    }

    // TODO: Move this somewhere else
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform(0, 1);
    float sample = uniform(gen);

    unsigned int index;

    for (auto i = 0; i < bins.size(); i++)
    {
        if (bins[i].first <= sample && sample < bins[i].second)
        {
            index = i;
            break;
        }
    }

    return index;
}