#pragma once

#include <vector>
#include "problem.h"
#include "tour_atom.h"

class AntColony;

class Ant
{
private:
    Problem *problem;
    const std::vector<std::vector<float>> *pheromon_matrix;
    std::vector<TourAtom> solution;

    std::vector<unsigned int> visited_nodes;
    unsigned int num_visited_customers;
    unsigned int current_node_id;
    unsigned int current_vehicle_number;
    float current_time;
    int current_load;
    float current_distance;

    void initialize_tour();
    std::vector<unsigned int> compute_candidate_arcs() const;
    unsigned int select_arc_nn(std::vector<unsigned int> candidate_nodes_ids) const;
    unsigned int select_arc_acs(std::vector<unsigned int> candidate_nodes_ids, AntColony *ant_colony, float alpha, float beta, float q_0, float rho) const;
    void insert_selected_arc(unsigned int selected_node_id);

    void insert_committed_customers(unsigned int vehicle_number);
    bool has_node_been_visited(unsigned int node_id) const;

    unsigned int sample_from_categorical(const std::vector<float> &distribution) const;

public:
    Ant(Problem *problem);

    std::vector<TourAtom> construct_solution_nn();
    std::vector<TourAtom> construct_solution_acs(AntColony *ant_colony, float alpha, float beta, float q_0, float rho);
};