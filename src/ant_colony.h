#pragma once

#include <vector>
#include "problem.h"
#include "ant.h"
#include "tour_atom.h"

class AntColony
{
private:
    std::vector<TourAtom> best_solution;
    float best_solution_score;
    std::vector<float> pheromon_matrix;

    Problem *problem;
    unsigned int num_ants;
    float alpha;
    float beta;
    float q_0;
    float rho;
    float tau_0;

    float compute_solution_score(const std::vector<TourAtom> &solution) const;

public:
    AntColony(Problem *problem, unsigned int num_ants, float alpha, float beta, float q_0, float rho);

    void step();
    void update_solution();
    float get_pheromons(unsigned int node_id_i, unsigned int node_id_j);

    const std::vector<TourAtom> &get_best_solution() const;
    float get_best_solution_score() const;

    void visual_dump_data() const;
};