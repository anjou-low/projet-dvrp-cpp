#include <vector>
#include "problem.h"
#include "tour_atom.h"
#include <iostream>
#include <time.h>
#include <string>
#include <chrono>


class Local_search	
{
private:
	Problem problem;
	std::vector<TourAtom> solution;
	std::vector<std::vector<TourAtom>> solution_vehicle;

	int calculate_remaining_demand(unsigned int to_touratom_position, unsigned int vehicle_position);
	int calculate_capacity_left_before(unsigned int to_touratom_position, unsigned int vehicle_position);
	float inter_vehicle_path_length_difference( unsigned int to_touratom_position_in_vehicle1, unsigned int vehicle1_position, unsigned int to_touratom_position_in_vehicle2, unsigned int vehicle2_position);
	float intra_path_length_difference(unsigned int touratom_position1, unsigned int touratom_position2, unsigned int vehicle_position);
	void update_tour_atom_values(unsigned int to_touratom_position, unsigned int vehicle_position);
	void inter_vehicle_swap(unsigned int to_touratom_position_in_vehicle1, unsigned int vehicle1_position, unsigned int to_touratom_position_in_vehicle2, unsigned int vehicle2_position);
	void intra_vehicle_swap(unsigned int touratom_position1, unsigned int touratom_position2, unsigned int vehicle_position);
public:
	Local_search(const Problem& p_problem, std::vector<TourAtom>& p_solution);
	int search(int t_ls);
	float compute_solution_score(const std::vector<TourAtom>& solution) const;
	std::vector<TourAtom> solution_from_search();
};