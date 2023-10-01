#include "Local_search.h"
#include <vector>
#include <iostream>
#include <cmath>


double ls_elapsed_since(const std::chrono::high_resolution_clock::time_point& time)
{
	auto now = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = now - time;

	return elapsed.count();
}

Local_search::Local_search(const Problem& p_problem, std::vector<TourAtom>& p_solution) : problem{ p_problem }, solution{ p_solution }
{
	std::vector<TourAtom> current_vector_vehicle;

	for (auto it = p_solution.begin(); it != p_solution.end(); it++) {
		if (p_problem.is_node_depot(it->node_id)) {
			current_vector_vehicle.push_back(*it);
			if ( current_vector_vehicle.size() > 1) {
				solution_vehicle.push_back(current_vector_vehicle);
				current_vector_vehicle.clear(); 
				current_vector_vehicle.push_back(*it);
			}
		}
		else {
			current_vector_vehicle.push_back(*it);
		}
	}
	// We add the last vehicle clients since it didn't end with a depot
	solution_vehicle.push_back(current_vector_vehicle);
	current_vector_vehicle.clear();

}
int Local_search::calculate_remaining_demand(unsigned int to_touratom_position, unsigned int vehicle_position)
{
	int demand = 0;
	for (int i = to_touratom_position; i < solution_vehicle[vehicle_position].size(); i++) {
		demand += problem.get_customer_demand(solution_vehicle[vehicle_position][i].node_id);
	}
	return demand;
}

int Local_search::calculate_capacity_left_before(unsigned int to_touratom_position, unsigned int vehicle_position) 
{
	int capacity_left = problem.get_vehicle_capacity();
	for (int i = 0; i < to_touratom_position-1; i++) {
		capacity_left -= problem.get_customer_demand(solution_vehicle[vehicle_position][i].node_id);
	}
	return capacity_left;
}

float Local_search::inter_vehicle_path_length_difference(unsigned int to_touratom_position_in_vehicle1, unsigned int vehicle1_position, unsigned int to_touratom_position_in_vehicle2, unsigned int vehicle2_position)
{
	float path1 = problem.get_distance(solution_vehicle[vehicle1_position][to_touratom_position_in_vehicle1 - 1].node_id, solution_vehicle[vehicle1_position][to_touratom_position_in_vehicle1].node_id);
	float path2 = problem.get_distance(solution_vehicle[vehicle2_position][to_touratom_position_in_vehicle2 - 1].node_id, solution_vehicle[vehicle2_position][to_touratom_position_in_vehicle2].node_id);
	
	float path1_cross = problem.get_distance(solution_vehicle[vehicle1_position][to_touratom_position_in_vehicle1 - 1].node_id, solution_vehicle[vehicle2_position][to_touratom_position_in_vehicle2].node_id);
	float path2_cross = problem.get_distance(solution_vehicle[vehicle2_position][to_touratom_position_in_vehicle2 - 1].node_id, solution_vehicle[vehicle1_position][to_touratom_position_in_vehicle1].node_id);
	
	return path1_cross + path2_cross - path1 - path2;
}

float Local_search::intra_path_length_difference(unsigned int touratom_position1, unsigned int touratom_position2, unsigned int vehicle_position)
{
	unsigned int tour_atom_pos1_id = solution_vehicle[vehicle_position][touratom_position1].node_id;
	unsigned int tour_atom_pos1_prev_id = solution_vehicle[vehicle_position][touratom_position1-1].node_id;
	unsigned int tour_atom_pos1_next_id = solution_vehicle[vehicle_position][touratom_position1 + 1].node_id;
	unsigned int tour_atom_pos2_id = solution_vehicle[vehicle_position][touratom_position2].node_id;
	unsigned int tour_atom_pos2_prev_id = solution_vehicle[vehicle_position][touratom_position2 - 1].node_id;
	unsigned int tour_atom_pos2_next_id = solution_vehicle[vehicle_position][touratom_position2 + 1].node_id;
	float path_diff;
	int path_position_diff = touratom_position2 - touratom_position1;
	if (abs(path_position_diff) < 2) {
		path_diff = -problem.get_distance(tour_atom_pos1_prev_id, tour_atom_pos1_id) - problem.get_distance(tour_atom_pos1_id, tour_atom_pos2_id) -
			problem.get_distance(tour_atom_pos2_id, tour_atom_pos2_next_id) + problem.get_distance(tour_atom_pos2_prev_id, tour_atom_pos2_id) +
			problem.get_distance(tour_atom_pos2_id, tour_atom_pos1_id) + problem.get_distance(tour_atom_pos1_id, tour_atom_pos2_next_id);
	}
	else {
		path_diff = -problem.get_distance(tour_atom_pos1_prev_id, tour_atom_pos1_id) - problem.get_distance(tour_atom_pos1_id, tour_atom_pos1_next_id) -
			problem.get_distance(tour_atom_pos2_prev_id, tour_atom_pos2_id) - problem.get_distance(tour_atom_pos2_id, tour_atom_pos2_next_id) +
			problem.get_distance(tour_atom_pos1_prev_id, tour_atom_pos2_id) + problem.get_distance(tour_atom_pos2_id, tour_atom_pos1_next_id) +
			problem.get_distance(tour_atom_pos2_prev_id, tour_atom_pos1_id) + problem.get_distance(tour_atom_pos1_id, tour_atom_pos2_next_id);
	}
	return path_diff;
}


void Local_search::inter_vehicle_swap( unsigned int to_touratom_position_in_vehicle1, unsigned int vehicle1_position, unsigned int to_touratom_position_in_vehicle2, unsigned int vehicle2_position)
{
	// copy end path 1 
	std::vector<TourAtom> copy_end_vehicle1;

	for (int i = to_touratom_position_in_vehicle1; i < solution_vehicle[vehicle1_position].size(); i++) {
		//copy_end_vehicle1.push_back(solution_vehicle[vehicle_path1_idx][i]);
		copy_end_vehicle1.push_back(
			TourAtom(solution_vehicle[vehicle1_position][i].node_id,
				solution_vehicle[vehicle1_position][i].load,
				solution_vehicle[vehicle1_position][i].end_of_service,
				solution_vehicle[vehicle1_position][i].distance));
	}

	int size = solution_vehicle[vehicle1_position].size();
	// Delete nodes in path 1 that begin at swap node
	while (solution_vehicle[vehicle1_position].size() - 1 >= to_touratom_position_in_vehicle1) {
		solution_vehicle[vehicle1_position].pop_back();
	}
	// Add ends nodes from path 2 to path 1
	for (int i = to_touratom_position_in_vehicle2; i < solution_vehicle[vehicle2_position].size(); i++) {
		solution_vehicle[vehicle1_position].push_back(solution_vehicle[vehicle2_position][i]);
	}
	update_tour_atom_values(to_touratom_position_in_vehicle1, vehicle1_position);
	// Delete nodes in path 2 that begin at swap node
	while (solution_vehicle[vehicle2_position].size() -1 >= to_touratom_position_in_vehicle2 ) {
		solution_vehicle[vehicle2_position].pop_back();
	}
	// adds end of path1 to path 2
	for (auto it = copy_end_vehicle1.begin(); it != copy_end_vehicle1.end(); it++) {
		solution_vehicle[vehicle2_position].push_back(*it);
	}
	update_tour_atom_values(to_touratom_position_in_vehicle2, vehicle2_position);
}

void Local_search::intra_vehicle_swap(unsigned int touratom_position1, unsigned int touratom_position2, unsigned int vehicle_position)
{
	unsigned int tour_atom_pos1_id = solution_vehicle[vehicle_position][touratom_position1].node_id;
	solution_vehicle[vehicle_position][touratom_position1].node_id = solution_vehicle[vehicle_position][touratom_position2].node_id;
	solution_vehicle[vehicle_position][touratom_position2].node_id = tour_atom_pos1_id;
	update_tour_atom_values(touratom_position1, vehicle_position);
}

int Local_search::search(int t_ls) {
	auto time_0 = std::chrono::high_resolution_clock::now();
	for (int vehicle_i_idx = 1; vehicle_i_idx < solution_vehicle.size() ; vehicle_i_idx++) {
		for (int vehicle_j_idx = 1; vehicle_j_idx < solution_vehicle.size() ; vehicle_j_idx++) {
			if (vehicle_i_idx != vehicle_j_idx) {
				for (int node_k_idx = 1; node_k_idx < solution_vehicle[vehicle_i_idx].size() - 1; node_k_idx++) {
					for (int node_l_idx = 1; node_l_idx < solution_vehicle[vehicle_j_idx].size() - 1; node_l_idx++) {
						if (!problem.has_c_node_been_committed(solution_vehicle[vehicle_i_idx][node_k_idx].node_id) && !problem.has_c_node_been_committed(solution_vehicle[vehicle_j_idx][node_l_idx].node_id)) {
							if (calculate_capacity_left_before(node_k_idx, vehicle_i_idx) >= calculate_remaining_demand(node_l_idx, vehicle_j_idx) && calculate_capacity_left_before(node_l_idx, vehicle_j_idx) >= calculate_remaining_demand(node_k_idx, vehicle_i_idx)) {
								if (inter_vehicle_path_length_difference(node_k_idx, vehicle_i_idx, node_l_idx, vehicle_j_idx) < 0) {
									inter_vehicle_swap(node_k_idx, vehicle_i_idx, node_l_idx, vehicle_j_idx);
									if (ls_elapsed_since(time_0) > t_ls) {
										return 0;
									};
								}
							}
						}
					}
				}
			}
			else {
				for (int node_k_idx = 1; node_k_idx < solution_vehicle[vehicle_i_idx].size() - 1; node_k_idx++) {
					for (int node_l_idx = node_k_idx + 1; node_l_idx < solution_vehicle[vehicle_i_idx].size() - 1; node_l_idx++) {
						if (!problem.has_c_node_been_committed(solution_vehicle[vehicle_i_idx][node_k_idx].node_id) && !problem.has_c_node_been_committed(solution_vehicle[vehicle_i_idx][node_l_idx].node_id)) {
							if (intra_path_length_difference(node_k_idx, node_l_idx, vehicle_i_idx) < 0) {
								intra_vehicle_swap(node_k_idx, node_l_idx, vehicle_i_idx);
								if (ls_elapsed_since(time_0) > t_ls) {
									return 0;
								};
							}
						}
					}
				}
			}
		}
	}
}

std::vector<TourAtom> Local_search::solution_from_search()
{
	unsigned int solution_length = solution_vehicle.size();
	unsigned int last_vehicle_length = solution_vehicle[solution_length - 1].size();

	std::vector<TourAtom> final_solution;
	for (int i = 0; i < solution_vehicle.size(); i++) {
		for (int j = 0; j < solution_vehicle[i].size() - 1; j++) {
			final_solution.push_back(solution_vehicle[i][j]);
		}
	}
	final_solution.push_back(solution_vehicle[solution_length - 1][last_vehicle_length - 1]);
	return final_solution;
}

void Local_search::update_tour_atom_values(unsigned int from_node_position, unsigned int vehicle_position)
{
	int current_load = problem.get_customer_demand(solution_vehicle[vehicle_position][from_node_position - 1].node_id);
	float current_service_time = problem.get_customer_service_time(solution_vehicle[vehicle_position][from_node_position - 1].node_id);
	float current_distance = solution_vehicle[vehicle_position][from_node_position - 1].distance;

	for (int i = from_node_position; i < solution_vehicle[vehicle_position].size()-1; i++) {
		current_load += problem.get_customer_demand(solution_vehicle[vehicle_position][i].node_id);
		current_service_time += problem.get_distance(solution_vehicle[vehicle_position][i].node_id, solution_vehicle[vehicle_position][i-1].node_id);
		current_service_time += problem.get_customer_service_time(solution_vehicle[vehicle_position][i].node_id);
		current_distance += problem.get_distance(solution_vehicle[vehicle_position][i].node_id, solution_vehicle[vehicle_position][i - 1].node_id);
		solution_vehicle[vehicle_position][i].load = current_load;
		solution_vehicle[vehicle_position][i].end_of_service = current_service_time;
		solution_vehicle[vehicle_position][i].distance = current_distance;
	}
}

float Local_search::compute_solution_score(const std::vector<TourAtom>& solution) const
{
	float score = 0;
	float last_distance = 0;

	for (const auto& tour_atom : solution)
	{
		if (problem.is_node_depot(tour_atom.node_id))
		{
			score += last_distance;
		}
		last_distance = tour_atom.distance;
	}

	score += last_distance;

	return score;
}