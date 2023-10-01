#pragma once

#include <string>
#include <vector>
#include <map>

struct Node
{
    unsigned int id;
    float x;
    float y;
    bool is_depot;
    float available_time;
    int demand;
    float service_time;

    Node(unsigned int id, float x, float y, bool is_depot, float available_time, int demand, float service_time);
};

class Problem
{
private:
    std::vector<Node *> nodes;
    std::vector<unsigned int> available_nodes_ids;
    std::vector<unsigned int> available_c_nodes_ids;
    std::map<unsigned int, std::vector<unsigned int>> vehicles_commitments;
    std::vector<unsigned int> committed_c_nodes_ids;

    // std::vector<std::vector<float>> distance_matrix;
    std::vector<float> distances;

    unsigned int num_customers;
    unsigned int num_vehicles;
    unsigned int vehicle_capacity;
    float last_update_time;

    std::string filepath;
    unsigned int t_wd;
    unsigned int n_ts;

    std::string dataset_name;
    float scaling_factor;

public:
    Problem(std::string filepath, unsigned int t_wd, unsigned int n_ts);

    std::vector<unsigned int> update(float time);
    void commit(unsigned int c_node_id, unsigned int vehicle_number);

    std::vector<unsigned int> get_available_nodes_ids() const;
    std::vector<unsigned int> get_available_c_nodes_ids() const;
    std::vector<unsigned int> get_vehicle_commitments(unsigned int vehicle_number) const;
    std::vector<unsigned int> get_committed_c_nodes_ids() const;
    float get_distance(unsigned int node_id_i, unsigned int node_id_j) const;

    unsigned int get_num_nodes() const;
    unsigned int get_num_available_nodes() const;
    unsigned int get_num_vehicles() const;
    unsigned int get_num_customers() const;
    unsigned int get_vehicle_capacity() const;
    unsigned int get_num_available_customers() const;

    int get_customer_demand(unsigned int c_node_id);
    float get_customer_service_time(unsigned int c_node_id);

    bool is_node_depot(unsigned int node_id) const;
    bool has_c_node_been_committed(unsigned int c_node_id) const;

    float get_scaling_factor() const;

    void visual_dump_data() const;
    void dump_to_file(const std::string &filename) const;
};