#include "problem.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <iostream>
#include <math.h>

Node::Node(unsigned int id, float x, float y, bool is_depot, float available_time, int demand, float service_time) : id{id}, x{x}, y{y}, is_depot{is_depot}, available_time{available_time}, demand{demand}, service_time{service_time} {};

Problem::Problem(std::string filepath, unsigned int t_wd, unsigned int n_ts) : t_wd{t_wd}, n_ts{n_ts}
{
    std::ifstream infile(filepath);
    std::string line;

    std::getline(infile, dataset_name);

    std::getline(infile, line);
    std::getline(infile, line);
    std::getline(infile, line);

    std::getline(infile, line);
    std::istringstream iss(line);
    iss >> num_vehicles >> vehicle_capacity;

    std::getline(infile, line);
    std::getline(infile, line);
    std::getline(infile, line);
    std::getline(infile, line);

    unsigned int depot_due_date;
    unsigned int counter = 0;

    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        unsigned int number, demand, ready_time, due_date;
        float x_coord, y_coord, service_time, available_time;
        if (!(iss >> number >> x_coord >> y_coord >> demand >> ready_time >> due_date >> service_time >> available_time))
        {
            break;
        }
        Node *node = new Node(number, x_coord, y_coord, false, available_time, demand, service_time);

        nodes.push_back(node);

        if (counter == 0)
        {
            // We extract the due date of the depot for the scaling factor
            depot_due_date = due_date;
        }

        counter++;
    }

    // Get the number of customers
    num_customers = nodes.size() - 1;

    // We scale the (x, y, service_time, available_time) so they fit in our day length
    scaling_factor = (float)t_wd / (float)depot_due_date;
    for (auto &node : nodes)
    {
        node->x *= scaling_factor;
        node->y *= scaling_factor;
        node->available_time *= scaling_factor;
        node->service_time *= scaling_factor;
    }

    // We add depot duplicates (one for each vehicle)
    float depot_x_coord = nodes[0]->x;
    float depot_y_coord = nodes[0]->y;

    for (auto i = 1; i <= num_vehicles; i++)
    {
        Node *node = new Node(num_customers + i, depot_x_coord, depot_y_coord, true, 0, 0, 0);
        nodes.push_back(node);
    }

    // We build the entire distance matrix
    for (auto i = 0; i < nodes.size(); i++)
    {
        for (auto j = 0; j < nodes.size(); j++)
        {
            float distance = sqrt(pow((nodes[i]->x - nodes[j]->x), 2) + pow(nodes[i]->y - nodes[j]->y, 2));
            distances.push_back(distance);
        }
    }

    // We initialize the vehicles_commitments
    for (auto i = 1; i <= num_vehicles; i++)
    {
        vehicles_commitments[i] = {};
    }

    last_update_time = -1;
}

std::vector<unsigned int> Problem::update(float time)
{
    available_nodes_ids.clear();
    available_c_nodes_ids.clear();

    std::vector<unsigned int> diff;

    for (auto i = 1; i < nodes.size(); i++)
    {
        if (nodes[i]->available_time <= time)
        {
            available_nodes_ids.push_back(nodes[i]->id);
        }
    }

    for (auto i = 1; i <= num_customers; i++)
    {
        if (nodes[i]->available_time <= time)
        {
            available_c_nodes_ids.push_back(nodes[i]->id);
        }

        if (nodes[i]->available_time > last_update_time && nodes[i]->available_time <= time)
        {
            diff.push_back(nodes[i]->id);
        }
    }

    last_update_time = time;

    return diff;
}

void Problem::commit(unsigned int c_node_id, unsigned int vehicle_number)
{
    // TODO : Add invariants
    // Node has not already been committed

    vehicles_commitments.at(vehicle_number).push_back(c_node_id);
    committed_c_nodes_ids.push_back(c_node_id);
}

unsigned int Problem::get_num_nodes() const
{
    // We remove 1 because the first node is a padding dummy
    return nodes.size() - 1;
}

unsigned int Problem::get_num_available_nodes() const
{
    return available_nodes_ids.size();
}

std::vector<unsigned int> Problem::get_available_nodes_ids() const
{
    return available_nodes_ids;
}

std::vector<unsigned int> Problem::get_available_c_nodes_ids() const
{
    return available_c_nodes_ids;
}

std::vector<unsigned int> Problem::get_committed_c_nodes_ids() const
{
    return committed_c_nodes_ids;
}

float Problem::get_distance(unsigned int node_id_i, unsigned int node_id_j) const
{
    //std::cout << "Problem::get_distance" << std::endl;
    return distances.at(node_id_i * nodes.size() + node_id_j);
}

std::vector<unsigned int> Problem::get_vehicle_commitments(unsigned int vehicle_number) const
{
    //std::cout << "Problem::get_vehicle_commitments" << std::endl;
    return vehicles_commitments.at(vehicle_number);
}

unsigned int Problem::get_num_vehicles() const
{
    return num_vehicles;
}

unsigned int Problem::get_num_customers() const
{
    return num_customers;
}

unsigned int Problem::get_vehicle_capacity() const
{
    return vehicle_capacity;
}

unsigned int Problem::get_num_available_customers() const
{
    return available_nodes_ids.size() - num_vehicles;
}

int Problem::get_customer_demand(unsigned int c_node_id)
{
    return nodes[c_node_id]->demand;
}

float Problem::get_customer_service_time(unsigned int c_node_id)
{
    return nodes[c_node_id]->service_time;
}

bool Problem::is_node_depot(unsigned int node_id) const
{
    return nodes[node_id]->is_depot;
}

bool Problem::has_c_node_been_committed(unsigned int c_node_id) const
{
    std::vector<unsigned int>::const_iterator it;
    it = std::find(committed_c_nodes_ids.begin(), committed_c_nodes_ids.end(), c_node_id);

    return (it != committed_c_nodes_ids.end());
}

float Problem::get_scaling_factor() const
{
    return scaling_factor;
}

void Problem::visual_dump_data() const
{
    for (auto i = 0; i <= num_customers; i++)
    {
        std::cout << nodes[i]->id << "," << nodes[i]->x << "," << nodes[i]->y << "," << nodes[i]->demand << std::endl;
    }
}

void Problem::dump_to_file(const std::string &filename) const
{
    std::ofstream data_file;
    data_file.open(filename);

    for (auto &node : nodes)
    {
        data_file << node->id << ", " << node->x << ", " << node->y << ", " << node->demand << ", " << node->service_time << ", " << node->is_depot << ", " << node->available_time << std::endl;
    }

    data_file.close();
}