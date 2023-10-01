
#pragma once

struct TourAtom
{
    unsigned int node_id;
    float load;
    float end_of_service;
    float distance;

    TourAtom(unsigned int node_id, float load, float end_of_service, float distance);
};