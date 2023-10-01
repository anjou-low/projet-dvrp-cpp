#include "tour_atom.h"

TourAtom::TourAtom(unsigned int node_id, float load, float end_of_service, float distance) : node_id{node_id}, load{load}, end_of_service{end_of_service}, distance{distance} {};