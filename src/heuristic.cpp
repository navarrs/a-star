/* --------------------------------------------------------------------------- *
 * @file:   heuristic.cpp
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the implementations for the struct of type
 *          planner::heuristic. 
 *
 * -------------------------------------------------------------------------- */
#include "planner.h"

namespace planner {
namespace heuristic {

unsigned int Function::manhattan(const planner::Coord& coord1, 
	                               const planner::Coord& coord2) {

	 const auto delta = std::move(get_delta(coord1, coord2));
	 return (unsigned int) (delta.r + delta.c);
}

unsigned int Function::euclidean(const planner::Coord& coord1, 
	                               const planner::Coord& coord2) {

	 const auto delta = std::move(get_delta(coord1, coord2));
	 return (unsigned int) (sqrt( pow(delta.r, 2) + pow( delta.c, 2)));
}

unsigned int Function::octagonal(const planner::Coord& coord1, 
	                               const planner::Coord& coord2) {

	const auto delta = std::move(get_delta(coord1, coord2));
	return (unsigned int) ((delta.r + delta.c) - std::min(delta.r, delta.c));
}

const planner::Coord Function::get_delta(const planner::Coord& coord1, 
	                                 const planner::Coord& coord2) {

	return {abs(coord1.r - coord2.r), abs(coord1.c - coord2.c)};
}

} // End of namespace heuristic.
} // End of namespace planner.
