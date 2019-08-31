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

unsigned int Function::manhattan( const planner::Coord& coordinate1, 
	                                const planner::Coord& coordinate2 ) 
{
	 auto delta = std::move( get_delta( coordinate1, coordinate2 ) );
	 return ( unsigned int ) ( delta.r + delta.c );
}

unsigned int Function::euclidean( const planner::Coord& coordinate1, 
	                                const planner::Coord& coordinate2 ) 
{
	 auto delta = std::move( get_delta( coordinate1, coordinate2 ) );
	 return ( unsigned int ) ( sqrt( pow( delta.r, 2) + pow( delta.c, 2 ) ) );
}

unsigned int Function::octagonal( const planner::Coord& coordinate1, 
	                                const planner::Coord& coordinate2 ) 
{
	auto delta = std::move( get_delta( coordinate1, coordinate2 ) );
	return ( unsigned int) ( ( delta.r + delta.c ) - std::min( delta.r, delta.c ) );
}

planner::Coord Function::get_delta( const planner::Coord& coordinate1, 
	                                  const planner::Coord& coordinate2 ) 
{
	return { abs( coordinate1.r - coordinate2.r ), 
		       abs( coordinate1.c - coordinate2.c ) };
}

} // End of namespace heuristic.
} // End of namespace planner.
