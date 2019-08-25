/* --------------------------------------------------------------------------- *
 * @file:   path_finder.cpp
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the implementations for the class PathFinder. This 
 *          class used for path finding using a search algorithm.  
 *
 * -------------------------------------------------------------------------- */
#include "planner.h"
#include "path_finder.h"

namespace planner 
{

	PathFinder::PathFinder() 
	{
		// Set source and destination to default values. 
		source_ = destination_ = { INT_MAX, INT_MAX };

		// Default search algorithm is Astar. 
		algorithm_ = planner::search_algorithm::TYPE::ASTAR;

		// Set the algorithm. 
		set_search_algorithm( algorithm_ );
	}

	PathFinder::~PathFinder() {
		/* Not implemented */
	}

	bool PathFinder::set_search_algorithm( const search_algorithm::TYPE &s ) 
	{
		switch( s ) 
		{
			// Bind astar algorithm to the search function. 
			case planner::search_algorithm::TYPE::ASTAR:
				search_algorithm_ = std::bind( search_algorithm::Function::astar, 
				                               std::placeholders::_1,
				                               std::placeholders::_2,
				                               std::placeholders::_3,
				                               std::placeholders::_4,
				                               std::placeholders::_5,
				                               std::placeholders::_6 );
				break;

			// Algorithm is not supported. 
			case planner::search_algorithm::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Search algorithm is not supported" << std::endl;
				return false;
		}
		return true;
	}

	bool PathFinder::set_source( const planner::Coord &source ) 
	{
		// Assert that source values are valid. 
		if ( source.r < 0        || source.c < 0       ||
			   source.r == INT_MAX || source.c == INT_MAX )
		{
			std::cout << "[ERROR] Invalid source coordinate with value (" 
								<< source.r << "," << source.c << ")" << std::endl;

			return false;
		}

		source_ = source;
		return true;
	}

	planner::Coord PathFinder::get_source() 
	{
		return source_;
	}

	bool PathFinder::set_destination( const planner::Coord &destination ) 
	{
		// Assert that destination values are valid. 
		if( destination.r < 0        || destination.c < 0       ||
			  destination.r == INT_MAX || destination.c == INT_MAX )
		{
			std::cout << "[ERROR] Invalid destination coordinate with value (" 
								<< destination.r << "," << destination.c << ")" << std::endl;

			return false;
		}

		destination_ = destination;
		return true;
	}

	planner::Coord PathFinder::get_destination( ) 
	{
		return destination_;
	}

	bool PathFinder::find_path( std::vector<std::vector<int> >& bin_map,
											        const planner::MapParameters& map_params, 
											        const planner::heuristic::TYPE& heuristic )
	{

		// Assert that map is not empty
		if ( bin_map.empty() )
		{
			std::cout << "[ERROR] Binary map is empty" << std::endl;
			return false;
		}

		// Assert heuristic
		if ( planner::heuristic::TYPE::NOT_SUPPORTED == heuristic )
		{
			std::cout << "[ERROR] Heuristic is not supported" << std::endl;
			return false;
		}

		path_.clear();

		search_algorithm_( bin_map, map_params, source_, destination_, 
											 heuristic, path_ );

		// Check if path was not found. 
		if ( path_.empty() )
		{
			std::cout << "[ERROR] Could not find path" << std::endl;
			return false; 
		}
		return true;
	}

	std::vector<planner::Coord> PathFinder::get_path()
	{
		return path_;
	}

	void PathFinder::print() 
	{
		std::cout << "[INFO] Planner configuration" << std::endl
		          << "\tSource: "           << source_
		          << "\tDestination: "      << destination_
		          << "\tSearch Algorithm: " << search_algorithm::NAME.find( 
		          	                            algorithm_ )->second << std::endl
		          << std::endl;
	}
} // End of namespace planner 
