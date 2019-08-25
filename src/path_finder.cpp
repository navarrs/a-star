/**
 * @file:   path_generator.cpp
 * @author: Ingrid Navarro
 * @brief:  This file contains the implementations for the class PathFinder.  
 *          This class used for path finding using the Astar algorithm. 
 */

#include "planner.h"
#include "path_finder.h"


namespace planner {

	PathFinder::PathFinder() {
		source_ = destination_ = { INT_MAX, INT_MAX };
		algorithm_ = search_algorithm::TYPE::ASTAR;
		set_search_algorithm( algorithm_ );
	}

	PathFinder::~PathFinder() {
		/* Not implemented */
	}

	bool PathFinder::set_search_algorithm( const search_algorithm::TYPE &s ) {
		switch( s ) {
			case search_algorithm::TYPE::ASTAR:
				// Bind astar algorithm to the search function. 
				search_algorithm_ = std::bind( search_algorithm::Function::astar, 
				                               std::placeholders::_1,
				                               std::placeholders::_2,
				                               std::placeholders::_3,
				                               std::placeholders::_4,
				                               std::placeholders::_5,
				                               std::placeholders::_6 );
				break;
			case search_algorithm::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Search algorithm is not supported " 
				          << std::endl;
				return EXIT_FAILURE;

		}
		return EXIT_SUCCESS;
	}

	void PathFinder::set_source( const planner::Coord &source ) {
		source_ = source;
	}

	planner::Coord PathFinder::get_source( ) {
		return source_;
	}

	void PathFinder::set_destination( const planner::Coord &destination ) {
		destination_ = destination;
	}

	planner::Coord PathFinder::get_destination( ) {
		return destination_;
	}

	bool PathFinder::find_path( std::vector<std::vector<int> >& grid,
											        const planner::MapParameters& map_params, 
											        const planner::heuristic::TYPE& heuristic )
	{

		// Assert that map is not empty
		if ( grid.empty() )
		{
			std::cout << "[ERROR] Map is empty" << std::endl;
			return false;
		}

		// Assert heuristic
		if ( planner::heuristic::TYPE::NOT_SUPPORTED == heuristic )
		{
			std::cout << "[ERROR] Heuristic is not valid" << std::endl;
			return false;
		}

		path_.clear();

		search_algorithm_( grid, map_params, source_, destination_, 
											 heuristic, path_ );

		if ( path_.empty() )
		{
			std::cout << "[ERROR] Could not find path" << std::endl;
			return false; 
		}
		return false;
	}

	void PathFinder::print() {
		std::cout << "[INFO] Planner configuration" << std::endl
		          << "\t Source: "           << source_
		          << "\t Destination: "      << destination_
		          << "\t Search Algorithm: " << search_algorithm::NAME.find( 
		          	                            algorithm_ )->second << std::endl
		          << std::endl;
	}
} // End of namespace planner 
