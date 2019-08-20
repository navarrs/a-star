/**
 * @file:   path_generator.cpp
 * @author: Ingrid Navarro
 * @brief:  This file contains the implementations for the class PathGenerator.  
 *          This class used for path finding using the Astar algorithm. 
 */

#include "planner.h"
#include "path_generator.h"


namespace planner {

	PathGenerator::PathGenerator() {
		directions_ = {
			{  0,  1 },  // Right 
			{  1,  0 },  // Down 
			{  0, -1 },  // Left
			{ -1,  0 },  // Up
	        { -1, -1 },  // Up-left
	        {  1,  1 },  // Down-right
	        { -1,  1 },  // Up-right
	        {  1, -1 }   // Down-left 
		};
		source_ = destination_ = { INT_MAX, INT_MAX };

		heuristic_ = heuristic::TYPE::MANHATTAN;
		set_heuristic( heuristic_ );
		num_directions_ = 4 ;

		algorithm_ = search_algorithm::TYPE::ASTAR;
		set_search_algorithm( algorithm_ );
	}

	PathGenerator::~PathGenerator() {
		/* Not implemented */
	}

	bool PathGenerator::set_heuristic( const heuristic::TYPE &h ) {
		
		heuristic_ = h;

		switch( heuristic_ ) {
			case heuristic::TYPE::EUCLIDEAN:
				// Bind euclidean function to the heuristic function and set
				// the number of directions to 8.
				heuristic_func_ = std::bind( heuristic::Function::euclidean, 
					                   		 std::placeholders::_1, 
					                   		 std::placeholders::_2 ); 
				num_directions_ = 8;
				break;
			case heuristic::TYPE::MANHATTAN:
				// Bind manhattan function to the heuristic function and set
				// the number of directions to 4.
				heuristic_func_ = std::bind( heuristic::Function::manhattan, 
					                   		 std::placeholders::_1, 
					                   		 std::placeholders::_2 ); 
				num_directions_ = 4;
				break;
			case heuristic::TYPE::OCTAGONAL:
				// Bind octagonal function to the heuristic function and set
				// the number of directions to 8.
				heuristic_func_ = std::bind( heuristic::Function::octagonal, 
					                         std::placeholders::_1, 
					                         std::placeholders::_2 ); 
				num_directions_ = 8;
				break;
			case heuristic::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Heuristic is not supported " 
				          << std::endl;
				return EXIT_FAILURE;
		}
		return EXIT_SUCCESS;
	}

	bool PathGenerator::set_search_algorithm( const search_algorithm::TYPE &s ) {
		switch( s ) {
			case search_algorithm::TYPE::ASTAR:
				// Bind astar algorithm to the search function. 
				search_algorithm_ = std::bind( search_algorithm::Function::astar, 
				                               std::placeholders::_1,
				                               std::placeholders::_2,
				                               std::placeholders::_3,
				                               std::placeholders::_4 );
				break;
			case search_algorithm::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Search algorithm is not supported " 
				          << std::endl;
				return EXIT_FAILURE;

		}
		return EXIT_SUCCESS;
	}

	void PathGenerator::set_source( const Coord &source ) {
		source_ = source;
	}

	Coord PathGenerator::get_source( ) {
		return source_;
	}

	void PathGenerator::set_destination( const Coord &destination ) {
		destination_ = destination;
	}

	Coord PathGenerator::get_destination( ) {
		return destination_;
	}

	void PathGenerator::print() {
		std::cout << "[INFO] Planner configuration" << std::endl
		          << "\t Source: "           << source_
		          << "\t Destination: "      << destination_
		          << "\t Heuristic: "        << heuristic::NAME.find( 
		          	                            heuristic_ )->second << std::endl 
		          << "\t Search Algorithm: " << search_algorithm::NAME.find( 
		          	                            algorithm_ )->second << std::endl
		          << std::endl;
	}
} // End of namespace planner 
