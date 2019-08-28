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
		// Initialize direction vector.
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

		// Set heuristic and number of possible directions.
		num_directions_ = 4;
		heuristic_ = planner::heuristic::TYPE::MANHATTAN;
		set_heuristic( heuristic_ );

		// Set source and destination to default values. 
		source_ = destination_ = { INT_MAX, INT_MAX };

		// Set the algorithm. 
		search_algorithm_ = planner::search_algorithm::TYPE::ASTAR;
		set_search_algorithm( search_algorithm_ );
	}

	PathFinder::~PathFinder() {
		/* Not implemented */
	}

	bool PathFinder::set_search_algorithm( 
		 														const planner::search_algorithm::TYPE &search ) 
	{
		if ( planner::search_algorithm::TYPE::NOT_SUPPORTED == search )
		{
			std::cout << "[ERROR] Search algorithm is not supported\n";
			return false;
		}

		search_algorithm_ = search;
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

		// search_algorithm_( bin_map, map_params, source_, destination_, 
		// 									 heuristic, path_ );

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

	bool PathFinder::set_heuristic( const heuristic::TYPE& heuristic ) 
	{
		switch( heuristic ) 
		{
			// Bind euclidean function to the heuristic function and set
			// the number of directions to 8.
			case planner::heuristic::TYPE::EUCLIDEAN:	
				heuristic_func_ = std::bind( planner::heuristic::Function::euclidean, 
					                   		 		 std::placeholders::_1, 
					                   		     std::placeholders::_2 ); 
				num_directions_ = 8;
				break;

			// Bind manhattan function to the heuristic function and set
			// the number of directions to 4.
			case planner::heuristic::TYPE::MANHATTAN:	
				heuristic_func_ = std::bind( planner::heuristic::Function::manhattan, 
					                   		     std::placeholders::_1, 
					                   		     std::placeholders::_2 ); 
				num_directions_ = 4;
				break;

			// Bind octagonal function to the heuristic function and set
			// the number of directions to 8.
			case planner::heuristic::TYPE::OCTAGONAL:	
				heuristic_func_ = std::bind( planner::heuristic::Function::octagonal, 
					                           std::placeholders::_1, 
					                           std::placeholders::_2 ); 
				num_directions_ = 8;
				break;

			// Heuristic not supported. 
			case planner::heuristic::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Heuristic is not supported " << std::endl;
				return false;
		}
		return true;
	}

	void PathFinder::print() 
	{
		std::cout << "[INFO] Planner configuration\n"
		          << "\tSource: "           << source_
		          << "\tDestination: "      << destination_
		          << "\tSearch Algorithm: " << planner::search_algorithm::NAME.find( 
		          														 search_algorithm_ )->second << '\n'
		          << "\tHeuristic: "        << planner::heuristic::NAME.find( 
		          	                           heuristic_ )->second << '\n';
	}

	bool PathFinder::is_coord_valid( const planner::Coord &coordinate, 
			                           const planner::MapParameters &map_param ) 
	{
		return ( coordinate.r >= 0 && coordinate.r < map_param.height_ && 
			       coordinate.c >= 0 && coordinate.c < map_param.width_ );
	}

	bool PathFinder::is_coord_destination( const planner::Coord &coordinate, 
										                   const planner::Coord &destination ) 
	{
		return coordinate == destination;
	}

	bool PathFinder::is_coord_blocked( const planner::Coord &coordinate,
									                 const std::vector<std::vector<int>> &bin_map ) 
	{
		return BLOCKED == bin_map[ coordinate.r ][ coordinate.c ];
	}

	// void PathFinder::get_path( std::vector<planner::Coord> &path, 
	// 												 const std::vector<std::vector<planner::Node>> &nodes, 
	// 	                       const planner::Coord &destination ) 
	// {
	// 	path.clear();
	// 	planner::Coord temp = destination;

	// 	while( !(nodes[ temp.r ][ temp.c ].parent == temp ) ) 
	// 	{
	// 		path.push_back( temp );
	// 		temp = nodes[ temp.r ][ temp.c ].parent;
	// 	}
	// 	path.push_back( temp );
	// }
} // End of namespace planner 
