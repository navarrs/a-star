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

namespace planner {

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
							<< source.r << "," << source.c << ")\n";

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
							<< destination.r << "," << destination.c << ")\n";

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
										        const planner::MapParameters& map_paramss )
{

	// Assert that map is not empty
	if ( bin_map.empty() )
	{
		std::cout << "[ERROR] Binary map is empty\n";
		return false;
	}

	// Assert heuristic
	if ( planner::heuristic::TYPE::NOT_SUPPORTED == heuristic_ )
	{
		std::cout << "[ERROR] Heuristic is not supported\n";
		return false;
	}

	path_.clear();

	switch( search_algorithm_ )
	{
		// Perform path finding using A* search algorithm.
		case planner::search_algorithm::TYPE::ASTAR:
			astar( bin_map, map_paramss );
			break;
		// Search algorithm is not supported. 
		case planner::search_algorithm::TYPE::NOT_SUPPORTED:
		default:
			std::cout << "[ERROR] Search algorithm is not supported\n";
			return false;
	}

	// Check if path was not found. 
	if ( path_.empty() )
	{
		std::cout << "[ERROR] Could not find path\n";
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
			std::cout << "[ERROR] Heuristic is not supported\n";
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

bool PathFinder::is_coord_in_range( const planner::Coord &coordinate, 
		                                const planner::MapParameters &map_params ) 
{
	return ( 0 <= coordinate.r && map_params.height_ > coordinate.r  && 
		       0 <= coordinate.c && map_params.width_  > coordinate.c );
}

bool PathFinder::is_coord_destination( const planner::Coord &coordinate ) 
{
	return destination_ == coordinate;
}

bool PathFinder::is_coord_blocked( const planner::Coord &coordinate,
								                   const std::vector<std::vector<int>> &bin_map ) 
{
	return BLOCKED == bin_map[ coordinate.r ][ coordinate.c ];
}

bool PathFinder::astar( std::vector<std::vector<int>> &bin_map, 
												const planner::MapParameters  &map_params )
{

	// Assert that neither source nor destination are out of range.
	if ( !is_coord_in_range( source_, map_params ) ) 
	{
		std::cout << "[ERROR] Source is invalid\n";
		return false;
	}
	if ( !is_coord_in_range( destination_, map_params ) ) 
	{
		std::cout << "[ERROR] Destination is invalid\n";
		return false;
	}

	// Assert that neither source nor destination are blocked.
	if ( is_coord_blocked( source_, bin_map ) ) 
	{
		std::cout << "[ERROR] Source is blocked\n";
		return false;
	}
	if ( is_coord_blocked( destination_, bin_map ) ) 
	{
		std::cout << "[ERROR] Destination is blocked\n";
		return false;
	}

	// Assert that source is not destination.
	if ( is_coord_destination( source_ ) ) 
	{
		std::cout << "[WARN] Already at destination\n";
		return true;
	}

	bool closed_list[ map_params.height_ ][ map_params.width_ ];
	memset( closed_list, false, sizeof( closed_list ) );

	// Initialize path with default value.
	std::vector<std::vector<planner::Node>> nodes;
	int r, c;
	for( r = 0; r < map_params.height_; r++ )
	{
		std::vector<planner::Node> node;
		for( c = 0; c < map_params.width_; c++ )
		{
			node.push_back( { { -1, -1 }, INT_MAX } );
		}
		nodes.push_back( node );
	}	

	// Initialize the parameters of the starting node.
	r = source_.r;
	c = source_.c;
	nodes[ r ][ c ].h = 0;
	nodes[ r ][ c ].parent = source_;
	
	planner::Node *temp_node = new Node( { source_, 0 } );
	std::set<planner::Node*> open_list;
	open_list.insert( temp_node );
	bool found_dst{ false };

	while( !open_list.empty() ) 
	{
		// Get top node
		temp_node = *open_list.begin();
		planner::Coord current_coord = temp_node->parent;

		// Erase from list of open nodes and add it to closed nodes
		open_list.erase( open_list.begin() );
		r = current_coord.r;
		c = current_coord.c;
		closed_list[ r ][ c ] = true;
		
		// Generate all successors
		unsigned int  h_temp;
		for ( size_t i = 0; i < num_directions_; i++ ) 
		{

			planner::Coord move = current_coord + directions_[ i ];

			// Assert that Coordinate is valid
			if ( is_coord_in_range( move, map_params ) ) 
			{
				// If its destination, finish
				if ( is_coord_destination( move ) ) 
				{
					nodes[ move.r ][ move.c ].parent = { r, c };
					std::cout << "[DONE] Found destination\n";
					found_dst = true;
					get_path( nodes );
					return true;
				} 
				// Not the destination 
				else if ( !closed_list[ move.r ][ move.c ] && 
					        !is_coord_blocked( move, bin_map ) ) 
				{
				  // Compute heuristic and insert to open if we get a better value than
				  // the current value. 
					h_temp = heuristic_func_( move, destination_ );

					// Check if a better heuristic was computed.
					if ( INT_MAX == nodes[ move.r ][ move.c ].h  || 
						   h_temp  <  nodes[ move.r ][ move.c ].h   ) 
					{
						temp_node = new Node( { move, h_temp } );
						open_list.insert( temp_node );
						nodes[ move.r ][ move.c ].h      = h_temp;
						nodes[ move.r ][ move.c ].parent = { r, c };
					}
				}
			}
		}
	}

	// If path was not found.
	if ( !found_dst ) 
	{
		std::cout << "[FATAL] Could not find destination\n";
		path_.clear();
		return found_dst;
	}
}

void PathFinder::get_path( const std::vector<std::vector<planner::Node>> &nodes ) 
{
	path_.clear();
	planner::Coord temp = destination_;

	while( !( temp == nodes[ temp.r ][ temp.c ].parent ) ) 
	{
		path_.push_back( temp );
		temp = nodes[ temp.r ][ temp.c ].parent;
	}
	path_.push_back( temp );
} 

} // End of namespace planner. 
