#include "planner.h"

namespace planner 
{
	namespace search_algorithm 
	{

		// Function::Function() 
		// {
		// 		// Moving directions
		// 		directions_ = 
		// 		{
		// 			{  0,  1 },  // Right 
		// 			{  1,  0 },  // Down 
		// 			{  0, -1 },  // Left
		// 			{ -1,  0 },  // Up
		//       { -1, -1 },  // Up-left
		//       {  1,  1 },  // Down-right
		//       { -1,  1 },  // Up-right
		//       {  1, -1 }   // Down-left 
		// 		};
				
		// 		num_directions_ = -1;
		// }

		bool Function::astar( std::vector<std::vector<int>> &bin_map, 
													const planner::MapParameters &map_param, 
													const planner::Coord &source, 
													const planner::Coord &destination,
													const planner::heuristic::TYPE &heuristic,
													std::vector<planner::Coord> &path )
		{

			// Assert that neither source nor destination are out of range.
			if ( !is_coord_valid( source, map_param ) ) 
			{
				std::cout << "[ERROR] Source is invalid" << std::endl;
				return false;
			}
			if ( !is_coord_valid( destination, map_param ) ) 
			{
				std::cout << "[ERROR] Destination is invalid" << std::endl;
				return false;
			}

			// Assert that neither source nor destination are blocked.
			if ( !is_coord_blocked( source, bin_map ) ) 
			{
				std::cout << "[ERROR] Source is blocked" << std::endl;
				return false;
			}
			if ( !is_coord_blocked( destination, bin_map ) ) 
			{
				std::cout << "[ERROR] Destination is blocked" << std::endl;
				return false;
			}

			// Assert that source is not destination.
			if ( is_coord_destination( source, destination ) ) 
			{
				std::cout << "[WARN] Already at destination" << std::endl;
				return true;
			}

			// Directions
			std::vector<planner::Coord> directions_;
			directions_ = 
			{
				{  0,  1 },  // Right 
				{  1,  0 },  // Down 
				{  0, -1 },  // Left
				{ -1,  0 },  // Up
	      { -1, -1 },  // Up-left
	      {  1,  1 },  // Down-right
	      { -1,  1 },  // Up-right
	      {  1, -1 }   // Down-left 
			};

			// Set heuristic function 
			std::function<unsigned int( const planner::Coord&, 
				                          const planner::Coord& )> heuristic_func_;
			int num_directions_;
			
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

			bool closed_list[ map_param.height_ ][ map_param.width_ ];
			memset( closed_list, false, sizeof( closed_list ) );

			// Initialize path with default value.
			std::vector<std::vector<planner::Node>> nodes;
			int r = 0, c = 0;
			for(  ; r < map_param.height_; r++ )
			{
				std::vector<planner::Node> node;
				for( ; c < map_param.width_; c++ )
				{
					node.push_back( { { -1, -1 }, INT_MAX } );
				}
				c = 0;
				nodes.push_back( node );
			}	

			// Initialize the parameters of the starting node.
			r = source.r;
			c = source.c;
			nodes[ r ][ c ].parent = source;
			
			planner::Node *temp_node = new Node( { source, 0 });
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
				for ( unsigned int i = 0; i < num_directions_; i++ ) 
				{
					planner::Coord move = current_coord + directions_[ i ];

					// Assert that Coordinate is valid
					if ( is_coord_valid( move, map_param ) ) 
					{
						std::cout << "GOT HERE" << std::endl;
						// If its destination, finish
						if ( is_coord_destination( move, destination ) ) 
						{
							nodes[ move.r ][ move.c ].parent = { r, c };
							std::cout << "[DONE] Found destination" << std::endl;
							found_dst = true;
							get_path( path, nodes, destination );
							return true;
						} 
						else if ( !closed_list[ move.r ][ move.c ] && 
											!is_coord_blocked( move, bin_map ) ) 
						{
						  // Otherwise, compute heuristic and insert to open if we get a better heuristic. 
							h_temp = heuristic_func_( move, destination );
							if ( nodes[ move.r ][ move.c ].h == INT_MAX || 
								   nodes[ move.r ][ move.c ].h > h_temp    ) 
							{
								temp_node = new Node( { move, h_temp } );
								open_list.insert( temp_node );
								nodes[ move.r ][ move.c ].h = h_temp;
								nodes[ move.r ][ move.c ].parent = { r, c };
							}
						}
					}
				}
			}

			// If path was not found.
			if ( !found_dst ) 
			{
				std::cout << "[SEARCH] Could not find destination." << std::endl;
				path.clear();
				return found_dst;
			}
		}

		// TODO: Make this one work.
		bool Function::set_heuristic( const heuristic::TYPE& heuristic ) 
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

		bool Function::is_coord_valid( const planner::Coord &coordinate, 
			                             const planner::MapParameters &map_param ) 
		{
			return ( coordinate.r >= 0 && coordinate.r < map_param.width_ && 
				       coordinate.c >= 0 && coordinate.c < map_param.height_ );
		}

		bool Function::is_coord_destination( const planner::Coord &coordinate, 
											                   const planner::Coord &destination ) 
		{
			return coordinate == destination;
		}

		bool Function::is_coord_blocked( const planner::Coord &coordinate,
										                 const std::vector<std::vector<int>> &bin_map ) 
		{
			return bin_map[ coordinate.r ][ coordinate.c ] == FREE;
		}

		void Function::get_path( std::vector<planner::Coord> &path, 
														 const std::vector<std::vector<planner::Node>> &nodes, 
			                       const planner::Coord &destination ) 
		{
			path.clear();
			Coord temp = destination;

			while( !(nodes[ temp.r ][ temp.c ].parent == temp ) ) 
			{
				path.push_back( temp );
				temp = nodes[ temp.r ][ temp.c ].parent;
			}
			path.push_back( temp );
		}
	} // End of namespace search algorithm
} // End of namespace planner 