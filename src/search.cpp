#include "planner.h"

namespace planner {

	namespace search_algorithm {

		Function::Function() {
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
				
				set_heuristic( planner::heuristic::TYPE::MANHATTAN );
				num_directions_ = 4 ;
		}

		bool Function::astar( std::vector<std::vector<int>> &grid, 
													const planner::MapParameters &map_param, 
													const planner::Coord &src, 
													const planner::Coord &dst,
													const planner::heuristic::TYPE &heuristic,
													std::vector<planner::Coord> &path )
		{
			// Assert that neither source nor destination are out of range.
			if ( !is_coord_valid( src, map_param ) ) {
				std::cout << "[ERROR] Source is invalid" << std::endl;
				return EXIT_FAILURE;
			}

			if ( !is_coord_valid( dst, map_param ) ) {
				std::cout << "[ERROR] Destination is invalid" << std::endl;
				return EXIT_FAILURE;
			}

			// Assert that neither source nor destination are blocked.
			if ( !is_coord_blocked( src, grid ) ) {
				std::cout << "[ERROR] Source is blocked" << std::endl;
				return EXIT_FAILURE;
			}
			if ( !is_coord_blocked( dst, grid ) ) {
				std::cout << "[ERROR] Destination is blocked" << std::endl;
				return EXIT_FAILURE;
			}

			// Assert that source is not destination.
			if ( is_coord_destination( src, dst ) ) {
				std::cout << "[WARN] Already at destination" << std::endl;
				return EXIT_SUCCESS;
			}

			// Set heuristic function 
			//set_heuristic( heuristic );


		}



		// 	bool closed_list[ map_param.height_ ][ map_param.width_ ];
		// 	memset( closed_list, false, sizeof( closed_list ) );

		// 	// Initialize path. 
		// 	std::vector<std::vector<planner::Node>> nodes;
		// 	int r = 0, c = 0;
		// 	for( ; r < map_param.height_; r++ ) {
		// 		std::vector<Node> node;
		// 		for( ; c < map_param.width_; c++ ) {
		// 			node.push_back( { {-1, -1}, INT_MAX } );
		// 		}
		// 		nodes.push_back( node );
		// 	}

		// 	// Initialize the parameters of the starting node.
		// 	r = src.r;
		// 	c = src.c;
		// 	nodes[ r ][ c ].h = 0;
		// 	nodes[ r ][ c ].parent = { r, c };

		// 	planner::Node *temp_node = new Node( { src, 0 } );
		// 	std::set<planner::Node*> open_list;
		// 	open_list.insert( temp_node );
		// 	bool found_dst{ false };

		// 	// while(!open_list.empty()) {
		// 	// 	// Get top node
		// 	// 	temp_node = *open_list.begin();
		// 	// 	planner::Coord current_coord = temp_node->parent;

		// 	// 	// Erase from list of open nodes and add it to closed nodes
		// 	// 	open_list.erase( open_list.begin() );
		// 	// 	r = current_coord.r;
		// 	// 	c = current_coord.c;
		// 	// 	closed_list[ r ][ c ] = true;
				
		// 	// 	// Generate all successors
		// 	// 	unsigned int  h_temp;
		// 	// 	for ( unsigned int i = 0; i < num_directions; i++ ) {
		// 	// 		Coord move = current_Coord + directions[i];

		// 	// 		// Assert that Coordinate is valid
		// 	// 		if (is_Coord_valid(move)) {
		// 	// 			// If its destination, finish
		// 	// 			if (is_Coord_destination(move, dst)) {
		// 	// 				nodes[move.r][move.c].parent = {r, c};
		// 	// 				std::cout << "Found destination" << std::endl;
		// 	// 				found_dst = true;
		// 	// 				// compute_path(nodes, path);
		// 	// 				return;
		// 	// 			} else if (!closed_list[move.r][move.c] && !is_Coord_blocked(grid, move)) {
		// 	// 			// Otherwise, compute heuristic and insert to open if we get a better heuristic. 
		// 	// 				h_temp = heuristic(move, dst);
		// 	// 				if (nodes[move.r][move.c].h == INT_MAX || nodes[move.r][move.c].h > h_temp) {
		// 	// 					temp = new node({move, h_temp});
		// 	// 					open_list.insert(temp);
		// 	// 					nodes[move.r][move.c].h = h_temp;
		// 	// 					nodes[move.r][move.c].parent = {r, c};
		// 	// 				}
		// 	// 			}
		// 	// 		}
		// 	// 	}
		// 	// }

		// 	return EXIT_SUCCESS;
		// }

		bool Function::set_heuristic( const heuristic::TYPE &heuristic ) {
		
		switch( heuristic ) {
			case planner::heuristic::TYPE::EUCLIDEAN:
				// Bind euclidean function to the heuristic function and set
				// the number of directions to 8.
				heuristic_func_ = std::bind( heuristic::Function::euclidean, 
					                   		 		 std::placeholders::_1, 
					                   		     std::placeholders::_2 ); 
				num_directions_ = 8;
				break;
			case planner::heuristic::TYPE::MANHATTAN:
				// Bind manhattan function to the heuristic function and set
				// the number of directions to 4.
				heuristic_func_ = std::bind( heuristic::Function::manhattan, 
					                   		 std::placeholders::_1, 
					                   		 std::placeholders::_2 ); 
				num_directions_ = 4;
				break;
			case planner::heuristic::TYPE::OCTAGONAL:
				// Bind octagonal function to the heuristic function and set
				// the number of directions to 8.
				heuristic_func_ = std::bind( heuristic::Function::octagonal, 
					                         std::placeholders::_1, 
					                         std::placeholders::_2 ); 
				num_directions_ = 8;
				break;
			case planner::heuristic::TYPE::NOT_SUPPORTED:
			default:
				std::cout << "[ERROR] Heuristic is not supported " 
				          << std::endl;
				return EXIT_FAILURE;
		}
		return EXIT_SUCCESS;
	}

		bool Function::is_coord_valid( const planner::Coord &co, 
			                           const planner::MapParameters &map_param ) {
			return ( co.r >= 0 && co.r < map_param.width_ && 
				     co.c >= 0 && co.c < map_param.height_ );
		}

		bool Function::is_coord_destination( const planner::Coord &co, 
											                   const planner::Coord &dst ) {
			return co == dst;
		}

		bool Function::is_coord_blocked( const planner::Coord &co,
										 const std::vector<std::vector<int>> &grid ) {
			return grid[ co.r ][ co.c ] == FREE;
		}


			

	// 	if (!found_dst) {
	// 		std::cout << "Could not find destination." << std::endl;
	// 		path = std::stack<Coord>();
	// 		return;
	// 	}
//	}
// /* @method: compute_path(...)
	//    @brief: Produces final path. */
	// void PathGenerator::compute_path(std::vector<std::vector<node> > &nodes, std::stack<Coord> &path) {
	// 	path = std::stack<Coord>();
	// 	Coord temp = dst;
	// 	while(!(nodes[temp.r][temp.c].parent == temp)) {
	// 		path.push(temp);
	// 		temp = nodes[temp.r][temp.c].parent;
	// 	}
	// 	path.push(temp);
	// }
	} // End of namespace search algorithm
} // End of namespace planner 