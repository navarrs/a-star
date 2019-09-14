/* -------------------------------------------------------------------------- *
 * @file:   path_finder.h
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the definitions for the class PathFinder. This 
 *          class used for path finding using a search algorithm.  
 *
 * -------------------------------------------------------------------------- */

#ifndef PATH_GENERATOR
#define PATH_GENERATOR

#include "planner.h"

namespace planner {

class PathFinder {
	public:
		/* ---------------------------------------------------------------------- *
		 * @name:   PathFinder
		 * @brief:  Constructor for the class PathFinder.
		 *
		 * @param:  
		 *
		 * @return: Creates an instance of a path generator with default 
		 *          parameters. 
		 * */
		PathFinder();

		/* ---------------------------------------------------------------------- *
		 * @name:   PathFinder
		 * @brief:  Destructor for the class PathFinder.
		 *
		 * @param:  
		 *
		 * @return: Creates an instance of a path generator with default 
		 *          parameters. 
		 * */
		~PathFinder();

		/* ---------------------------------------------------------------------- *
		 * @name:   set_search_algorithm
		 * @brief:  Sets the algorithm of type planner::search_algorithm::TYPE 
		 *          that will be used to perform path finding. 
		 *
		 * @param:  search(in): Type of search algorithm. Currently supports:
		 *            1) A-Star.
		 *
		 * @return: False if: 
		 *						1) The search algorithm is not supported. 
		 *					Otherwise, true. 
		 * */
		bool set_search_algorithm(const planner::search_algorithm::TYPE &search);

		/* ---------------------------------------------------------------------- *
		 * @name:   set_heuristic
		 * @brief:  Sets type of heuristic to use to optimize path finding.
		 *
		 * @param:  heuristic(in): Heuristic function to use. Currently supports.
		 *            1) Euclidean distance. 
		 *						2) Manhattan distance. 
		 *            3) Octagonal distance. 
		 *
		 * @return: False if:
		 *						1) Heuristic function is not supported. 
		 *					Otherwise, true.
		 * */
		bool set_heuristic(const planner::heuristic::TYPE& heuristic); 

		/* ---------------------------------------------------------------------- *
		 * @name:   set_source
		 * @brief:  Sets the starting coordinate. 
		 *
		 * @param:  source(in): Struct of type planner::Coord that represents the 
		 *          starting point for the planning. 
		 *
		 * @return: False if: 
		 *						1) The coordinate has invalid values.  
		 *					Otherwise, true. 
		 * */
		bool set_source(const planner::Coord &source);

		/* ---------------------------------------------------------------------- *
		 * @name:   get_source
		 * @brief:  Gets a struct of type planner::Coord that represents the 
		 *          starting coordinate. 
		 *
		 * @param: 
		 *
		 * @return: Struct of type planner::Coord that represents the starting 
		 *          coordinate. 
		 * */
		planner::Coord get_source();

		/* ---------------------------------------------------------------------- *
		 * @name:   set_destination
		 * @brief:  Sets the destination coordinate. 
		 *
		 * @param:  destination(in): Struct of type planner::Coord that represents 
		 *          the starting point for the planning. 
		 *
		 * @return: False if: 
		 *						1) The coordinate has invalid values.  
		 *					Otherwise, true. 
		 * */
		bool set_destination(const planner::Coord &destination);

		/* ---------------------------------------------------------------------- *
		 * @name:   get_destination
		 * @brief:  Gets a struct of type planner::Coord that represents the 
		 *          destination coordinate. 
		 *
		 * @param: 
		 *
		 * @return: Struct of type planner::Coord that represents the destination 
		 *          coordinate. 
		 * */
		planner::Coord get_destination();

		/* ---------------------------------------------------------------------- *
		 * @name:   print
		 * @brief:  Prints path finder configuration. 
		 *
		 * @param: 
		 *
		 * @return: 
		 * */
		void print();

		/* ---------------------------------------------------------------------- *
		 * @name:   find_path
		 * @brief:  Calls search algorithm method to find a path given starting
		 *          and destination coordinates, a heuristic and the binary map
		 *				  along with its configuration. 
		 *
		 * @param:  bin_map(in): Binary representation of the map. 
		 * @param:  map_params(in): Struct of type planner::MapParams that contains 
		 *          the parameters used to create the map. 
		 *
		 * @return: False if:
		 *						1) Binary map is empty.
		 *            2) Heuristic is not supported. 
		 *            3) No path was found. 
		 *          Otherwise, true.
		 * */
		bool find_path(std::vector<std::vector<unsigned int>>& bin_map,
									const planner::MapParameters& map_params);

		/* ---------------------------------------------------------------------- *
		 * @name:   get_path
		 * @brief:  Gets a vector containing the coordinates of the path found by
		 *          the search algorithm.
		 * 
		 * @param: 
		 *
		 * @return: Vector of coordinates of type planner::Coord that represent the 
		 *          path obtained by the search algorithm.
		 * */
		std::vector<planner::Coord> get_path();

	private:

		/* ---------------------------------------------------------------------- *
		 * @name:   is_coord_in_range
		 * @brief:  Checks if a Coordinate is withing the world's range.
		 *
		 * @param:  coordinate(in): Coordinate to evaluate.
		 * @param:  map_params(in):  Configuration parameters of map. 
		 *
		 * @return: False if:
		 *						1) Coordinate is not within world's range. 
		 *					Otherwise, true.
		 * */
		 inline bool is_coord_in_range(const planner::Coord &coordinate, 
			                             const planner::MapParameters &map_params);

		/* ---------------------------------------------------------------------- *
		 * @name:   is_coord_destination
		 * @brief:  Checks if a coordinate is the same as the destination.
		 *
		 * @param:  coordinate(in):  Coordinate to evaluate. 
		 *
		 * @return: True if:
		 *						1) Coordinate is the destination. 
		 *					Otherwise, false.
		 * */
		 inline bool is_coord_destination(const planner::Coord &coordinate);

		/* ---------------------------------------------------------------------- *
		 * @name:   is_coord_blocked
		 * @brief:  Checks if a coordinate is blocked.
		 *
		 * @param:  coordinate(in): Coordinate to evaluate.
		 * @param:  bin_map(in): Binary representation of the map.
		 *
		 * @return: True if:
		 *						1) Coordinate is blocked. 
		 *					Otherwise, false.
		 * */
		 inline bool is_coord_blocked(const planner::Coord &coordinate, 
			                      const std::vector<std::vector<unsigned int>> &bin_map);


		 /* --------------------------------------------------------------------- *
		 * @name:   astar
		 * @brief:  Implementation of A* search for path finding. 
		 *
		 * @param:  bin_map(in): Binary representation of the map.
		 * @param:  map_param(in): Parameters used to create the map.
		 *
		 * @return: False if:
		 *						1) Either source or destination are out of range. 
		 *            2) Either source or destination are blocked. 
		 *            3) No destination was found. 
		 *					Otherwise, true.
		 * */
		 bool astar(std::vector<std::vector<unsigned int>> &bin_map, 
								const planner::MapParameters  &map_param);

		/* ---------------------------------------------------------------------- *
		 * @name:   get_path
		 * @brief:  Gets the path found by the search algorithm.
		 *
		 * @param:  nodes(in): Vector of nodes to generate the path. 
		 *
		 * @return: 
		 * */
		 void get_path(const std::vector<std::vector<planner::Node>> &nodes); 


	 	/* ---------------------------------------------------------------------- *
		 * Members 
		 * */

	  // Vector that contains possible directions (e.g. north, south, etc...)
		std::vector<planner::Coord> directions_;

		// Number of allowed directions. 
		int num_directions_;

		// Heuristic function.
		planner::heuristic::TYPE heuristic_;
		std::function<unsigned int(const planner::Coord&, 
			                         const planner::Coord&)> heuristic_func_;

		// Source and destination coordinates. 
		planner::Coord source_;
		planner::Coord destination_;

		// Vector to store the path. 
		std::vector<planner::Coord> path_;

		// Search algorithm.
		planner::search_algorithm::TYPE search_algorithm_;

}; // End of class PathFinder
} // End of namespace planner

#endif