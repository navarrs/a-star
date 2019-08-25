/* --------------------------------------------------------------------------- *
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

namespace planner 
{
	class PathFinder 
	{
		public:
			/* -------------------------------------------------------------------- *
			 * @name:   PathFinder
			 * @brief:  Constructor for the class PathFinder.
			 *
			 * @param:  
			 *
			 * @return: Creates an instance of a path generator with default 
			 *          parameters. 
			 * ------------------------------------------------------------------- */
			PathFinder();

			/* -------------------------------------------------------------------- *
			 * @name:   PathFinder
			 * @brief:  Destructor for the class PathFinder.
			 *
			 * @param:  
			 *
			 * @return: Creates an instance of a path generator with default 
			 *          parameters. 
			 * ------------------------------------------------------------------- */
			~PathFinder();

			/* -------------------------------------------------------------------- *
			 * @name:   set_search_algorithm
			 * @brief:  Sets the algorithm of type planner::search_algorithm::TYPE 
			 *          that will be used to perform path finding. 
			 *
			 * @param:  s: Type of search algorithm.
			 *
			 * @return: False if: 
			 *						1) The search algorithm is not supported. 
			 *					Otherwise, true. 
			 * ------------------------------------------------------------------- */
			bool set_search_algorithm( const planner::search_algorithm::TYPE &s );

			/* -------------------------------------------------------------------- *
			 * @name:   set_source
			 * @brief:  Sets the starting coordinate. 
			 *
			 * @param:  source: Struct of type planner::Coord that represents the 
			 *          starting point for the planning. 
			 *
			 * @return: False if: 
			 *						1) The coordinate has invalid values.  
			 *					Otherwise, true. 
			 * ------------------------------------------------------------------- */
			bool set_source( const planner::Coord &source );

			/* -------------------------------------------------------------------- *
			 * @name:   get_source
			 * @brief:  Gets a struct of type planner::Coord that represents the 
			 *          starting coordinate. 
			 *
			 * @param: 
			 *
			 * @return: Struct of type planner::Coord that represents the starting 
			 *          coordinate. 
			 * ------------------------------------------------------------------- */
			planner::Coord get_source();

			/* -------------------------------------------------------------------- *
			 * @name:   set_destination
			 * @brief:  Sets the destination coordinate. 
			 *
			 * @param:  destination: Struct of type planner::Coord that represents the 
			 *          starting point for the planning. 
			 *
			 * @return: False if: 
			 *						1) The coordinate has invalid values.  
			 *					Otherwise, true. 
			 * ------------------------------------------------------------------- */
			bool set_destination( const planner::Coord &destination );

			/* -------------------------------------------------------------------- *
			 * @name:   get_destination
			 * @brief:  Gets a struct of type planner::Coord that represents the 
			 *          destination coordinate. 
			 *
			 * @param: 
			 *
			 * @return: Struct of type planner::Coord that represents the destination 
			 *          coordinate. 
			 * ------------------------------------------------------------------- */
			planner::Coord get_destination( );

			/* -------------------------------------------------------------------- *
			 * @name:   print
			 * @brief:  Prints path finder configuration. 
			 *
			 * @param: 
			 *
			 * @return: 
			 * ------------------------------------------------------------------- */
			void print();

			/* -------------------------------------------------------------------- *
			 * @name:   find_path
			 * @brief:  Calls search algorithm method to find a path given starting
			 *          and destination coordinates, a heuristic and the binary map
			 *				  along with its configuration. 
			 *
			 * @param:  bin_map: Binary representation of the map. 
			 * @param:  map_params: Struct of type planner::MapParams that contains 
			 *          the parameters used to create the map.
			 * @param:  heuristic: Struct of type planner::heuristic that represents 
			 *          the heuristic function to used for planning.  
			 *
			 * @return: False if:
			 *						1) Binary map is empty.
			 *            2) Heuristic is not supported. 
			 *            3) No path was found. 
			 *          Otherwise, true.
			 * ------------------------------------------------------------------- */
			bool find_path( std::vector<std::vector<int>>& bin_map,
											const planner::MapParameters& map_params, 
											const planner::heuristic::TYPE& heuristic );

			/* -------------------------------------------------------------------- *
			 * @name:   get_path
			 * @brief:  Gets a vector containing the coordinates of the path found by
			 *          the search algorithm. 
			 *
			 * @return: Vector of coordinates that represent the path. 
			 * ------------------------------------------------------------------- */
			std::vector<planner::Coord> get_path();

		private:
	
			// Source and destination coordinates. 
			planner::Coord source_;
			planner::Coord destination_;

			// Vector to store the path. 
			std::vector<planner::Coord> path_;

			// Function to bind with an algorithm of type planner::search_algorithm.
			std::function<bool( std::vector<std::vector<int> >&,
											    const planner::MapParameters&, 
											    const planner::Coord&, 
											    const planner::Coord&,
											    const planner::heuristic::TYPE&,
											    std::vector<planner::Coord>& )> search_algorithm_;	

			// Type of
			search_algorithm::TYPE algorithm_;

			// void compute_path(std::vector<std::vector<node> > &nodes, std::stack<Coord> &path);
	};
} // End of namespace planner

#endif