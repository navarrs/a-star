/**
 * @file:   path_finder.h
 * @author: Ingrid Navarro
 * @brief:  This file contains the definitions for the class PathGenerator. This 
 *          class used for path finding using the Astar algorithm. 
 */

#ifndef PATH_GENERATOR
#define PATH_GENERATOR

#include "planner.h"

namespace planner {

	class PathFinder {
		public:
			/* --------------------------------------------------------------- *
			 * @name:   PathGenerator()
			 * @brief:  Constructor for the class PathGenerator.
			 * @param:  None.
			 * @return: Creates an instance of a path generator with default 
			 *          parameters. 
		     * -------------------------------------------------------------- */
			PathFinder();

			/* --------------------------------------------------------------- *
			 * @name:   ~PathGenerator()
			 * @brief:  Destructor for the class PathGenerator.
			 * @param:  None.
		     * -------------------------------------------------------------- */
			~PathFinder();

			/* --------------------------------------------------------------- *
			 * @name:   set_search_algorithm( ... )
			 * @brief:  Sets the search algorithm to perform path finding. 
			 *          Current supported algorithms:
			 *           	1) Astar. 
			 * @param:  ( s ): Represents the type of search algorithm. 
			 * @return: False if search algorithm is invalid, otherwise true.
		     * -------------------------------------------------------------- */
			bool set_search_algorithm( const planner::search_algorithm::TYPE &s );

			/* --------------------------------------------------------------- *
			 * @name:   set_source( ... )
			 * @brief:  Sets starting Coordinate. 
			 * @param:  ( source ): Starting Coordinate. 
		     * -------------------------------------------------------------- */
			void set_source( const planner::Coord &source );

			/* --------------------------------------------------------------- *
			 * @name:   get_source( ... )
			 * @brief:  Gets starting Coordinate. 
			 * @return: ( Coord ): Source Coordinate. 
		     * -------------------------------------------------------------- */
			Coord get_source( );

			/* --------------------------------------------------------------- *
			 * @name:   set_destination( ... )
			 * @brief:  Sets destination Coordinate. 
			 * @param:  ( destination ): Destination Coordinate. 
		     * -------------------------------------------------------------- */
			void set_destination( const planner::Coord &destination );

			/* --------------------------------------------------------------- *
			 * @name:   get_destination( ... )
			 * @brief:  Gets destination Coordinate. 
			 * @return: ( Coord ): Destination Coordinate. 
		     * -------------------------------------------------------------- */
			Coord get_destination( );

			/* --------------------------------------------------------------- *
			 * @name:   print( ... )
			 * @brief:  Prints planner configuration
		   * -------------------------------------------------------------- */
			void print();

			/* --------------------------------------------------------------- *
			 * @name:   find_path( ... )
			 * @brief:  Calls search algorithm method to find path. 
			 * @param:  ( grid ): Represents the binary map.
			 *          ( map_params ): Map configuration.
			 *				  ( heuristic ): Heuristic method to use for planning.
			 * @returns: False if path is empty, true otherwise. 
		   * -------------------------------------------------------------- */
			bool find_path( std::vector<std::vector<int>>& grid,
											const planner::MapParameters& map_params, 
											const planner::heuristic::TYPE& heuristic );

		private:
	
			// Source and destination Coordinates. 
			planner::Coord source_;
			planner::Coord destination_;

			// Path 
			std::vector<planner::Coord> path_;

			std::function<bool( std::vector<std::vector<int> >&,
											    const planner::MapParameters&, 
											    const planner::Coord&, 
											    const planner::Coord&,
											    const planner::heuristic::TYPE&,
											    std::vector<planner::Coord>& )> search_algorithm_;	

			search_algorithm::TYPE algorithm_;

			// void compute_path(std::vector<std::vector<node> > &nodes, std::stack<Coord> &path);
	};
} // End of namespace planner

#endif