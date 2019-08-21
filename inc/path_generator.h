/**
 * @file:   path_generator.h
 * @author: Ingrid Navarro
 * @brief:  This file contains the definitions for the class PathGenerator. This 
 *          class used for path finding using the Astar algorithm. 
 */

#ifndef PATH_GENERATOR
#define PATH_GENERATOR

#include "planner.h"

namespace planner {

	class PathGenerator {
		public:
			/* --------------------------------------------------------------- *
			 * @name:   PathGenerator()
			 * @brief:  Constructor for the class PathGenerator.
			 * @param:  None.
			 * @return: Creates an instance of a path generator with default 
			 *          parameters. 
		     * -------------------------------------------------------------- */
			PathGenerator();

			/* --------------------------------------------------------------- *
			 * @name:   ~PathGenerator()
			 * @brief:  Destructor for the class PathGenerator.
			 * @param:  None.
		     * -------------------------------------------------------------- */
			~PathGenerator();

			/* --------------------------------------------------------------- *
			 * @name:   set_heuristic( ... )
			 * @brief:  Sets type of heuristic to use to perform path finding. 
			 *          Current supported heuristics:
			 *           	1) Euclidean distance. 
			 *              2) Manhattan distance. 
			 *              3) Octagonal distance. 
			 * @param:  ( h ): Represents the type of heuristic function. 
			 * @return: False if heuristic function is invalid, otherwise true.
		     * -------------------------------------------------------------- */
			bool set_heuristic( const heuristic::TYPE &h );

			/* --------------------------------------------------------------- *
			 * @name:   set_search_algorithm( ... )
			 * @brief:  Sets the search algorithm to perform path finding. 
			 *          Current supported algorithms:
			 *           	1) Astar. 
			 * @param:  ( s ): Represents the type of search algorithm. 
			 * @return: False if search algorithm is invalid, otherwise true.
		     * -------------------------------------------------------------- */
			bool set_search_algorithm( const search_algorithm::TYPE &s );

			/* --------------------------------------------------------------- *
			 * @name:   set_source( ... )
			 * @brief:  Sets starting Coordinate. 
			 * @param:  ( source ): Starting Coordinate. 
		     * -------------------------------------------------------------- */
			void set_source( const Coord &source );

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
			void set_destination( const Coord &destination );

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

		private:
			// Vector of movement directions. 
			std::vector<Coord> directions_;
			int num_directions_;

			// Source and destination Coordinates. 
			Coord source_;
			Coord destination_;

			// Heuristic function-related
			std::function<unsigned int( Coord, Coord )> heuristic_func_;
			heuristic::TYPE heuristic_;

			std::function<bool( std::vector<std::vector<int> >&,
							    const MapParameters&, 
							    const Coord&, const Coord& )> search_algorithm_;		

			search_algorithm::TYPE algorithm_;

			// void compute_path(std::vector<std::vector<node> > &nodes, std::stack<Coord> &path);
	};
} // End of namespace planner

#endif