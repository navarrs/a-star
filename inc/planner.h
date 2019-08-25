
#ifndef PLANNING
#define PLANNING

#include <opencv2/opencv.hpp>
#include <functional>
#include <algorithm>
#include <stack> 
#include <iostream>

// Colors
#define SBLACK cv::Scalar(0,    0,   0)
#define SRED   cv::Scalar(0,    0, 255)
#define VBLUE  cv::Vec3b(255,   0,   0)
#define VWHITE cv::Vec3b(255, 255, 255)

// Common parameters
#define FREE_CELL 225
#define FREE      0
#define BLOCKED   1

namespace planner 
{
	// Struct that stores map parameters
	struct MapParameters 
	{
		// Map dimensions. 
		int width_;
		int height_;

		// How much will obstacles be dilated. 
		int dilation_;

		// Map grid divisions.
		int window_size_;
		int num_divs_w_;
		int num_divs_h_;

		// Thresholds to consider a map cell to be an obstacle or not. 
		int min_thresh_;
		int max_thresh_;

		/* ----------------------------------------------------------------------- *
		 * @name:   print
		 * @brief:  Prints the map configuration.
		 *
		 * @param:  
		 *
		 * @return: 
		 * ---------------------------------------------------------------------- */
		void print() 
		{
			std::cout << "[INFO] Map Configuration "   << std::endl
		            << "\tHeight: "      << height_      << std::endl
		            << "\tWidth: "       << width_       << std::endl
		            << "\tDilation: "    << dilation_    << std::endl
		            << "\tWindow size: " << window_size_ << std::endl
		            << "\tMin thresh: "  << min_thresh_  << std::endl
		            << "\tMax thresh: "  << max_thresh_  << std::endl;
		}
	}; // End of struct MapParameters

	// Struct that represents Coordinates in a map.
	struct Coord 
	{
		// Row, column
		int r, c;

		/* ----------------------------------------------------------------------- *
		 * @name:   ==
		 * @brief:  Overloads operator == to compare struct of type planner::Coord.
		 *
		 * @param:  coordinate: Coordinate to compare.
		 *
		 * @return: False if:
		 *            1) Coordinates are not equal. 
		 *          Otherwise, true. 
		 * ---------------------------------------------------------------------- */
		bool operator ==( const Coord &coordinate ) const 
		{ 
			return r == coordinate.r && c == coordinate.c; 
		}

		/* ----------------------------------------------------------------------- *
		 * @name:   +
		 * @brief:  Overloads operator + to add struct of type planner::Coord.
		 *
		 * @param:  coordinate: Coordinate to add.
		 *
		 * @return: Result of adding coordinates. 
		 * ---------------------------------------------------------------------- */
		Coord operator +( const Coord &coordinate ) 
		{ 
			return {r + coordinate.r, c + coordinate.c}; 
		}

		/* ----------------------------------------------------------------------- *
		 * @name:   <<
		 * @brief:  Overloads operator << to stream struct of type planner::Coord.
		 *
		 * @param:  out: Stream object.
		 *          coordinate: Coordinate to add.
		 *
		 * @return: Coordinate stream. 
		 * ---------------------------------------------------------------------- */
		friend std::ostream& operator <<( std::ostream &out, const Coord &coordinate ) 
		{
			out << "<" << coordinate.r << "," << coordinate.c << ">\n";
			return out;
		}
	}; // End of struct Coord. 

	struct Node {
		planner::Coord parent;
		unsigned int h;
	};

	// Heuristic 
	namespace heuristic 
	{

		// Enum that encompasses supported heuristic functions.
		enum class TYPE 
		{
			EUCLIDEAN = 0,
			MANHATTAN,
			OCTAGONAL,
			NOT_SUPPORTED
		};

		// Mapping function from enum to string.
		const std::map<TYPE, std::string> NAME
		{
		  {TYPE::EUCLIDEAN,     "Euclidean"     },
		  {TYPE::MANHATTAN,     "Manhattan"     },
		  {TYPE::OCTAGONAL,     "Octagonal"     },
		  {TYPE::NOT_SUPPORTED, "Not supported" },
		};

		// Methods for heuristic functions.
		class Function 
		{
			public:
				/* ------------------------------------------------------------------- *
				 * @name:   manhattan
				 * @brief:  Computes the Manhattan distance between two coordinates.
				 *
				 * @param:  coordinate1: Coordinate of type planner::Coord. 
				 * @param:  coordinate2: Coordinate of type planner::Coord,
				 *
				 * @return: Absolute value of the Manhattan distance. 
				 * ------------------------------------------------------------------ */
				static unsigned int manhattan( const planner::Coord& coordinate1,  
					                             const planner::Coord& coordinate2 );

				/* ------------------------------------------------------------------- *
				 * @name:   euclidean
				 * @brief:  Computes the euclidean distance between two coordinates.
				 *
				 * @param:  coordinate1: Coordinate of type planner::Coord. 
				 * @param:  coordinate2: Coordinate of type planner::Coord,
				 *
				 * @return: Absolute value of the euclidean distance. 
				 * ------------------------------------------------------------------ */
				static unsigned int euclidean( const planner::Coord& coordinate1,  
																			 const planner::Coord& coordinate2 );

				/* ------------------------------------------------------------------- *
				 * @name:   octagonal
				 * @brief:  Computes the octagonal distance between two coordinates.
				 *
				 * @param:  coordinate1: Coordinate of type planner::Coord. 
				 * @param:  coordinate2: Coordinate of type planner::Coord,
				 *
				 * @return: Absolute value of the octagonal distance. 
				 * ------------------------------------------------------------------ */
				static unsigned int octagonal( const planner::Coord& coordinate1, 
																		   const planner::Coord& coordinate2 );

			private:
				/* ------------------------------------------------------------------- *
				 * @name:   get_delta
				 * @brief:  Computes absolute difference between two Coordinates.
				 *
				 * @param:  coordinate1: Coordinate of type planner::Coord. 
				 * @param:  coordinate2: Coordinate of type planner::Coord,
				 *
				 * @return: Absolute difference between coordinates. 
				 * ------------------------------------------------------------------ */
				static planner::Coord get_delta( const planner::Coord &coordinate1,  
																			   const planner::Coord& coordinate2 );
		};
	} // End of namespace heuristic


	// Search algorithms 
	namespace search_algorithm 
	{
		// Enum that encompasses supported search algorithms.
		enum class TYPE 
		{
			ASTAR = 0,
			NOT_SUPPORTED
		};

		// Mapping function from enum to string.
		const std::map<TYPE, std::string> NAME
		{
			{TYPE::ASTAR,         "A star search" },
			{TYPE::NOT_SUPPORTED, "Not supported" },
		};

		// Methods for search algorithms.
		class Function 
		{
			public:

				/* ------------------------------------------------------------------- *
				 * @name:   Function
				 * @brief:  Constructor for the planner::search_algorithm Function class. 
				 *
				 * @param:  
				 *
				 * @return: 
				 * ------------------------------------------------------------------ */
				Function(){};

				/* ------------------------------------------------------------------- *
				 * @name:   Function
				 * @brief:  Constructor for the planner::search_algorithm Function class. 
				 *
				 * @param:  
				 *
				 * @return: 
				 * ------------------------------------------------------------------ */
				~Function(){};

				/* ------------------------------------------------------------------- *
				 * @name:   astar
				 * @brief:  Astar-based search algorithm.
				 *
				 * @param:  bin_map: Binary representation of the map for path finding. 
				 * @param:  map_param: Configuration parameters used to create the map.
				 * @param:  source: Struct of type planner::Coord that represents the 
				 *          starting point.
				 * @param:  destination: Struct of type planner::Coord that represents  
				 *          the destination point.  
				 * @param:  heuristic: Heuristic function used to optimize path finding.
				 * @param:  path: Vector where the way points of the found path will be 
				 *          stored. 
				 *
				 * @return: False if:
				 *						1) Source or destination coordinates are not valid. 
				 *						2) Source or destination coordinates are blocked. 
				 *						3) Source is already at destination.
				 *            4) No path was found. 
				 *					Otherwise, true.
				 * ------------------------------------------------------------------ */
				 static bool astar( std::vector<std::vector<int>> &bin_map,
								            const planner::MapParameters &map_param,
				                    const planner::Coord &source, 
				                    const planner::Coord &destination,
				                    const planner::heuristic::TYPE &heuristic,
				                    std::vector<planner::Coord> &path );
			private:

				// TODO: Make this one work
				/* ------------------------------------------------------------------- *
				 * @name:   set_heuristic
				 * @brief:  Sets type of heuristic to use to optimize path finding.
				 *
				 * @param:  heuristic: Heuristic function to use. Currently supports.
				 *            1) Euclidean distance. 
				 *						2) Manhattan distance. 
				 *            3) Octagonal distance. 
				 *
				 * @return: False if:
				 *						1) Heuristic function is not supported. 
				 *					Otherwise, true.
				 * ------------------------------------------------------------------ */
				 bool set_heuristic( const heuristic::TYPE& heuristic ); 

				/* ------------------------------------------------------------------- *
				 * @name:   is_coord_valid
				 * @brief:  Checks if a Coordinate is withing the world's range.
				 *
				 * @param:  coordinate: Coordinate to evaluate.
				 * @param:  map_param:  Configuration parameters of map. 
				 *
				 * @return: False if:
				 *						1) Coordinate is not within world's range. 
				 *					Otherwise, true.
				 * ------------------------------------------------------------------ */
				static bool is_coord_valid( const planner::Coord &coordinate, 
					                          const planner::MapParameters &map_param );

				/* ------------------------------------------------------------------- *
				 * @name:   is_coord_destination
				 * @brief:  Checks if a coordinate is the same as the destination.
				 *
				 * @param:  coordinate:  Coordinate to evaluate.
				 * @param:  destination: Destination coordinate. 
				 *
				 * @return: True if:
				 *						1) Coordinate is the destination. 
				 *					Otherwise, false.
				 * ------------------------------------------------------------------ */
				static bool is_coord_destination( const planner::Coord &coordinate, 
					                                const planner::Coord &destination );

				/* ------------------------------------------------------------------- *
				 * @name:   is_coord_blocked
				 * @brief:  Checks if a coordinate is blocked.
				 *
				 * @param:  coordinate: Coordinate to evaluate.
				 * @param:  bin_map: Binary representation of the map.
				 *
				 * @return: True if:
				 *						1) Coordinate is blocked. 
				 *					Otherwise, false.
				 * ------------------------------------------------------------------ */
				static bool is_coord_blocked( const planner::Coord &coordinate, 
					                            const std::vector<std::vector<int>> &bin_map );

				/* ------------------------------------------------------------------- *
				 * @name:   get_path
				 * @brief:  Gets the path found by the search algorithm.
				 *
				 * @param:  path:  Vector of coordinates that represent the path. 
				 * @param:  nodes: Vector of nodes to generate the path. 
				 * @param:  destination: Destination coordinate. 
				 *
				 * @return: 
				 * ------------------------------------------------------------------ */
				static void get_path( std::vector<planner::Coord> &path, 
											        const std::vector<std::vector<planner::Node>> &nodes, 
			                        const planner::Coord &destination );

				// Vector of movement directions. 
				std::vector<planner::Coord> directions_;
				int num_directions_;

				// Set heuristic function 
				std::function<unsigned int( 
					const planner::Coord&, const planner::Coord& )> heuristic_func_;

		};
	} // End of namespace search_algorihthm
}

#endif