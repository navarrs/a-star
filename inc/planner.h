
#ifndef PLANNING
#define PLANNING

#include <opencv2/opencv.hpp>
#include <functional>
#include <algorithm>
#include <stack> 
#include <iostream>

// Threshold to create binary maps
#define THRESH_MIN 225
#define THRESH_MAX 255

// Colors
#define SBLACK cv::Scalar(0,    0,   0)
#define SRED   cv::Scalar(0,    0, 255)
#define VBLUE  cv::Vec3b(255,   0,   0)
#define VWHITE cv::Vec3b(255, 255, 255)

// Common parameters
#define FREE_CELL 225
#define FREE      0
#define BLOCKED   1

namespace planner {

	// Struct that stores map parameters
	struct MapParameters {

		int width_;
		int height_;
		int dilation_;
		int window_size_;
		int num_divs_w_;
		int num_divs_h_;
		int min_thresh_;
		int max_thresh_;

		void print() {
			std::cout << "[INFO] Map Configuration "   << std::endl
		          << "\tHeight: "      << height_      << std::endl
		          << "\tWidth: "       << width_       << std::endl
		          << "\tDilation: "    << dilation_    << std::endl
		          << "\tWindow size: " << window_size_ << std::endl
		          << "\tMin thresh: "  << min_thresh_  << std::endl
		          << "\tMax thresh: "  << max_thresh_  << std::endl;
		}
	};

	// Struct that represents Coordinates in a map.
	struct Coord {
		// Row, column
		int r, c;
		// Overloading operators 
		bool operator ==( const Coord &co ) const { 
			return r == co.r && c == co.c; 
		}
		Coord operator +( const Coord &co ) { 
			return {r + co.r, c + co.c}; 
		}
		friend std::ostream& operator <<( std::ostream &out, const Coord &co ) {
			out << "<" << co.r << "," << co.c << ">\n";
			return out;
		}
	};

	struct Node {
		Coord parent;
		unsigned int h;
	};

	// Heuristic 
	namespace heuristic {

		// Enum that encompasses supported heuristic functions.
		enum class TYPE {
			EUCLIDEAN = 0,
			MANHATTAN,
			OCTAGONAL,
			NOT_SUPPORTED
		};

		// Mapping function from enum to string.
		const std::map<TYPE, std::string> NAME{
		  {TYPE::EUCLIDEAN,     "Euclidean"     },
		  {TYPE::MANHATTAN,     "Manhattan"     },
		  {TYPE::OCTAGONAL,     "Octagonal"     },
		  {TYPE::NOT_SUPPORTED, "Not supported" },
		};

		// Methods for heuristic functions.
		class Function {
			public:
				/* ----------------------------------------------------------- *
			     * @name:   manhattan()
			     * @brief:  Computes the Manhattan distance between the two 
			     *          Coordinates.
			     * @param:  ( s ): Coordinate 1. 
			     *          ( d ): Coordinate 2.
			     * @return: Absolute Manhattan distance computed.  
			     * ---------------------------------------------------------- */
				static unsigned int manhattan(Coord s, Coord d);

				/* ----------------------------------------------------------- *
			     * @name:   euclidean()
			     * @brief:  Computes the Euclidean distance between the two 
			     *          Coordinates.
			     * @param:  ( s ): Coordinate 1. 
			     *          ( d ): Coordinate 2.
			     * @return: Absolute Euclidean distance computed.  
			     * ---------------------------------------------------------- */
				static unsigned int euclidean(Coord s, Coord d);

				/* ----------------------------------------------------------- *
			     * @name:   octagonal()
			     * @brief:  Computes the octagonal distance between the two 
			     *          Coordinates.
			     * @param:  ( s ): Coordinate 1. 
			     *          ( d ): Coordinate 2.
			     * @return: Absolute octagonal distance computed.  
			     * ---------------------------------------------------------- */
				static unsigned int octagonal(Coord s, Coord d);

			private:
				/* ----------------------------------------------------------- *
			     * @name:   get_delta()
			     * @brief:  Computes absolute difference between two Coordinates.
			     * @param:  ( s ): Coordinate 1. 
			     *          ( d ): Coordinate 2.
			     * @return: Absolute difference.  
			     * ---------------------------------------------------------- */
				static Coord get_delta(Coord s, Coord d);
		};
	} // End of namespace heuristic


	// Search algorithms 
	namespace search_algorithm {

		// Enum that encompasses supported search algorithms.
		enum class TYPE {
			ASTAR = 0,
			NOT_SUPPORTED
		};

		// Mapping function from enum to string.
		const std::map<TYPE, std::string> NAME{
			{TYPE::ASTAR,         "A star search" },
			{TYPE::NOT_SUPPORTED, "Not supported" },
		};

		// Methods for search algorithms.
		class Function {
			public:

				Function();
				~Function();
				/* ----------------------------------------------------------- *
			     * @name:   astar()
			     * @brief:  Astar-based search.
			     * @param:  ( grid ): Binary map representing the world map. 
			     *          ( map_param  ): Map parameters.
			     *          ( src ): Coordinate representing the start point.
			     *          ( dst ): Coordinate representing the destination.
			     *          ( heuristic ): Heuristic to use for planning.
			     *				  ( path ): Path computed from source to destination. 
			     * @return: False if: 
			     *            1) Either source or destination are invalid.
			     *            2) Either source or destination are blocked. 
			     *          Otherwise, true. 
			     * ---------------------------------------------------------- */
				static bool astar( std::vector<std::vector<int>> &grid,
								           const planner::MapParameters &map_param,
				                   const planner::Coord &src, 
				                   const planner::Coord &dst,
				                   const planner::heuristic::TYPE &heuristic,
				                   std::vector<planner::Coord> &path );
			private:

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

				/* ----------------------------------------------------------- *
			     * @name:   is_Coord_valid()
			     * @brief:  Checks if a Coordinate is withing the world's range.
			     * @param:  ( co  ): Coordinate to evaluate.
			     *          ( map_param ): Map parameters. 
			     * @return: False if the coordinate is not within the world's
			     *          range. Otherwise, true.
			     * ---------------------------------------------------------- */
				static bool is_coord_valid( const planner::Coord &co, 
					                          const planner::MapParameters &map_param );

				/* ----------------------------------------------------------- *
			     * @name:   is_coord_destination()
			     * @brief:  Checks if a coordinate is the same as the destination 
			     * @param:  ( co  ): Coordinate to evaluate.
			     *          ( dst ): Destination coordinate. 
			     * @return: True if the coordinate is the same as the destination
			     *          Otherwise, false.
			     * ---------------------------------------------------------- */
				static bool is_coord_destination( const planner::Coord &co, 
					                                const planner::Coord &dst );


				/* ----------------------------------------------------------- *
			     * @name:   is_coord_blocked()
			     * @brief:  Checks if a coordinate is blocked. 
			     * @param:  ( co   ): Coordinate to evaluate.
			     *          ( grid ): Binary grid that represents the world.
			     * @return: True if the coordinate is blocked. Otherwise, false.
			     * ---------------------------------------------------------- */
				static bool is_coord_blocked( const planner::Coord &co, 
					                   const std::vector<std::vector<int>> &grid );

				// Vector of movement directions. 
				std::vector<planner::Coord> directions_;
				int num_directions_;

				// Heuristic function 
				std::function<unsigned int( planner::Coord, 
					                          planner::Coord )> heuristic_func_;

		};
	} // End of namespace search_algorihthm
}

#endif