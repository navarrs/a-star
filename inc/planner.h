
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
#define FREE_BLOCK  225

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
		bool operator ==( const Coord &co ) { 
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

	using node_list = std::set<Node*>;

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
				/* ----------------------------------------------------------- *
			     * @name:   astar()
			     * @brief:  Astar-based search.
			     * @param:  ( grid ): Binary map representing the world map. 
			     *          ( map_param  ): Map parameters.
			     *          ( src ): Coordinate representing the start point.
			     *          ( dst ): Coordinate representing the destination.
			     * @return: False if: 
			     *            1) Either source or destination are invalid.
			     *            2) Either source or destination are blocked. 
			     *          Otherwise, true. 
			     * ---------------------------------------------------------- */
				static bool astar( std::vector<std::vector<int>> &grid,
								   const MapParameters &map_param,
				                   const Coord &src, const Coord &dst  );
			private:

				/* ----------------------------------------------------------- *
			     * @name:   is_Coord_valid()
			     * @brief:  Checks if a Coordinate is withing the world's range.
			     * @param:  ( co  ): Coordinate to evaluate.
			     *          ( map_param ): Map parameters. 
			     * @return: False if the Coordinate is not within the world's
			     *          range. Otherwise, true.
			     * ---------------------------------------------------------- */
				static bool is_coord_valid( const Coord &co, 
					                        const MapParameters &map_param );


				// bool is_Coord_destination( Coord s, Coord d );
				// bool is_Coord_blocked( const std::vector<std::vector<int> > &graph, 
				// 	                   Coord co );
		};
	} // End of namespace search_algorihthm
}

#endif