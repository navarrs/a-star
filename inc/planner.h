/* --------------------------------------------------------------------------- *
 * @file:   planner.h
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the types used in along the planner namespace.
 *
 * -------------------------------------------------------------------------- */
#ifndef PLANNING
#define PLANNING

#include <opencv2/opencv.hpp>
#include <functional>
#include <algorithm>
#include <stack> 
#include <iostream>

namespace planner  {

const unsigned int FREE_CELL = 225;
const unsigned int FREE      = 0;
const unsigned int BLOCKED   = 1;


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
	 * @name:   operator<<
	 * @brief:  Prints the map configuration.
	 *
	 * @param:  out(out): Stream object.
	 *          coordinate(in): Map parameters to print.
	 *
	 * @return: 
	 * */
	friend std::ostream& operator<<( std::ostream &out, const MapParameters& mp )
	{
		out << "[INFO] Map Configuration"      
	      << "\n\tHeight: "      << mp.height_     
        << "\n\tWidth: "       << mp.width_      
        << "\n\tDilation: "    << mp.dilation_   
        << "\n\tWindow size: " << mp.window_size_
        << "\n\tMin thresh: "  << mp.min_thresh_ 
        << "\n\tMax thresh: "  << mp.max_thresh_ ;
    return out;
	}
}; // End of struct MapParameters

// Struct that represents Coordinates in a map.
struct Coord 
{
	// Row, column
	int r, c;

	/* ----------------------------------------------------------------------- *
	 * @name:   operator==
	 * @brief:  Overloads operator == to compare struct of type planner::Coord.
	 *
	 * @param:  coordinate(in): Coordinate to compare.
	 *
	 * @return: False if:
	 *            1) Coordinates are not equal. 
	 *          Otherwise, true. 
	 * */
	bool operator==( const Coord &coordinate ) const 
	{ 
		return r == coordinate.r && c == coordinate.c; 
	}

	/* ----------------------------------------------------------------------- *
	 * @name:   +
	 * @brief:  Overloads operator + to add struct of type planner::Coord.
	 *
	 * @param:  coordinate(in): Coordinate to add.
	 *
	 * @return: Result of adding coordinates. 
	 * */
	Coord operator+( const Coord &coordinate ) 
	{ 
		return { r + coordinate.r, c + coordinate.c }; 
	}

	/* ----------------------------------------------------------------------- *
	 * @name:   operator<<
	 * @brief:  Overloads operator << to stream struct of type planner::Coord.
	 *
	 * @param:  out(out): Stream object.
	 *          coordinate(in): Coordinate to print.
	 *
	 * @return: Coordinate stream. 
	 * */
	friend std::ostream& operator <<( std::ostream &out, const Coord &coordinate ) 
	{
		out << "<" << coordinate.r << "," << coordinate.c << ">\n";
		return out;
	}
}; // End of struct Coord. 

struct Node 
{
	planner::Coord parent;
	unsigned int h;
	unsigned int g;

	/* ----------------------------------------------------------------------- *
	 * @name:   f
	 * @brief:  Computes cost: f = h + g
	 *
	 * @param: 
	 *
	 * @return: Cost 
	 * */
	unsigned int f() const
	{
		return h + g;
	}

	/* ----------------------------------------------------------------------- *
	 * @name:   operator<<
	 * @brief:  Overloads operator << to stream struct of type planner::Coord.
	 *
	 * @param:  out(out): Stream object.
	 *          coordinate(in): Coordinate to print.
	 *
	 * @return: Coordinate stream. 
	 * */
	friend std::ostream& operator <<( std::ostream& out, const Node& node ) 
	{
		out << "Node:"
		    << "\n\tParent: " << node.parent 
		    << "\n\tF: "      << node.f()
		    << "\n\tH: "			<< node.h
		    << "\n\tG: "			<< node.g;
		return out;
	}

}; // End of struct Node. 

// Heuristic 
namespace heuristic {

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
  { TYPE::EUCLIDEAN,     "EUCLIDEAN"     },
  { TYPE::MANHATTAN,     "MANHATTAN"     },
  { TYPE::OCTAGONAL,     "OCTAGONAL"     },
  { TYPE::NOT_SUPPORTED, "NOT_SUPPORTED" },
};

// Mapping function from string to enum.
const std::map<std::string, TYPE> NAME2TYPE
{
	{ "EUCLIDEAN",     TYPE::EUCLIDEAN     },
	{ "MANHATTAN",     TYPE::MANHATTAN     },
	{ "OCTAGONAL",     TYPE::OCTAGONAL     },
	{ "NOT_SUPPORTED", TYPE::NOT_SUPPORTED },
};

// Methods for heuristic functions.
class Function 
{
	public:
		/* ---------------------------------------------------------------------- *
		 * @name:   manhattan
		 * @brief:  Computes the Manhattan distance between two coordinates.
		 *
		 * @param:  coordinate1(in): Coordinate of type planner::Coord. 
		 * @param:  coordinate2(in): Coordinate of type planner::Coord,
		 *
		 * @return: Absolute value of the Manhattan distance. 
		 * */
		static unsigned int manhattan( const planner::Coord& coordinate1,  
			                             const planner::Coord& coordinate2 );

		/* ---------------------------------------------------------------------- *
		 * @name:   euclidean
		 * @brief:  Computes the euclidean distance between two coordinates.
		 *
		 * @param:  coordinate1(in): Coordinate of type planner::Coord. 
		 * @param:  coordinate2(in): Coordinate of type planner::Coord,
		 *
		 * @return: Absolute value of the euclidean distance. 
		 * */
		static unsigned int euclidean( const planner::Coord& coordinate1,  
																	 const planner::Coord& coordinate2 );

		/* ---------------------------------------------------------------------- *
		 * @name:   octagonal
		 * @brief:  Computes the octagonal distance between two coordinates.
		 *
		 * @param:  coordinate1(in): Coordinate of type planner::Coord. 
		 * @param:  coordinate2(in): Coordinate of type planner::Coord,
		 *
		 * @return: Absolute value of the octagonal distance. 
		 * */
		static unsigned int octagonal( const planner::Coord& coordinate1, 
																   const planner::Coord& coordinate2 );

	private:
		/* ---------------------------------------------------------------------- *
		 * @name:   get_delta
		 * @brief:  Computes absolute difference between two Coordinates.
		 *
		 * @param:  coordinate1(in): Coordinate of type planner::Coord. 
		 * @param:  coordinate2(in): Coordinate of type planner::Coord,
		 *
		 * @return: Absolute difference between coordinates. 
		 * */
		static planner::Coord get_delta( const planner::Coord &coordinate1,  
																	   const planner::Coord& coordinate2 );
};
} // End of namespace heuristic.


// Search algorithms 
namespace search_algorithm {
// Enum that encompasses supported search algorithms.
enum class TYPE 
{
	ASTAR = 0,
	
	NOT_SUPPORTED
};

// Mapping function from enum to string.
const std::map<TYPE, std::string> NAME
{
	{ TYPE::ASTAR,         "ASTAR"         },
	{ TYPE::NOT_SUPPORTED, "NOT_SUPPORTED" },
};

// Mapping function from string to enum.
const std::map<std::string, TYPE> NAME2TYPE
{
	{ "ASTAR",         TYPE::ASTAR         },
	{ "NOT_SUPPORTED", TYPE::NOT_SUPPORTED },
};

} // End of namespace search_algorihthm.
} // End of namespace planner.

#endif