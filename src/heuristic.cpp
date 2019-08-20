#include "planner.h"

namespace planner {

	namespace heuristic {

		unsigned int Function::manhattan( planner::Coord s, planner::Coord d ) {
			 auto delta = std::move( get_delta( s, d ) );
			 return ( unsigned int ) ( delta.r + delta.c );
		}

		unsigned int Function::euclidean(planner::Coord s, planner::Coord d ) {
			 auto delta = std::move( get_delta( s, d ) );
			 return ( unsigned int ) ( sqrt( pow( delta.r, 2) + pow( delta.c, 2 ) ) );
		}

		unsigned int Function::octagonal(planner::Coord s, planner::Coord d) {
			auto delta = std::move( get_delta( s, d ) );
			return ( unsigned int) ( ( delta.r + delta.c ) - std::min( delta.r, delta.c ) );
		}

		Coord Function::get_delta(planner::Coord s, planner::Coord d) {
			return { abs( s.r - d.r ), abs( s.c - d.c ) };
		}

	} // End of namespace heuristic 
} // End of namespace planner