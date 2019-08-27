#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <unistd.h>

#include "planner.h"
#include "path_finder.h"
#include "map.h"


/*	@method: mouse_calback(...)
	@brief: Mouse callback to set locations of source and destination
			points on obstacle map. 
	@param: 
		event (int)  	event that triggered the callback
		x (int)      	x position of mouse when event was triggered
		y (int)      	y position of mouse when event was triggered
		flags (int)  	other flags that were triggered
		param (void *)	obstacle map
	@returns:
		void
*/
void mouse_callback(int event, int x, int y, int flags, void* param);

int main( int argc, char* argv[] ) 
{
	/**
	 * Parse arguments 
	 */
	boost::program_options::options_description description( "Usage" );
	description.add_options()
		( "help", "Program usage." )
		( "map-path",   boost::program_options::value<std::string>()->default_value( 
				"../maps/map1.png" ), "Path to input map." )
		( "map-config", boost::program_options::value<std::string>()->default_value(
				"../maps/map.yml" ), "Path to map configuration." )
		( "heuristic",  boost::program_options::value<std::string>()->default_value(
				"euclidean"), "Heuristic function used with the search algorithm." )
		( "search",    boost::program_options::value<std::string>()->default_value(
			  "astar"),     "Search algorithm for planning" );
	boost::program_options::variables_map options;
	boost::program_options::store( boost::program_options::command_line_parser( 
		     argc, argv ).options( description ).run(), options );

	try
	{
		boost::program_options::notify( options );
	}
	catch ( std::exception& e )
	{
		std::cerr << "[FATAL] Could not parse arguments. Error: " 
		          << e.what()
		          << std::endl;
		return EXIT_FAILURE;
	}

	std::string map_path {       options[ "map-path"   ].as<std::string>() };
	std::string map_config_file{ options[ "map-config" ].as<std::string>() };
	std::string heuristic_str{   options[ "heuristic"  ].as<std::string>() };
	std::string search_str{      options[ "search"     ].as<std::string>() };

	// Validate input path to map. 
	if ( map_path.empty() )
	{
		std::cerr << "[FATAL] Need to provide a path to the input map" << std::endl;
		return EXIT_FAILURE;
	}

	// Validate input path to configuration file. 
	if ( map_config_file.empty() )
	{
		std::cerr << "[FATAL] Need to provide a path to the configuration file"
		          << std::endl;
		return EXIT_FAILURE;	
	}

	// Get heuristic.
	boost::to_upper( heuristic_str );
	planner::heuristic::TYPE heuristic{ planner::heuristic::NAME2TYPE.find( 
																	          heuristic_str )->second };
	if ( planner::heuristic::TYPE::NOT_SUPPORTED == heuristic )
	{
		std::cerr << "[FATAL] Heuristic is not supported" << std::endl;
		return EXIT_FAILURE;
	}

	// Get planner.
	boost::to_upper( search_str );
	planner::search_algorithm::TYPE search{ planner::search_algorithm::NAME2TYPE.find( 
																	          search_str )->second };
	if ( planner::search_algorithm::TYPE::NOT_SUPPORTED == search )
	{
		std::cerr << "[FATAL] Search algorithm is not supported" << std::endl;
		return EXIT_FAILURE;
	}

	/**
	 * Creating Map
	 */ 
	std::cout <<  "[START] Creating world..." << std::endl;
	cv::Mat input_map = cv::imread( map_path );
	planner::Map map( map_config_file, input_map );
	if ( !map.create_obstacle_map() ) {
		std::cout << "[FATAL] Could not create map" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "[INFO] Displaying maps..." << std::endl;
	map.display();
	std::cout << "[DONE]" << std::endl;

	/**
	 * Creating Planner
	 */ 
	std::cout << "[START] Creating planner..." << std::endl;
	planner::PathFinder path_finder;

	std::cout << "[INFO] Setting Search Algorithm " << std::endl;
	if ( !path_finder.set_search_algorithm( 
		                 planner::search_algorithm::TYPE::ASTAR ) ) {
		std::cout << "[ERROR] Could not set search algorithm" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "[INFO] Setting source and destination" << std::endl;
	path_finder.set_source( { 2, 4 } );
	path_finder.set_destination( { 24,  32 } );
	std::cout << "[INFO] Displaying planner configuration " << std::endl;
	path_finder.print();
	std::cout << "[DONE]" << std::endl;

	/**
	 * Planning
	 */ 
	std::cout << "[START] Planning" << std::endl;
	std::vector<std::vector<int>> binmap = map.get();
	if( !path_finder.find_path( binmap, map.get_configuration(), heuristic ) )  
	{
		std::cout << "[ERROR] Planner could not find path" << std::endl;
		return EXIT_FAILURE;
	}
	std::vector<planner::Coord> path = path_finder.get_path();

	std::cout << "[INFO] Tracing path" << std::endl;
	map.trace_path( path );
	map.display();
	std::cout << "[DONE] " << std::endl;

	return EXIT_SUCCESS;
}

// // Mouse callback to set start and destination 
// void mouse_callback(int event, int x, int y, int flags, void* param) {
// 	Mat img = *((Mat *) param);
// 	Rect bounds(0, 0, img.cols, img.rows);

// 	int init_x = (x / IM_WINDOW);
// 	int init_y = (y / IM_WINDOW);
// 	// Set source with left button click 
// 	if (event == EVENT_LBUTTONDBLCLK) { 
// 		// Get patch that corresponds to location 
// 		Mat patch(IM_WINDOW, IM_WINDOW, CV_8UC3);
// 		Rect roi(init_x * IM_WINDOW, init_y * IM_WINDOW, IM_WINDOW, IM_WINDOW);
// 		patch = img(roi & bounds);
// 		cvtColor(patch, patch, CV_BGR2GRAY);

// 		// Get average of patch to determine if it is an obstacle or not
// 		if (average(patch) < THRESH_OBSTACLE) {
// 			cout << "Invalid location. " << endl;
// 		} else {
// 			cout << "Setting source at (" << x << ", " << y << ")" << endl;
// 			// If source has already been set, delete it on map
// 			if (src.second != INT_MAX) {
// 				cout << "Resetting source at (" << x << ", " << y << ")" << endl;
// 				circle(img, 
// 					Point((src.second + 0.5) * IM_WINDOW, (src.first + 0.5) * IM_WINDOW), 
// 					RADIUS, WHITE, 2*THICK);
// 			}
// 			// Set source
// 			src.second = init_x;
// 			src.first = init_y;
// 			circle(img, 
// 				Point((src.second + 0.5) * IM_WINDOW, (src.first + 0.5) * IM_WINDOW), 
// 				RADIUS, GREEN, 2*THICK);
// 			cout << "Scaled source at (" << src.second << ", " << src.first << ")" << endl;
// 		}
// 	// Set destination with right button click 
// 	} else if (event == EVENT_MBUTTONDBLCLK ) {
// 		// Get patch that corresponds to location 
// 		Mat patch(IM_WINDOW, IM_WINDOW, CV_8UC3);
// 		Rect roi(init_x * IM_WINDOW, init_y * IM_WINDOW, IM_WINDOW, IM_WINDOW);
// 		patch = img(roi & bounds);
// 		cvtColor(patch, patch, CV_BGR2GRAY);
		
// 		// Get average of patch to determine if it is an obstacle or not
// 		if (average(patch) < THRESH_OBSTACLE) {
// 			cout << "Invalid location. " << endl;
// 		} else {
// 			cout << "Setting destination at (" << x << ", " << y << ")" << endl;
// 			// If source has already been set, delete it on map
// 			if (dest.second != INT_MAX) {
// 				cout << "Resetting destination at (" << x << ", " << y << ")" << endl;
// 				circle(img, 
// 					Point((dest.second + 0.5) * IM_WINDOW, (dest.first + 0.5) * IM_WINDOW), 
// 					RADIUS, WHITE, 2*THICK);
// 			}
// 			// Set source
// 			dest.second  = init_x;
// 			dest.first = init_y;
// 			circle(img, 
// 				Point((dest.second + 0.5) * IM_WINDOW, (dest.first + 0.5) * IM_WINDOW), 
// 				RADIUS, RED, 2*THICK);
// 			cout << "Scaled destination at (" << dest.second << ", " << dest.first << ")" << endl;
// 		}
// 	}
// }