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

int main( int argc, char* argv[] ) {

	std::string map_path { "../maps/map1.png" };
	std::string map_config_file{ "../maps/map.yml" };
	bool visualize { false };

	// Parse arguments 
	int c; 
	while( ( c = getopt( argc, argv, "m:c:hv" ) ) != -1 ) {
		switch( c ) {
			case 'm': if( optarg ) map_path = optarg; break;
			case 'c': if( optarg ) map_config_file = optarg; break;
			case 'v': if( argv[ optind] ) visualize = atoi(argv[ optind]); break;
			case 'h':
			case '?':
				std::cout << "[USAGE]"
				          << "\t -h \tUsage help.\n"
				          << "\t -p \tFull path to the map to test. Default: "
				          << map_path << "\n"
				          << "\t -c \tFull path to map configuration. Default:"
				          << map_config_file << "\n"
				          << "\t -v \tVisualize process. Default: " << visualize
				          << std::endl;
				exit( EXIT_SUCCESS );
		} 
	}

	//-------------------------- Creating Map
	std::cout <<  "[START] Creating world..." << std::endl;
	cv::Mat input_map = cv::imread( map_path );
	planner::Map map( map_config_file, input_map );
	if ( !map.create_obstacle_map() ) {
		std::cout << "[ERROR] Could not create map" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "[INFO] Displaying maps..." << std::endl;
	map.display();
	std::cout << "[DONE]" << std::endl;

	//-------------------------- Creating planner
	std::cout << "[START] Creating planner..." << std::endl;
	planner::PathFinder path_finder;

	std::cout << "[INFO] Setting Search Algorithm " << std::endl;
	if ( !path_finder.set_search_algorithm( 
		                 planner::search_algorithm::TYPE::ASTAR ) ) {
		std::cout << "[ERROR] Could not set search algorithm" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "[INFO] Setting source and destination" << std::endl;
	path_finder.set_source( { 2, 2 } );
	path_finder.set_destination( { 2,  2 } );
	std::cout << "[INFO] Displaying planner configuration " << std::endl;
	path_finder.print();
	std::cout << "[DONE]" << std::endl;


	std::cout << "[INFO] Finding path" << std::endl;
	std::vector<std::vector<int>> binmap = map.get();
	path_finder.find_path( binmap,
												 map.get_configuration(),
												 planner::heuristic::TYPE::EUCLIDEAN );

	// std::stack<planner::Coord> path
	// astar.a_star_search(map.get_world(), path);

	// if (path.empty()) 
	// 	cout << "No path was found" << endl;
	// else {
	// 	map.trace_path(path);
	// 	map.print_world();
	// 	Mat final_map = map.get_input_map();
	// 	namedWindow("Final Map");
	// 	moveWindow("Final Map", 300, 350);
	// 	imshow("Final Map", final_map);
	// 	waitKey(0); //press q
	// }
	// cout << "[DONE] " << endl;

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