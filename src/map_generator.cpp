/**
 * @file:   map.cpp
 * @author: Ingrid Navarro
 * @brief:  This file contains the implementations for the class Map. This class 
 *          is used to create the map where the path planning will be performed. 
 *          It converts a given image into an obstacle grid. 
 */

#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

#include "planner.h"
#include "map_generator.h"

namespace planner {

	// Default constructor
	Map::Map() {
		map_.width_        = 640;
		map_.height_       = 480;
		map_.dilation_     = 2;
		map_.window_size_  = 10;
		map_.num_divs_w_   = ( int ) ( map_.width_  / map_.window_size_ );
		map_.num_divs_h_   = ( int ) ( map_.height_ / map_.window_size_ );
		map_.min_thresh_   = 200;
		map_.max_thresh_   = 255;

		std::vector<int> w_width( map_.num_divs_w_, 0 );
		std::vector<std::vector<int> > temp_bin_map_( map_.num_divs_h_, w_width );
		bin_map_ = temp_bin_map_;

		input_map_    = cv::Mat::zeros( cv::Size( map_.width_, 
			                                      map_.height_ ), 
		                                          CV_8UC3 );
		obstacle_map_ = cv::Mat::zeros( cv::Size( map_.width_, map_.height_ ), 
		                                          CV_8UC3 );

		print_config();

	}

	// Constructor with parameters
	Map::Map( const std::string &map_configuration_file, cv::Mat& map ) {

		// Get map parameters from YAML file 
		YAML::Node map_config = YAML::LoadFile( map_configuration_file );

		try {
			map_.width_        = map_config[   "map_width"].as<int>();
			map_.height_       = map_config[  "map_height"].as<int>();
			map_.dilation_     = map_config["map_dilation"].as<int>();
			map_.window_size_  = map_config[ "window_size"].as<int>();
			map_.min_thresh_   = map_config[  "min_thresh"].as<int>();
			map_.max_thresh_   = map_config[  "max_thresh"].as<int>();
		} catch( std::exception &e ) {
			std::cout << "[ERROR] Unable to parse map configuration: "
			          << e.what();
			exit( EXIT_FAILURE );
		}

		print_config();

		if ( map_.width_       < 0 || map_.height_   < 0 || 
			 map_.window_size_ < 0 || map_.dilation_ < 0  )  {
			std::cout << "[ERROR] Invalid input parameters " << std::endl;
			exit( EXIT_FAILURE );
		}

		map_.num_divs_w_ = ( int ) ( map_.width_  / map_.window_size_ );
		map_.num_divs_h_ = ( int ) ( map_.height_ / map_.window_size_ );

		std::vector<int> w_width(map_.num_divs_w_, 0);
		std::vector<std::vector<int> > temp_bin_map_(map_.num_divs_h_, w_width);
		bin_map_ = temp_bin_map_;

		if( !map.data ) {
			std::cout << "[ERROR] No image data";
			exit( EXIT_FAILURE );
		}
		// Resize input map
		cv::resize( map, input_map_, cv::Size( map_.width_, map_.height_ ), CV_8UC3 );
		obstacle_map_ = cv::Mat::zeros( input_map_.size(), CV_8UC3 );
	}

	void Map::print_config() {
		map_.print();
	}

	bool Map::set( std::string map_path ) {
		input_map_ = cv::imread( map_path, CV_LOAD_IMAGE_COLOR );
		if ( !input_map_.data ) {
			std::cout << "[ERROR] No image data. " << std::endl;
			return EXIT_FAILURE;
		}
		cv::resize( input_map_, input_map_, cv::Size( map_.width_, map_.height_ ) );
		return EXIT_SUCCESS;
	}

	MapParameters Map::get() {
		return map_;
	}

	bool Map::create() {

		if ( !input_map_.data ) {
			std::cout << "[ERROR] empty matrix" << std::endl;
			return EXIT_FAILURE;
		}

		// Convert input map to grayscale
		cv::Mat gray_map( input_map_.size(), CV_8UC1 );
		cv::cvtColor( input_map_, gray_map, CV_BGR2GRAY );

		// Binarize grayscale map 
		cv::Mat binary_map( gray_map.size(), CV_8UC1 );
		cv::threshold( gray_map, binary_map, map_.min_thresh_, 
			           map_.max_thresh_, CV_THRESH_BINARY_INV );

		// Dilate binarized map
		cv::Mat dilated_map( binary_map.size(), CV_8UC1 ); 
	    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, 
	    	                                     cv::Size( 2*map_.dilation_+1, 
	    	                                     	       2*map_.dilation_+1), 
	    	                                     cv::Point(map_.dilation_, 
	    	                                     	       map_.dilation_) );
		cv::dilate( binary_map, dilated_map, element, cv::Point(-1, -1), 3 );
		
		// Create obstacle map
		create_bgr_obstacle_map( dilated_map );

		// Generate bin_map_ 
		if ( EXIT_FAILURE == generate_bin_map() ) {
			std::cout << "[ERROR] Could not create binary map" << std::endl;
			return EXIT_FAILURE;
		}
		draw_grid();
		return EXIT_SUCCESS;
	}

	void Map::create_bgr_obstacle_map(const cv::Mat& dilated_map ) {
		cv::Mat temp_obstacle_map( input_map_.size(), CV_8UC3 );
		for ( unsigned int r = 0; r < dilated_map.rows; r++ ) {
			const uchar* R = dilated_map.ptr<uchar>(r);
			for ( unsigned int c = 0; c < dilated_map.cols; c++ ){
				uchar color = R[c];
				temp_obstacle_map.at<cv::Vec3b>(
					cv::Point(c, r) ) = color == map_.max_thresh_ ? VBLUE : VWHITE;
			}
		}
		obstacle_map_ = temp_obstacle_map;
	}

	int Map::generate_bin_map() {

		cv::Rect bounds(0, 0, obstacle_map_.cols, obstacle_map_.rows);
		cv::Mat map_crop = cv::Mat( map_.window_size_, map_.window_size_, 
			                        CV_8UC3, cv::Scalar( 0, 0, 0 ) );

		for ( unsigned int x = 0; x < obstacle_map_.cols; x += map_.window_size_) {
			for ( unsigned int y = 0; y < obstacle_map_.rows; y += map_.window_size_) {
				cv::Rect roi(x, y, map_.window_size_, map_.window_size_);
				map_crop = obstacle_map_(roi & bounds);
				int avg = roi_average( map_crop );
				if ( avg < 0 ) {
					std::cout << "[ERROR] invalid values on map" << std::endl;
					return EXIT_FAILURE;
				}
				bin_map_[y / map_.window_size_][x / map_.window_size_ ] = (avg < FREE_BLOCK ? 0 : 1);
			}
		}
		return EXIT_SUCCESS;
	}

	void Map::display( ) {
		cv::imshow( "Input Map", input_map_ );
		cv::imshow( "Obstacle Map", obstacle_map_);
		print_bin();
		cv::waitKey( 0 );
	}

	void Map::draw_grid() {
		unsigned int i = 0;
		// Draw vertical lines
		for (i = 0; i < obstacle_map_.cols; i += map_.window_size_) {
			cv::line( obstacle_map_, cv::Point(i, 0), cv::Point(i, obstacle_map_.rows-1), 
				      SBLACK, 1 );
		}
		// Draw horizontal lines
		for (i = 0; i < obstacle_map_.rows; i += map_.window_size_) {
		 	cv::line( obstacle_map_, cv::Point(0, i), cv::Point(obstacle_map_.cols-1, i), 
		 		      SBLACK, 1 );
		}
	}

	int Map::roi_average(cv::Mat roi) {
		int num_elements = roi.rows * roi.cols;
		int sum = 0;
		for ( unsigned int i = 0; i < roi.rows; i++ ) {
			for ( unsigned int j = 0; j < roi.cols; j++ ) {
				sum += (int)roi.at<uchar>(i, j);
			}
		}
		return (sum == 0 ? 0 : sum / num_elements);
	}

	void Map::print_bin() {
		for ( unsigned int i = 0; i < bin_map_.size(); i++ ) {
			for ( unsigned int j = 0; j < bin_map_[i].size(); j++ ) {
				std::cout << bin_map_[i][j] << " ";
			}
			std::cout << std::endl;
		}
	}

	void Map::trace(std::stack<Coord> path) {
		// Coord prev = {INT_MAX, INT_MAX};
		// while(!path.empty()) {
		// 	Coord p = path.top();
		// 	if (!(prev.c == INT_MAX)) {
		// 		//C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		// 		cv::circle(input_map, cv::Point(p.c * IM_WINDOW, p.r * IM_WINDOW), RADIUS, SRED, POINT_THICK);
		// 	}
		// 	path.pop();
		// 	std::cout << "->("<< p.r <<","<<p.c <<")";
		// 	bin_map_[p.r][p.c] = 7;
		// 	prev = p;
		// }
		// std::cout << std::endl;
	}
}
