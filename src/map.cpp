/* --------------------------------------------------------------------------- *
 * @file:   map.cpp
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the implementations for the class Map. Such class 
 *          is used to create the a binary map that represents an occupancy map 
 *          from an image. 
 *
 * -------------------------------------------------------------------------- */

#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

#include "planner.h"
#include "map.h"

namespace planner 
{
	// Default constructor
	Map::Map() 
	{
		map_params_.width_        = 640;
		map_params_.height_       = 480;
		map_params_.dilation_     = 2;
		map_params_.window_size_  = 10;
		map_params_.num_divs_w_   = ( int ) ( map_params_.width_  / map_params_.window_size_ );
		map_params_.num_divs_h_   = ( int ) ( map_params_.height_ / map_params_.window_size_ );
		map_params_.min_thresh_   = 200;
		map_params_.max_thresh_   = 255;

		std::vector<int> w_width( map_params_.num_divs_w_, 0 );
		std::vector<std::vector<int> > temp_bin_map_( map_params_.num_divs_h_, 
																									w_width );
		bin_map_ = temp_bin_map_;

		input_map_ = cv::Mat::zeros( 
								cv::Size( map_params_.width_, map_params_.height_ ), CV_8UC3 );

		obstacle_map_ = cv::Mat::zeros( 
			          cv::Size( map_params_.width_, map_params_.height_ ), CV_8UC3 );

		print_config();

	}

	// Constructor with parameters
	Map::Map( const std::string &map_configuration_file, cv::Mat& map ) 
	{

		// Get map parameters from YAML file 
		// YAML::Node map_config = YAML::LoadFile( map_configuration_file );

		// try 
		// {
		// 	map_params_.width_        = map_config[   "map_width"].as<int>();
		// 	map_params_.height_       = map_config[  "map_height"].as<int>();
		// 	map_params_.dilation_     = map_config["map_dilation"].as<int>();
		// 	map_params_.window_size_  = map_config[ "window_size"].as<int>();
		// 	map_params_.min_thresh_   = map_config[  "min_thresh"].as<int>();
		// 	map_params_.max_thresh_   = map_config[  "max_thresh"].as<int>();
		// } 
		// catch( std::exception &e ) 
		// {
		// 	std::cout << "[ERROR] Unable to parse map configuration: " << e.what();
		// 	exit( EXIT_FAILURE );
		// }

		print_config();

		// Assert that the parameters have valid values.
		if ( map_params_.width_       < 0 || map_params_.height_   < 0 || 
			   map_params_.window_size_ < 0 || map_params_.dilation_ < 0  )  
		{
			std::cout << "[ERROR] Invalid input parameters " << std::endl;
			exit( EXIT_FAILURE );
		}

		map_params_.num_divs_w_ = ( int ) ( map_params_.width_  / map_params_.window_size_ );
		map_params_.num_divs_h_ = ( int ) ( map_params_.height_ / map_params_.window_size_ );

		std::vector<int> w_width(map_params_.num_divs_w_, 0);
		std::vector<std::vector<int> > temp_bin_map_( map_params_.num_divs_h_, 
			                                            w_width);
		bin_map_ = temp_bin_map_;

		// Verify map has data. 
		if( !map.data ) 
		{
			std::cout << "[ERROR] No image data";
			exit( EXIT_FAILURE );
		}
	
		// Resize input map to size specified parameters.
		cv::resize( map, input_map_, 
								cv::Size( map_params_.width_, map_params_.height_ ), CV_8UC3 );

		obstacle_map_ = cv::Mat::zeros( input_map_.size(), CV_8UC3 );
	}

	void Map::print_config() 
	{
		map_params_.print();
	}

	bool Map::set( const std::string& map_path ) 
	{

		// Check if input path is empty.
		if ( map_path.empty() )
		{
			std::cout << "[ERROR] Need to provide a path" << std::endl;
			return false;
		}

		input_map_ = cv::imread( map_path, CV_LOAD_IMAGE_COLOR );

		// Check that input map has data. 
		if ( !input_map_.data ) {
			std::cout << "[ERROR] No image data. " << std::endl;
			return false;
		}

		// Resize input map to size specified parameters.
		cv::resize( input_map_, input_map_, 
			          cv::Size( map_params_.width_, map_params_.height_ ) );
		
		return true;
	}

	std::vector<std::vector<int>> Map::get() 
	{
		return bin_map_;
	}

	planner::MapParameters Map::get_configuration() 
	{
		return map_params_;
	}

	bool Map::create_obstacle_map() 
	{

		// Check that input map has data. 
		if ( !input_map_.data ) 
		{
			std::cout << "[ERROR] Empty input map" << std::endl;
			return false;
		}

		// Convert input map to grayscale
		cv::Mat gray_map( input_map_.size(), CV_8UC1 );
		cv::cvtColor( input_map_, gray_map, CV_BGR2GRAY );

		// Binarize grayscale map 
		cv::Mat binary_map( gray_map.size(), CV_8UC1 );
		cv::threshold( gray_map, binary_map, map_params_.min_thresh_, 
			             map_params_.max_thresh_, CV_THRESH_BINARY_INV );

		// Dilate binarized map
		cv::Mat dilated_map( binary_map.size(), CV_8UC1 ); 
	  cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, 
	  	cv::Size( 2 * map_params_.dilation_ + 1, 2 * map_params_.dilation_ + 1 ), 
	  	cv::Point( map_params_.dilation_, map_params_.dilation_) );

		cv::dilate( binary_map, dilated_map, element, cv::Point(-1, -1), 3 );
		
		// Create obstacle map
		create_bgr_obstacle_map( dilated_map );

		// Generate bin_map_ 
		if ( !generate_bin_map() ) 
		{
			std::cout << "[ERROR] Could not create binary map" << std::endl;
			return false;
		}

		// Draw grid on obstacle map.
		draw_grid();

		return true;
	}

	void Map::create_bgr_obstacle_map( const cv::Mat& dilated_map ) 
	{
		const cv::Vec3b blue  = cv::Vec3b(255,   0,   0);
		const cv::Vec3b white = cv::Vec3b(255, 255, 255);

		cv::Mat temp_obstacle_map( input_map_.size(), CV_8UC3 );

		for ( unsigned int r = 0; r < dilated_map.rows; r++ ) 
		{
			const uchar* R = dilated_map.ptr<uchar>( r );
			for ( unsigned int c = 0; c < dilated_map.cols; c++ )
			{
				uchar color = R[ c ];
				temp_obstacle_map.at<cv::Vec3b>(
					cv::Point( c, r ) ) = color == map_params_.max_thresh_ ? blue : white;
			}
		}
		obstacle_map_ = temp_obstacle_map;
	}

	bool Map::generate_bin_map() 
	{
		int win_size = map_params_.window_size_;
		const cv::Scalar black = cv::Scalar( 0, 0, 0 );

		cv::Rect bounds(0, 0, obstacle_map_.cols, obstacle_map_.rows);

		cv::Mat map_crop = cv::Mat( win_size, win_size, CV_8UC3, black );

		for ( unsigned int x = 0; x < obstacle_map_.cols; x += win_size ) 
		{
			for ( unsigned int y = 0; y < obstacle_map_.rows; y += win_size ) 
			{
				cv::Rect roi( x, y, win_size, win_size );
				map_crop = obstacle_map_( roi & bounds );
				
				int avg = roi_average( map_crop );
				if ( avg < 0 ) 
				{
					std::cout << "[ERROR] invalid values on map" << std::endl;
					return false;
				}

				bin_map_[ y / win_size ][ x / win_size ] = ( avg < FREE_CELL ? BLOCKED : FREE );
			}
		}
		return true;
	}

	void Map::display( ) 
	{
		cv::imshow( "Input Map", input_map_ );
		cv::imshow( "Obstacle Map", obstacle_map_);

		display_binmap();

		cv::waitKey( 0 );
	}

	void Map::draw_grid() 
	{
		unsigned int i = 0;
		const cv::Scalar black = cv::Scalar( 0, 0, 0 );

		// Draw vertical lines
		for ( i = 0; i < obstacle_map_.cols; i += map_params_.window_size_ ) 
		{
			cv::line( obstacle_map_, 
				        cv::Point( i, 0 ), 
				        cv::Point( i, obstacle_map_.rows - 1 ), 
				        black, 1 );
		}

		// Draw horizontal lines
		for ( i = 0; i < obstacle_map_.rows; i += map_params_.window_size_ ) 
		{
		 	cv::line( obstacle_map_, 
		 		        cv::Point( 0, i ), 
		 		        cv::Point( obstacle_map_.cols - 1, i ), 
		 		        black, 1 );
		}
	}

	int Map::roi_average( const cv::Mat& roi ) 
	{
		int num_elements = roi.rows * roi.cols;
		int sum = 0;

		for ( unsigned int i = 0; i < roi.rows; i++ ) 
		{
			for ( unsigned int j = 0; j < roi.cols; j++ ) 
			{
				sum += ( int ) roi.at<uchar>( i, j );
			}
		}
		return ( sum == 0 ? 0 : sum / num_elements );
	}

	void Map::display_binmap() 
	{
		// Iterate over elements in bimap
		for( auto& row: bin_map_ )
		{
			for( auto& col: row )
			{
				std::cout << col << " ";
			}
			std::cout << std::endl;
		}
	}

	void Map::trace_path( const std::vector<planner::Coord>& path) 
	{
		planner::Coord prev = { INT_MAX, INT_MAX };
		const cv::Scalar red = cv::Scalar( 0, 0, 255);

		for( auto& coord: path )
		{
			planner::Coord p = coord;

			if ( !(prev.c == INT_MAX) ) 
			{
				//C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
				cv::circle( input_map_, 
					          cv::Point( p.c * map_params_.window_size_, 
					          	         p.r * map_params_.window_size_ ),
					          2, red, 1);
			}
			std::cout << "->("<< p.r <<","<<p.c <<")";
			bin_map_[ p.r ][ p.c ] = 7;
			prev = p;
		}
		std::cout << std::endl;
	}
} // End of namespace planner
