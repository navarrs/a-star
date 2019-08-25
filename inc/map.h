/* --------------------------------------------------------------------------- *
 * @file:   map.h
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the definitions for the class Map. Such class is 
 *          used to create the a binary map that represents an occupancy map 
 *          from an image. 
 *
 * -------------------------------------------------------------------------- */

#ifndef MAP
#define MAP

#include "planner.h"

namespace planner 
{
	
	class Map 
	{
		public:
			/* -------------------------------------------------------------------- *
			 * @name:   Map
			 * @brief:  Constructor for the class Map.
			 *
			 * @param:  
			 *
			 * @return: Creates an instance of a map with default parameters. 
			 * ------------------------------------------------------------------- */
			Map();

			/* -------------------------------------------------------------------- *
			 * @name:   Map
			 * @brief:  Constructor for the class Map.
			 *
			 * @param:  map_configuration: Full path to the configuration of the map.
			 *			    map: OpenCV mat that represents the occupancy map. 
			 *
			 * @return: Creates an instance of a map from a given configuration. 
			 * -------------------------------------------------------------------- */
			Map( const std::string& map_configuration, cv::Mat &map);
			
			/* -------------------------------------------------------------------- *
			 * @name:   ~Map
			 * @brief:  Destructor for the class Map.
			 *
			 * @param:  
			 *
			 * @return: Deletes an instance of a map. 
			 * -------------------------------------------------------------------- */
			~Map() { };

			/* -------------------------------------------------------------------- *
			 * @name:   set
			 * @brief:  Sets an OpenCV mat to input map given an input path. 
			 *
			 * @param:  input_path: Full path where the input map can be found.
			 *
			 * @return: False if:
			 *						1) Input path is invalid.
			 *						2) Image is empty.
			 *          Otherwise, true.
			 * -------------------------------------------------------------------- */
			bool  set( const std::string& input_path );

			/* -------------------------------------------------------------------- *
			 * @name:   get
			 * @brief:  Gets a binary representation of the occupancy grid. 
			 *
			 * @param:
			 *
			 * @return: Binary occupancy grid.
			 * ------------------------------------------------------------------- */
			std::vector<std::vector<int>> get();

			/* -------------------------------------------------------------------- *
			 * @name:   get_configuration
			 * @brief:  Gets input map parameters. 
			 *
			 * @param:
			 *
			 * @return: Struct of type planner::MapParameters containing parameters 
			 *          used to create the input map.
			 * ------------------------------------------------------------------- */
			planner::MapParameters get_configuration();

			/* --------------------------------------------------------------------- *
			 * @name:   create_obstacle_map
			 * @brief:  Creates dilated occupancy grid from input map. 
			 *
			 * @param:  
			 *
			 * @return: False if:
			 *						1) Input map is empty.
			 *						2) Could not generate binary map.
			 *					Otherwise, true.
			 * -------------------------------------------------------------------- */
			bool create_obstacle_map();

			/* -------------------------------------------------------------------- *
			 * @name:   display
			 * @brief:  Displays input map, occupancy map with grid and binary map
			 *
			 * @param:  
			 *
			 * @return: 
			 * -------------------------------------------------------------------- */
			void display();

			/* -------------------------------------------------------------------- *
			 * @name:   trace_path( std::stack<Coord> )
			 * @brief:  Takes path computed by astar and traces it on the world map.
			 * @param:  ( std::stack<Coord> ) waypoints obtained from path finding.
			 * @return: void
			 * -------------------------------------------------------------------- */
			// void trace(std::stack<Coord> path_);
			
		private:
			/* -------------------------------------------------------------------- *
			 * @name:   create_bgr_obstacle_map
			 * @brief:  Creates a BGR occupancy map with dilated obstacles.
			 *
			 * @param:  dilated_map: Grayscale obstacle map.
			 *
			 * @return: 
			 * -------------------------------------------------------------------- */
			void create_bgr_obstacle_map( const cv::Mat& dilated_map );

			/* -------------------------------------------------------------------- *
			 * @name:   generate_bin_map
			 * @brief:  Creates a binary occupancy map. 
			 *
			 * @param:  
			 *
			 * @return: False if:
			 *						1) Map has invalid values. 
			 *					Otherwise, true.
			 * -------------------------------------------------------------------- */
			bool generate_bin_map();

			/* -------------------------------------------------------------------- *
			 * @name:   draw_grid
			 * @brief:  Draws a grid on the BGR obstacle map based on window size 
			 *          specified in configuration. 
			 *
			 * @param:  
			 *
			 * @return: 
			 * -------------------------------------------------------------------- */
			void draw_grid();

			/* -------------------------------------------------------------------- *
			 * @name:   roi_average
			 * @brief:  Computes the (integer) average of an image RoI to determine
			 *          if a cell is empty or blocked. 
			 *
			 * @param:  roi: Region of Interest of map.
			 *
			 * @return: Average of the image RoI.
			 * -------------------------------------------------------------------- */
			int  roi_average( const cv::Mat& roi);

			/* -------------------------------------------------------------------- *
			 * @name:   print_config
			 * @brief:  Prints map class configuration.
			 *
			 * @param:  			 
			 *
			 * @return: 
			 * -------------------------------------------------------------------- */
			void print_config();

			/* --------------------------------------------------------------------- *
			 * @name:   display_binmap()
			 * @brief:  Prints to console the binary obstacle map,
			 *
			 * @param:  
			 *
			 * @return: 
			 * -------------------------------------------------------------------- */
			void display_binmap();

			/* -------------------------------------------------------------------- */
			
			// Input map provided by the user.
			cv::Mat input_map_;

			// Occupancy map: Input map with dilated obstacles.
			cv::Mat obstacle_map_;

			// Binary occupancy map.
			std::vector<std::vector<int>> bin_map_;

			// Configuration parameters of the input map.
			MapParameters map_params_;

	}; // End of class Map
} // End of namespace planner 

#endif