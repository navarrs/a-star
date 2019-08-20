/**
 * @file:   map.h
 * @author: Ingrid Navarro
 * @brief:  This file contains the definitions for the class Map. This class is 
 *          used to create the map where the path planning will be performed. 
 *          It converts a given image into an obstacle grid. 
 */

#ifndef MAP
#define MAP

#include "planner.h"

namespace planner {
	
	class Map {

		public:
			/* --------------------------------------------------------------- *
			 * @name:   Map()
			 * @brief:  Constructor for the class Map.
			 * @param:  None.
			 * @return: Creates an instance of a map with default parameters. 
			 * -------------------------------------------------------------- */
			Map();

			/* --------------------------------------------------------------- *
			 * @name:   Map( std::string )
			 * @brief:  Constructor for the class Map.
			 * @param:  ( std::string ) map configuration file
			 *			( cv::Mat     ) input map 
			 * @return: Creates an instance of a map. 
			 * -------------------------------------------------------------- */
			Map( const std::string& map_configuration, cv::Mat &map);
			
			/* --------------------------------------------------------------- *
			 * @name:   ~Map( )
			 * @brief:  Destructor for the class Map.
			 * @param:  None.
			 * @return: Deletes an instance of a map. 
			 * -------------------------------------------------------------- */
			~Map() { };

			/* --------------------------------------------------------------- *
			 * @name:   set(...)
			 * @brief:  sets input map to file in given path. 
			 * @param:  ( std::string ) that indicates the path where the input 
			 *          map is located. 
			 * @return: True if successful, false otherwise.
			 * -------------------------------------------------------------- */
			bool  set( std::string input_path );

			/* --------------------------------------------------------------- *
			 * @name:   Get(...)
			 * @brief:  Gets input map parameters. 
			 * @return: Struct containing the parameters used to create the map.
			 * -------------------------------------------------------------- */
			MapParameters get();

			/* --------------------------------------------------------------- *
			 * @name:   create( int )
			 * @brief:  Returns content of input_map. 
			 * @param:  None
			 * @return: Void
			 * -------------------------------------------------------------- */
			bool  create( );

			/* --------------------------------------------------------------- *
			 * @name:   display( )
			 * @brief:  Displays maps
			 *          finding.
			 * @param:  None
			 * @return: void
			 * -------------------------------------------------------------- */
			void display();

			/* --------------------------------------------------------------- *
			 * @name:   trace_path( std::stack<Coord> )
			 * @brief:  Takes path computed by astar and traces it on the world map.
			 * @param:  ( std::stack<Coord> ) waypoints obtained from path finding.
			 * @return: void
			 * -------------------------------------------------------------- */
			void trace(std::stack<Coord> path_);
			
		private:
			/* --------------------------------------------------------------- *
			 * @name:   create_bgr_obstacle_map( const cv::Mat& )
			 * @brief:  Dilates obstacles on map to protect from colisions. 
			 * @param:  ( cv::Mat ) representing the binary obstacle map
			 * @return: void
			 * -------------------------------------------------------------- */
			void create_bgr_obstacle_map( const cv::Mat& dilated_map );

			/* --------------------------------------------------------------- *
			 * @name:   generate_bin_map( )
			 * @brief:  Creates obstacle graph to be used by the path planning 
			 *          algorithm.
			 * @param:  None
			 * @return: True if successful, false otherwise.
			 * -------------------------------------------------------------- */
			int generate_bin_map();

			/* --------------------------------------------------------------- *
			 * @name:   draw_grid( )
			 * @brief:  Draws a grid on input map based on user-specified window 
			 *          size.
			 * @param:  None
			 * @return: void
			 * -------------------------------------------------------------- */
			void draw_grid();

			/* --------------------------------------------------------------- *
			 * @name:   roi_average( cv::Mat )
			 * @brief:  Computes the (integer) average of an image RoI.
			 * @param:  ( cv::Mat ) image RoI.
			 * @return: ( int )     average of image RoI
			 * -------------------------------------------------------------- */
			int  roi_average(cv::Mat roi);

			/* --------------------------------------------------------------- *
			 * @name:   print_config( )
			 * @brief:  Prints map configuration.
			 * @param:  None			 
			 * @return: None
			 * -------------------------------------------------------------- */
			void print_config();

			/* --------------------------------------------------------------- *
			 * @name:   print( )
			 * @brief:  Prints to console the obstacle map needed for path 
			 *          finding.
			 * @param:  None
			 * @return: void
			 * -------------------------------------------------------------- */
			void print_bin();

			/* -------------------------------------------------------------- */
			cv::Mat input_map_;
			cv::Mat obstacle_map_;

			std::vector<std::vector<int> > bin_map_;
			std::stack<Coord> path_;

			MapParameters map_;

	}; // End of class Map
} // End of namespace planner 

#endif