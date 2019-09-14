/* -------------------------------------------------------------------------- *
 * @file:   map.h
 * @date:   August 24, 2019
 * @author: Ingrid Navarro
 *
 * @brief:  This file contains the definitions for the class Map. Such class is 
 *          used to create the a binary map and an obstacle map that represent 
 *          the occupancy map. 
 *
 * -------------------------------------------------------------------------- */

#ifndef MAP
#define MAP

#include "planner.h"

namespace planner {
	
class Map {
	public:
		/* ---------------------------------------------------------------------- *
		 * @name:   Map
		 * @brief:  Default constructor for the class Map.
		 *
		 * @param:  
		 *
		 * @return: Creates an instance of a map with default parameters. 
		 * */
		Map();

		/* ---------------------------------------------------------------------- *
		 * @name:   Map
		 * @brief:  Constructor for the class Map.
		 *
		 * @param:  map_configuration(in): Full path to the configuration of the map.
		 *			    map(in): OpenCV mat that represents the occupancy map. 
		 *
		 * @return: Creates an instance of a map from a given configuration. 
		 * */
		Map(const std::string& map_configuration, cv::Mat& map);
		
		/* ---------------------------------------------------------------------- *
		 * @name:   ~Map
		 * @brief:  Destructor for the class Map.
		 *
		 * @param:  
		 *
		 * @return: Deletes an instance of a map. 
		 * */
		~Map() { /* Empty body */ };

		/* ---------------------------------------------------------------------- *
		 * @name:   set
		 * @brief:  Sets the input map from a given path.  
		 *
		 * @param:  input_path(in): Full path where the input map can be found.
		 *
		 * @return: False if:
		 *						1) Input path is invalid.
		 *						2) Image is empty.
		 *          Otherwise, true.
		 * */
		bool set_input_map(const std::string& input_path);

		/* ---------------------------------------------------------------------- *
		 * @name:   get_binary_map
		 * @brief:  Gets a binary representation of the occupancy grid. 
		 *
		 * @param:
		 *
		 * @return: Binary occupancy grid.
		 * */
		std::vector<std::vector<unsigned int>> get_binary_map();

		/* ---------------------------------------------------------------------- *
		 * @name:   get_obstacle_map
		 * @brief:  Gets a obstacle map. 
		 *
		 * @param:
		 *
		 * @return: cv::Mat that represents the obstacle map.
		 * */
		cv::Mat get_obstacle_map();

		/* ---------------------------------------------------------------------- *
		 * @name:   get_configuration
		 * @brief:  Gets input map parameters. 
		 *
		 * @param:
		 *
		 * @return: Struct of type planner::MapParameters containing parameters 
		 *          used to create the input map.
		 * */
		planner::MapParameters get_configuration();

		/* ---------------------------------------------------------------------- *
		 * @name:   create_obstacle_map
		 * @brief:  Creates dilated occupancy grid from input map. 
		 *
		 * @param:  
		 *
		 * @return: False if:
		 *						1) Input map is empty.
		 *						2) Could not generate binary map.
		 *					Otherwise, true.
		 * */
		bool create_obstacle_map();

		/* ---------------------------------------------------------------------- *
		 * @name:   display
		 * @brief:  Displays input map, occupancy map with grid and binary map
		 *
		 * @param:  
		 *
		 * @return: 
		 * */
		void display();

		/* ---------------------------------------------------------------------- *
		 * @name:   trace_path
		 * @brief:  Takes the path computed by the planner and traces it on the 
		 *          input map and on the binary map.
		 *
		 * @param:  path(in): Vector of coordinates that represent the path. 
		 *
		 * @return: 
		 * */
		void trace_path(const std::vector<planner::Coord>& path);
		
	private:
		/* ---------------------------------------------------------------------- *
		 * @name:   roi_average
		 * @brief:  Computes the (integer) average of an image RoI to determine
		 *          if a cell is empty or blocked. 
		 *
		 * @param:  roi(in): Region of Interest of map.
		 *
		 * @return: Average of the image RoI.
		 * */
		unsigned int roi_average(const cv::Mat& roi);

		/* ---------------------------------------------------------------------- *
		 * @name:   generate_binary_map
		 * @brief:  Creates a binary occupancy map. 
		 *
		 * @param:  
		 *
		 * @return: False if:
		 *						1) Map has invalid values. 
		 *					Otherwise, true.
		 * */
		bool generate_binary_map();

		/* ---------------------------------------------------------------------- *
		 * @name:   create_bgr_obstacle_map
		 * @brief:  Creates a BGR occupancy map with dilated obstacles.
		 *
		 * @param:  dilated_map(in): Grayscale obstacle map.
		 *
		 * @return: 
		 * */
		void create_bgr_obstacle_map(const cv::Mat& dilated_map);


		/* ---------------------------------------------------------------------- *
		 * @name:   draw_grid
		 * @brief:  Draws a grid on the BGR obstacle map based on window size 
		 *          specified in configuration. 
		 *
		 * @param:  
		 *
		 * @return: 
		 * */
		void draw_grid();

		/* ---------------------------------------------------------------------- *
		 * @name:   display_binmap()
		 * @brief:  Prints to console the binary obstacle map,
		 *
		 * @param:  
		 *
		 * @return: 
		 * */
		void display_binmap();

		/* ---------------------------------------------------------------------- *
		 * Members 
		 * */
		
		// Input map provided by the user.
		cv::Mat input_map_;

		// Occupancy map: Input map with dilated obstacles.
		cv::Mat obstacle_map_;

		// Binary occupancy map.
		std::vector<std::vector<unsigned int>> binary_map_;

		// Configuration parameters of the input map.
		MapParameters map_params_;

}; // End of class Map.
}  // End of namespace planner.

#endif