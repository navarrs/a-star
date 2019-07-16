#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <string>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
using namespace cv;

// Window size 
#define IM_WIDTH   640 
#define IM_HEIGHT  480
#define IM_WINDOW   10

// Grid size
#define GRID_WIDTH  IM_WIDTH / IM_WINDOW
#define GRID_HEIGHT IM_HEIGHT / IM_WINDOW

// Threshold to create binary maps
#define THRESH_MIN      225
#define THRESH_MAX      255

// Threshold to detect obstacles
#define THRESH_OBSTACLE 200

// Colors
#define BLACK Scalar(0,     0,   0)
#define BLUE  Scalar(255,   0,   0)
#define GREEN Scalar(  0, 255,   0)
#define RED   Scalar(  0,   0, 255)
#define WHITE Scalar(255, 255, 255)

#define BLUE_V3  Vec3b(255,   0,    0)
#define WHITE_V3 Vec3b(255, 255, 255)

// Common parameters
#define THICK   1
#define RADIUS  4
#define ESC    27

// Shortcuts to make pairs 
typedef pair<int, int> Pair; 
typedef pair<double, pair<int, int> > pPair; 

Pair src  = make_pair(INT_MAX, INT_MAX);
Pair dest = make_pair(INT_MAX, INT_MAX);

// node structure
struct node {
	int parent_i, parent_j;
	double f, h, g;
};

// Check if pair on grid is not out of range
bool is_pair_valid(int r, int c) {
	return (r >= 0 && r < GRID_HEIGHT && c >= 0 && c < GRID_WIDTH);
}

// Check if pair on grid is not blocked 
bool is_pair_blocked(const vector<vector<int> > &grid, int r, int c) {
	return grid[r][c];
}

// Check if source is destination
bool is_destination(int r, int c, Pair d) {
	return (r == d.first) && (c == d.second);
}

// Compute f, g, h
double compute_g_cost(double g) {
	return g + 1.0;
}

double compute_heuristic_cost(int r, int c, Pair d) {
	return ((double) sqrt(pow((r-d.first), 2) + pow((c-d.second), 2)));
}

double compute_f_cost(double g, double h) {
	return g + h;
}

// Trace path 
void trace_path(node node_list[][GRID_WIDTH], Pair dest, vector<vector <int> > &grid, Mat &map) {
	cout << "Path is: " << endl;
	int row = dest.first;
	int col = dest.second;
	stack<Pair> path;

	while(!(node_list[row][col].parent_i == row && node_list[row][col].parent_j == col)) {
		path.push(make_pair(row, col));
		int temp_row = node_list[row][col].parent_i;
		int temp_col = node_list[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	path.push(make_pair(row, col));
	Pair prev = make_pair(INT_MAX, INT_MAX);

	while(!path.empty()) {
		pair<int, int> p = path.top();

		if (!(prev.first == INT_MAX)) {
			line(map, 
				Point(p.second * IM_WINDOW + IM_WINDOW, p.first * IM_WINDOW + IM_WINDOW), 
				Point(prev.second * IM_WINDOW + IM_WINDOW, prev.first * IM_WINDOW + IM_WINDOW), 
				RED, THICK, RADIUS);
		}

		path.pop();
		cout << "->("<< p.first <<","<<p.second<<")";
		grid[p.first][p.second] = 5;
		prev = p;
	}
	cout << endl;
}

// A* algorithm 
void a_star_search(vector<vector<int> > &grid, Pair src, Pair dest, Mat &map) {

	// Assert that neither source or destination are out of range
	if (!is_pair_valid(src.first, src.second)) {
		cout << "Source is invalid." << endl;
		return;
	}
	if (!is_pair_valid(dest.first, dest.second)) {
		cout << "Destination is invalid." << endl;
		return;
	}

	// Assert that neither source or destination are blocked
	if (is_pair_blocked(grid, src.first, src.second) || is_pair_blocked(grid, dest.first, dest.second)) {
		cout << "Either source or destination are blocked." << endl;
		return;
	}

	// Assert that source is not destination 
	if (is_destination(src.first, src.second, dest)) {
		cout << "Already at destination." << endl;
		return;
	}

	// Initialize the closed list
	bool closed_list[GRID_HEIGHT][GRID_WIDTH];
	memset(closed_list, false, sizeof(closed_list));
	node node_list[GRID_HEIGHT][GRID_WIDTH];
	int i, j;
	for (i = 0; i < GRID_HEIGHT; i++) {
		for (j = 0; j < GRID_HEIGHT; j++) {
			node_list[i][j].f = FLT_MAX;
			node_list[i][j].h = FLT_MAX;
			node_list[i][j].g = FLT_MAX;
			node_list[i][j].parent_i = -1;
			node_list[i][j].parent_j = -1;
		}
	}

	// Initialize the parameters of the starting node. 
	i = src.first;
	j = src.second;
	node_list[i][j].f = 0.0;
	node_list[i][j].g = 0.0;
	node_list[i][j].h = 0.0;
	node_list[i][j].parent_i = i;
	node_list[i][j].parent_j = j;

	/*
		Create open list having information as <f, <i, j>>, 
		where f = g + h, and i, j are the row and column index
		of that node. 
		Note that 0 <= i < ROW & 0 <= j < COL
		This open list is implenented as a set of pair of pair.
	*/
	set<pPair> open_list;
	// Put starting node on the open list and set its 'f' as 0.0
	open_list.insert(make_pair(0.0, make_pair(i, j)));
	bool found_destination = false;

	while (!open_list.empty()) {

		pPair p = *open_list.begin();
		
		// Remove vertex from open list
		open_list.erase(open_list.begin());

		// Add this vertex to the closed list
		i = p.second.first;
		j = p.second.second;
		closed_list[i][j] = true;

		// Generate all successors
		double g_new, f_new, h_new;

		for (int temp_i = i - 1; temp_i <= i + 1; temp_i++) {
			for (int temp_j = j - 1; temp_j <= j + 1; temp_j++) {
				if (is_pair_valid(temp_i, temp_j)) {
					if (is_destination(temp_i, temp_j, dest)) {
						node_list[temp_i][temp_j].parent_i = i;
						node_list[temp_i][temp_j].parent_j = j;
						trace_path(node_list, dest, grid, map);
						found_destination = true;
						return;
					} else if (!closed_list[temp_i][temp_j] && !is_pair_blocked(grid, temp_i, temp_j)) {
						g_new = compute_g_cost(node_list[temp_i][temp_j].g);
						h_new = compute_heuristic_cost(temp_i, temp_j, dest);
						f_new = compute_f_cost(g_new, h_new);
						// If it isn’t on the open list, add it to 
		                // the open list. Make the current square 
		                // the parent of this square. Record the 
		                // f, g, and h costs of the square node 
		                //                OR 
		                // If it is on the open list already, check 
		                // to see if this path to that square is better, 
		                // using 'f' cost as the measure. 
		                if (node_list[temp_i][temp_j].f == FLT_MAX || node_list[temp_i][temp_j].f > f_new) {
		                	open_list.insert(make_pair(f_new, make_pair(temp_i, temp_j)));
		                	node_list[temp_i][temp_j].f = f_new;
		                	node_list[temp_i][temp_j].g = g_new;
		                	node_list[temp_i][temp_j].h = h_new;
		                	node_list[temp_i][temp_j].parent_i = i;
		                	node_list[temp_i][temp_j].parent_j = j;
		                }
					}
				}
			}
		} 
	}
	if (!found_destination) {
		cout << "Could not find destination." << endl;
		return;
	}
}


/*	@method: average(...)
	@brief: Computes the (integer) average of an image patch.
	@param:
		patch (Mat)  Image patch
	@returns
		avg (int)   Average of image patch. 
*/
int average(Mat patch);
	
/*  @method: dilate_obstacles(...)
	@brief: Creates an RGB obstacle map with dilated obstacles.
	@param:
		obstacle_map (&Mat)			RGB obstacle map
		dilated_map (const &Mat)	dilated binary map
	@returns
		void
*/
void dilate_obstacles(Mat& obstacle_map, const Mat& dilated_map);

/*  @method: draw_grid(...)
	@brief: Draws a grid on input map based on user-specified window size.
	@params: 
		map (&Mat)    input map. 
	@returns:
		void
*/
void draw_grid(Mat &map);

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


/*	@method: print_graph(...)
	@brief:  Prints graph.
	@param:
		v (vector<vector<int> >) input graph
	@returns:
		void
*/
void print_graph(vector<vector<int> > v);

/*	@method: generate_graph(...)
	@brief:  Creates obstacle graph to be used by the 
			 A* algorithm for path planning. 
	@param:
		map (Mat) binary map that contains dilated obstacles. 
		graph (vector<vector<int> >) graph used by A*
	@returns:
		void
*/
void generate_graph(Mat map, vector<vector <int> > &graph);


int main(int argc, char* argv[]) {
	//Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("inpath", po::value<string>()->default_value("../maps/map1.png"), "Path to input file.")
		("dilation", po::value<int>()->default_value(10), "Size of dilation kernel. ");
	po::variables_map opts;
	po::store(po::command_line_parser(argc, argv).options(description).run(), opts);
	try {
		po::notify(opts);
	} catch (exception& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}
	if (opts.count("help")) {
		cout << description;
		return 1;
	}
	string input_path = opts["inpath"].as<string>();
	int dilation = opts["dilation"].as<int>();

	// Read input image
	Mat input_map = imread(input_path, 1);
	if (!input_map.data) {
		cout << "No image data." << endl;
		return -1;
	}
	resize(input_map, input_map, Size(IM_WIDTH, IM_HEIGHT));
	imshow("Input Map", input_map);

	// Convert input map to grayscale 
	Mat gray_map;
	cvtColor(input_map, gray_map, CV_BGR2GRAY);
	imshow("Grayscale Map", gray_map);

	// Create binary map 
	Mat binary_map(gray_map.size(), CV_8UC1);
	threshold(gray_map, binary_map, THRESH_MIN, THRESH_MAX, THRESH_BINARY_INV);
	imshow("Binary Map", binary_map);

	// Mat dilated_map;
	Mat dilated_map(binary_map.size(), CV_8UC1); 
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*dilation+1, 2*dilation+1), Point(dilation, dilation));
	dilate(binary_map, dilated_map, element, Point(-1, -1), 3);
	imshow("Dilated Map", dilated_map);

	// Create an obtacle map 
	Mat obstacle_map(input_map.size(), CV_8UC3);
	dilate_obstacles(obstacle_map, dilated_map);
	imshow("Obstacle RGB Map", obstacle_map);

	// Generate graph 
	// int graph[map.rows / IM_WINDOW][map.cols / IM_WINDOW];
	vector<int> g(GRID_WIDTH, 0);
	vector<vector<int> > graph(GRID_HEIGHT, g);
	generate_graph(obstacle_map, graph);

	// Draw grid on map 
	draw_grid(obstacle_map);
	namedWindow("Obstacle Map", 1);
	setMouseCallback("Obstacle Map", mouse_callback, &obstacle_map);
	bool stop = false;
	while (!stop) {
		int key = waitKey(15);
		stop = key == ESC ? true : false;
		imshow("Obstacle Map", obstacle_map);
	}

	// Print graph
	print_graph(graph);

	src = make_pair(40, 44); // row , col
	dest = make_pair(4, 20);
	// Draw starting and ending points
	circle(input_map, Point(src.second * IM_WINDOW + IM_WINDOW, src.first * IM_WINDOW + IM_WINDOW), 5, Scalar(255, 0, 0), 5, 8);
	circle(input_map, Point(dest.second * IM_WINDOW + IM_WINDOW, dest.first * IM_WINDOW + IM_WINDOW), 5, Scalar(0, 255, 0), 5, 8);

	// Compute A*
	a_star_search(graph, src, dest, input_map);
	print_graph(graph);
	imshow("Path", input_map);
	waitKey(0);

	return 0;
}

// Creates RGB obstacle map
void dilate_obstacles(Mat& obstacle_map, const Mat& dilated_map) {
	for (int i = 0; i < dilated_map.rows; i++) {
		for (int j = 0; j < dilated_map.cols; j++) {
			uchar color = dilated_map.at<uchar>(i, j);
			obstacle_map.at<Vec3b>(Point(j, i)) = color == THRESH_MAX ? BLUE_V3 : WHITE_V3;
		}
	}
}

// Computes average of image patch 
int average(Mat patch) {
	int num_elements = patch.rows * patch.cols;
	int sum = 0;
	for (int i = 0; i < patch.rows; i++) {
		for (int j = 0; j < patch.cols; j++) {
				sum += (int)patch.at<uchar>(i, j);
		}
	}
	return (sum == 0 ? 0 : sum / num_elements);
}

// Print graph
void print_graph(vector<vector<int> > v) {
	for (int i = 0; i < v.size(); i++) {
		for (int j = 0; j < v[i].size(); j++) {
			cout << v[i][j] << " ";
		}
		cout << endl;
	}
}

// Generate graph 
void generate_graph(Mat map, vector<vector <int> > &graph) {
	Rect bounds(0, 0, map.cols, map.rows);
	Mat map_crop = Mat(IM_WINDOW, IM_WINDOW, CV_8UC3, Scalar(0,0,0));
	for (int x = 0; x < map.cols; x += IM_WINDOW) {
		for (int y = 0; y < map.rows; y += IM_WINDOW) {
			Rect roi(x, y, IM_WINDOW, IM_WINDOW);
			map_crop = map(roi & bounds);
			int avg = average(map_crop);
			graph[y / IM_WINDOW][x / IM_WINDOW] = (avg < 220 ?  1 : 0);
		}
	}
}

// Draws grid on map
void draw_grid(Mat &map) {
	int i = 0;
	// Draw vertical line
	for (i = 0; i < map.cols; i += IM_WINDOW) {
		line(map, Point(i, 0), Point(i, map.rows-1), BLACK, THICK);
	}
	// Draw horizontal line
	for (i = 0; i < map.rows; i += IM_WINDOW) {
	 	line(map, Point(0, i), Point(map.cols-1, i), BLACK, THICK);
	}
}

// Mouse callback to set start and destination 
void mouse_callback(int event, int x, int y, int flags, void* param) {
	Mat img = *((Mat *) param);
	Rect bounds(0, 0, img.cols, img.rows);

	int init_x = (x / IM_WINDOW);
	int init_y = (y / IM_WINDOW);
	// Set source with left button click 
	if (event == EVENT_LBUTTONDBLCLK) { 
		// Get patch that corresponds to location 
		Mat patch(IM_WINDOW, IM_WINDOW, CV_8UC3);
		Rect roi(init_x * IM_WINDOW, init_y * IM_WINDOW, IM_WINDOW, IM_WINDOW);
		patch = img(roi & bounds);
		cvtColor(patch, patch, CV_BGR2GRAY);

		// Get average of patch to determine if it is an obstacle or not
		if (average(patch) < THRESH_OBSTACLE) {
			cout << "Invalid location. " << endl;
		} else {
			cout << "Setting source at (" << x << ", " << y << ")" << endl;
			// If source has already been set, delete it on map
			if (src.second != INT_MAX) {
				cout << "Resetting source at (" << x << ", " << y << ")" << endl;
				circle(img, 
					Point((src.second + 0.5) * IM_WINDOW, (src.first + 0.5) * IM_WINDOW), 
					RADIUS, WHITE, 2*THICK);
			}
			// Set source
			src.second = init_x;
			src.first = init_y;
			circle(img, 
				Point((src.second + 0.5) * IM_WINDOW, (src.first + 0.5) * IM_WINDOW), 
				RADIUS, GREEN, 2*THICK);
			cout << "Scaled source at (" << src.second << ", " << src.first << ")" << endl;
		}
	// Set destination with right button click 
	} else if (event == EVENT_MBUTTONDBLCLK ) {
		// Get patch that corresponds to location 
		Mat patch(IM_WINDOW, IM_WINDOW, CV_8UC3);
		Rect roi(init_x * IM_WINDOW, init_y * IM_WINDOW, IM_WINDOW, IM_WINDOW);
		patch = img(roi & bounds);
		cvtColor(patch, patch, CV_BGR2GRAY);
		
		// Get average of patch to determine if it is an obstacle or not
		if (average(patch) < THRESH_OBSTACLE) {
			cout << "Invalid location. " << endl;
		} else {
			cout << "Setting destination at (" << x << ", " << y << ")" << endl;
			// If source has already been set, delete it on map
			if (dest.second != INT_MAX) {
				cout << "Resetting destination at (" << x << ", " << y << ")" << endl;
				circle(img, 
					Point((dest.second + 0.5) * IM_WINDOW, (dest.first + 0.5) * IM_WINDOW), 
					RADIUS, WHITE, 2*THICK);
			}
			// Set source
			dest.second  = init_x;
			dest.first = init_y;
			circle(img, 
				Point((dest.second + 0.5) * IM_WINDOW, (dest.first + 0.5) * IM_WINDOW), 
				RADIUS, RED, 2*THICK);
			cout << "Scaled destination at (" << dest.second << ", " << dest.first << ")" << endl;
		}
	}
}