#include "GroundGrid.h"
#include "GroundSegmentation.h"

double kitti_base_to_baseX = 1.95; //distance between lidar and front wheel in x-axis
double kitti_base_to_baseY = 0;
double kitti_base_to_baseZ = -1.73;
size_t thread_count = 8; // default 8, max 64, min 1
uint32_t max_ring = 0;
double min_outlier_detection_ground_confidence = 0.0;
double outlier_tolerance = -.5;
double patch_size_change_distance = 0.0;
double minimum_distance_factor = 0.0;
double miminum_point_height_threshold = 0.0;
double minimum_point_height_obstacle_threshold = 0.0;
double ground_patch_detection_minimum_point_count_threshold = 0.01;
double distance_factor = 0.0;
int point_count_cell_variance_threshold = 0.0;
double occupied_cells_point_count_factor = 1.0;
double occupied_cells_decrease_factor = 1.0;