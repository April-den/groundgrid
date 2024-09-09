# Ground Grid
Keep the core and rewrite the program to remove dependencies required by [GroundGrid](https://github.com/dcmlr/groundgrid) like: ROS, PCL, tf and grid map. The update version is compatible with more operation system and more streamlined.

<!--  -->
📜 ChangeLog
- 2024-07-31: Ectracted useful function from "grid_map" library to local GridMap.h. GroundSegmentation::filter_cloud and GroundSegmentation::insert_cloud are on going
- 2024-08-06: Basic structure has been built.Rewrote kitti_data_publish from python to c++ to read and store velodyne, label, calib and poses data. Implemented trasformation based on Eigen and get rid of "tf"
- 2024-08-06 Fix the bug of abnormal output after transformation. GridMapConfig.h is temporally used to store parameters. Later it will be witten in the form of ".toml"
- 2024-08-14 Structure completed but segement results are wrong
- 2024-08-22 Fix the bug while coping point clouds and coordinates transformation
- 2024-09-02 Successfully segment point cloud of one frame

## Problems Meet

1. While coping 
void wrapIndexToRange(Index &index, const Size &bufferSize)
    {
        for (int i = 0; i < index.size(); i++)
        {
            wrapIndexToRange(index[i], bufferSize[i]);
        }
    }

void wrapIndexToRange(int &index, int bufferSize){...}

Got error: invalid initialization of reference of type ‘grid_map::Index&’ {aka ‘Eigen::Array<int, 2, 1>&’} from expression of type ‘Eigen::DenseCoeffsBase<Eigen::Array<int, 2, 1>, 1>::Scalar’ {aka ‘int’}. 
Don't know what cause the compile error. Solved by without calling the reload function "void wrapIndexToRange(int &index, int bufferSize)" in "void wrapIndexToRange(Index &index, const Size &bufferSize)", but directly implementing the function.

2. In cpp folder, I try to use "Eigen" library to implement coordinate transformation and calibration. "Eigen" provides low precision result compared with matlab and python for matrix calculation. In the calculated roattion matrix, only the pose is closed to matlab and python output. So in the c++ program, I ignored the rotation parameters. 

3. There maybe several bugIt's necessary to emphasize the naming style for transformation in kitti_data_publisher.py, GroundGrid.cpp, GroundGridNodelet.cpp, KITTIEvaluate.launch and KITTIPlayback.launch of [GroundGrid](https://github.com/dcmlr/groundgrid). In original code, the developer use "base_to_map" to represent parameters for coordinate transformation from base to map. It's only for coordinate system from base to map, but not for points. If we get (poseX, poseY, poseZ) for base_to_map and our points are in the base coordinate system, the transformed points have x = points.x - poseX; y = points.y - poseY; z - points.z+poseZ. The name base_to_map actually means transform points in "map" to points in "base". The official document (tf::TransformBroadcaster::sendTransform(parent frame, child frame), tf2_ros::Buffer::lookupTransform(parent frame, child frame)) also state that "send a transformation from the child to the parent frame", which is oposite to the name grid_to_map.

4. Don't understand why the if condition is "std::hypot(lastOdom.point.poseX-inOdom.point.poseX, 2.0f) + std::hypot(lastOdom.point.poseY-inOdom.point.poseY, 2.0f) >= 1.0". In my opinion, the grid_map should update, if the movement in x_axis or y_axis is larger than the resolution. Don't 

5. Use this command "g++ cpp/groundgrid/main.cpp -o main -lpthread" to make the multi thread computation available. Otherwise, got this "undefined reference to `pthread_create'collect2: error: ld returned 1 exit status". Directly linking static library in tasks.json doesn't work.
