# Ground Grid
Keep the core and rewrite the program to remove dependencies required by [GroundGrid](https://github.com/dcmlr/groundgrid) like: ROS, PCL, tf and grid map. The update version is compatible with more operation system and more streamlined.

<!--  -->
📜 ChangeLog
- 2024-07-31: Ectracted useful function from "grid_map" library to local GridMap.h. GroundSegmentation::filter_cloud and GroundSegmentation::insert_cloud are on going
- 2024-08-06: Basic structure has been built.Rewrote kitti_data_publish from python to c++ to read and store velodyne, label, calib and poses data. Implemented trasformation based on Eigen and get rid of "tf".

## Problems Meet
1. If functions in class are defined in source file, they cannot be found. If defined directly in head files, everthing works. If the functions don't belong to a class like bigFind() shown in test.cpp and test.h, it also works. 
2. While coping 
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