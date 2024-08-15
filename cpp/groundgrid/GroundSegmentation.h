

#pragma once

#include <Eigen/Eigen>
#include "GroundGrid.h"

#include <thread>
#include <algorithm>

namespace groundgrid
{
  class GroundSegmentation
  {
  public:
    GroundSegmentation() {};
    void init(const size_t dimension, const float &resolution)
    {
      const size_t cellCount = std::round(dimension / resolution);
      expectedPoints.resize(cellCount, cellCount);
      for (size_t i = 0; i < cellCount; ++i)
      {
        for (size_t j = 0; j < cellCount; ++j)
        {
          const float &dist = std::hypot(i - cellCount / 2.0, j - cellCount / 2.0);
          expectedPoints(i, j) = std::atan(1 / dist) / verticalPointAngDist;
        }
      }
      Eigen::initParallel();
    }

    std::vector<tPoint> filter_cloud(std::vector<tPoint> cloud, const tPoint cloudOrigin, tPose mapToBase, grid_map::GridMap &map)
    {
      static unsigned int time_vals = 0;

      std::vector<tPoint> filtered_cloud;
      filtered_cloud.reserve(cloud.size());

      map.add("groundCandidates", 0.0);
      map.add("planeDist", 0.0);
      map.add("m2", 0.0);
      map.add("meanVariance", 0.0);

      // raw point count layer for the evaluation
      map.add("pointsRaw", 0.0);

      map["groundCandidates"].setZero();
      map["points"].setZero();
      map["minGroundHeight"].setConstant(std::numeric_limits<float>::max());
      map["maxGroundHeight"].setConstant(std::numeric_limits<float>::min());

      map.add("variance", 0.0);
      static const grid_map::Matrix &ggv = map["variance"];
      static grid_map::Matrix &gpl = map["points"];
      static grid_map::Matrix &ggl = map["ground"];
      const auto &size = map.getSize();
      const size_t threadcount = thread_count;

      std::vector<std::pair<size_t, grid_map::Index>> point_index;
      point_index.reserve(cloud.size());
      std::vector<std::vector<std::pair<size_t, grid_map::Index>>> point_index_list;
      point_index_list.resize(threadcount);

      // Collect all outliers for the outlier detection evaluation
      std::vector<size_t> outliers;
      std::vector<std::vector<size_t>> outliers_list;
      outliers_list.resize(threadcount);

      // store ignored points to re-add them afterwards
      std::vector<std::pair<size_t, grid_map::Index>> ignored;
      std::vector<std::vector<std::pair<size_t, grid_map::Index>>> ignored_list;
      ignored_list.resize(threadcount);

      // Divide the point cloud into threadcount sections for threaded calculations
      std::vector<std::thread> threads;

      for (size_t i = 0; i < threadcount; ++i)
      {
        const size_t start = std::floor((i * cloud.size()) / threadcount);
        const size_t end = std::ceil(((i + 1) * cloud.size()) / threadcount);
        threads.push_back(std::thread(&GroundSegmentation::insert_cloud, this, cloud, start, end, std::cref(cloudOrigin), std::ref(point_index_list[i]), std::ref(ignored_list[i]),
                                      std::ref(outliers_list[i]), std::ref(map)));
      }

      // wait for results
      std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

      // join results
      for (const auto &point_index_part : point_index_list)
        point_index.insert(point_index.end(), point_index_part.begin(), point_index_part.end());
      for (const auto &outlier_index_part : outliers_list)
        outliers.insert(outliers.end(), outlier_index_part.begin(), outlier_index_part.end());
      for (const auto &ignored_part : ignored_list)
        ignored.insert(ignored.end(), ignored_part.begin(), ignored_part.end());

      // Divide the grid map into four section for threaded calculations
      threads.clear();
      for (unsigned short section = 0; section < 4; ++section)
        threads.push_back(std::thread(&GroundSegmentation::detect_ground_patches, this, std::ref(map), section));

      // wait for results
      std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
      spiral_ground_interpolation(map, mapToBase);
      map["points"].setConstant(0.0);

      // Re-add ignored points
      point_index.insert(point_index.end(), ignored.begin(), ignored.end());

      // Debugging statistics
      const double &min_dist_fac = minimum_distance_factor * 5;
      const double &min_point_height_thres = miminum_point_height_threshold;
      const double &min_point_height_obs_thres = minimum_point_height_obstacle_threshold;

      for (const std::pair<size_t, grid_map::Index> &entry : point_index)
      {
        const tPoint &point = cloud[entry.first];
        const grid_map::Index &gi = entry.second;
        const double &groundheight = ggl(gi(0), gi(1));

        const bool ground_point = point.ring == 40 || point.ring == 44 || point.ring == 48 || point.ring == 49 || point.ring == 60 || point.ring == 72;
        // copy the points intensity because it get's overwritten for evaluation purposes
        const float &variance = ggv(gi(0), gi(1));

        if (size(0) <= gi(0) + 3 || size(1) <= gi(1) + 3)
          continue;

        const float dist = std::hypot(point.poseX - cloudOrigin.poseX, point.poseY - cloudOrigin.poseY);
        const double tolerance = std::max(std::min((min_dist_fac * dist) / variance * min_point_height_thres, min_point_height_thres), min_point_height_obs_thres);

        if (tolerance + groundheight < point.poseZ)
        { // non-ground points
          tPoint &segmented_point = filtered_cloud.emplace_back(point);
          segmented_point.intensity = 99;
          gpl(gi(0), gi(1)) += 1.0f;
        }
        else
        {
          tPoint &segmented_point = filtered_cloud.emplace_back(point); // ground point
          segmented_point.intensity = 49;
        }
      }

      // Re-add outliers to cloud
      for (size_t i : outliers)
      {
        const tPoint &point = cloud[i];
        const bool ground_point = point.ring == 40 || point.ring == 44 || point.ring == 48 || point.ring == 49 || point.ring == 60 || point.ring == 72;
        tPoint &segmented_point = filtered_cloud.emplace_back(point); // ground point
        segmented_point.intensity = 49;
      }
      return filtered_cloud;
    }

    void insert_cloud(std::vector<tPoint> cloud, const size_t start, const size_t end, const tPoint &cloudOrigin, std::vector<std::pair<size_t, grid_map::Index>> &point_index,
                      std::vector<std::pair<size_t, grid_map::Index>> &ignored, std::vector<size_t> &outliers, grid_map::GridMap &map)
    {
      static const grid_map::Matrix &ggp = map["groundpatch"];

      static grid_map::Matrix &gpr = map["pointsRaw"];
      static grid_map::Matrix &gpl = map["points"];
      static grid_map::Matrix &ggl = map["ground"];
      static grid_map::Matrix &gmg = map["groundCandidates"];
      static grid_map::Matrix &gmm = map["meanVariance"];
      static grid_map::Matrix &gmx = map["maxGroundHeight"];
      static grid_map::Matrix &gmi = map["minGroundHeight"];
      static grid_map::Matrix &gmd = map["planeDist"];
      static grid_map::Matrix &gm2 = map["m2"];

      const auto &size = map.getSize();

      point_index.reserve(end - start);

      for (size_t i = start; i < end; ++i)
      {
        const tPoint &point = cloud[i];
        const auto &pos = grid_map::Position(point.poseX, point.poseY);
        const float sqdist = std::pow(point.poseX - cloudOrigin.poseX, 2.0) + std::pow(point.poseY - cloudOrigin.poseY, 2.0);

        bool toSkip = false;

        grid_map::Index gi;
        map.getIndex(pos, gi);

        if (!map.isInside(pos))
          continue;

        // point count map used for evaluation
        gpr(gi(0), gi(1)) += 1.0f;

        if (point.ring > max_ring || sqdist < minDistSquared)
        {
          ignored.push_back(std::make_pair(i, gi));
          continue;
        }

        // Outlier detection test
        const float oldgroundheight = ggl(gi(0), gi(1));
        if (point.poseZ < oldgroundheight - 0.2)
        {

          // get direction
          tPoint vec;
          vec.poseX = point.poseX - cloudOrigin.poseX;
          vec.poseY = point.poseY - cloudOrigin.poseY;
          vec.poseZ = point.poseZ - cloudOrigin.poseZ;

          float len = std::sqrt(std::pow(vec.poseX, 2.0f) + std::pow(vec.poseY, 2.0f) + std::pow(vec.poseZ, 2.0f));
          vec.poseX /= len;
          vec.poseY /= len;
          vec.poseZ /= len;

          // check for occlusion
          for (int step = 3; (std::pow(step * vec.poseX, 2.0) + std::pow(step * vec.poseY, 2.0) + std::pow(step * vec.poseZ, 2.0)) < std::pow(len, 2.0) && vec.poseZ < -0.01f; ++step)
          {
            grid_map::Index intersection, pointPosIndex;
            grid_map::Position intersecPos(step * (vec.poseX) + cloudOrigin.poseX, step * (vec.poseY) + cloudOrigin.poseY);
            map.getIndex(intersecPos, intersection);

            // Check if inside map borders
            if (intersection(0) <= 0 || intersection(1) <= 0 || intersection(0) >= size(0) - 1 || intersection(1) >= size(1) - 1)
              continue;

            // check if known ground occludes the line of sight
            const auto &block = ggp.block<3, 3>(std::max(intersection(0) - 1, 2), std::max(intersection(1) - 1, 2));
            if (block.sum() > min_outlier_detection_ground_confidence && ggp(intersection(0), intersection(1)) > 0.01f && ggl(intersection(0), intersection(1)) >= step * vec.poseZ + cloudOrigin.poseZ + outlier_tolerance)
            {
              outliers.push_back(i);
              toSkip = true;
              break;
            }
          }
        }

        if (toSkip)
          continue;

        float &groundheight = gmg(gi(0), gi(1));
        float &mean = gmm(gi(0), gi(1));

        float planeDist = 0.0;
        point_index.push_back(std::make_pair(i, gi));

        float &points = gpl(gi(0), gi(1));
        float &maxHeight = gmx(gi(0), gi(1));
        float &minHeight = gmi(gi(0), gi(1));
        float &planeDistMap = gmd(gi(0), gi(1));
        float &m2 = gm2(gi(0), gi(1));

        planeDist = point.poseZ - cloudOrigin.poseZ;
        groundheight = (point.poseZ + points * groundheight) / (points + 1.0);

        if (mean == 0.0)
          mean = planeDist;
        if (!std::isnan(planeDist))
        {
          float delta = planeDist - mean;
          mean += delta / (points + 1);
          planeDistMap = (planeDist + points * planeDistMap) / (points + 1.0);
          m2 += delta * (planeDist - mean);
        }

        maxHeight = std::max(maxHeight, static_cast<float>(point.poseZ));
        minHeight = std::min(minHeight, static_cast<float>(point.poseZ) - 0.0001f); // to make sure maxHeight > minHeight
        points += 1.0;
      }
    }

    void detect_ground_patches(grid_map::GridMap &map, unsigned short section) const
    {
      const grid_map::Matrix &gcl = map["groundCandidates"];
      const static auto &size = map.getSize();
      const static float resolution = map.getResolution();
      static const grid_map::Matrix &gm2 = map["m2"];
      static const grid_map::Matrix &gpl = map["points"];
      static grid_map::Matrix &ggv = map["variance"];
      // calculate variance
      ggv = gm2.array().cwiseQuotient(gpl.array() + std::numeric_limits<float>::min());

      int cols_start = 2 + section % 2 * (gcl.cols() / 2 - 2);
      int rows_start = section >= 2 ? gcl.rows() / 2 : 2;
      int cols_end = (gcl.cols()) / 2 + section % 2 * (gcl.cols() / 2 - 2);
      int rows_end = section >= 2 ? gcl.rows() - 2 : (gcl.rows()) / 2;

      for (int i = cols_start; i < cols_end; ++i)
      {
        for (int j = rows_start; j < rows_end; ++j)
        {
          const float sqdist = (std::pow(i - (size(0) / 2.0), 2.0) + std::pow(j - (size(1) / 2.0), 2.0)) * std::pow(resolution, 2.0);

          if (sqdist <= std::pow(patch_size_change_distance, 2.0))
            detect_ground_patch<3>(map, i, j);
          else
            detect_ground_patch<5>(map, i, j);
        }
      }
    }

    template <int S>
    void detect_ground_patch(grid_map::GridMap &map, size_t i, size_t j) const
    {
      static grid_map::Matrix &ggl = map["ground"];
      static grid_map::Matrix &ggp = map["groundpatch"];
      static grid_map::Matrix &ggv = map["variance"];
      static const grid_map::Matrix &gmi = map["minGroundHeight"];
      static const grid_map::Matrix &gpl = map["points"];
      static const auto &size = map.getSize();
      static const float resolution = map.getResolution();
      const int center_idx = std::floor(S / 2);

      const auto &pointsBlock = gpl.block<S, S>(i - center_idx, j - center_idx);
      const float sqdist = (std::pow(i - (size(0) / 2.0), 2.0) + std::pow(j - (size(1) / 2.0), 2.0)) * std::pow(resolution, 2.0);
      const int patchSize = S;
      const float &expectedPointCountperLaserperCell = expectedPoints(i, j);
      const float &pointsblockSum = pointsBlock.sum();
      float &oldConfidence = ggp(i, j);
      float &oldGroundheight = ggl(i, j);

      // early skipping of (almost) empty areas
      if (pointsblockSum < std::max(std::floor(mConfig.ground_patch_detection_minimum_point_count_threshold * patchSize * expectedPointCountperLaserperCell), 3.0))
        return;

      // calculation of variance threshold
      // limit the value to the defined minimum and 10 times the defined minimum
      const float varThresholdsq = std::min(std::max(sqdist * std::pow(mConfig.distance_factor, 2.0), std::pow(mConfig.minimum_distance_factor, 2.0)), std::pow(mConfig.minimum_distance_factor * 10, 2.0));
      const auto &varblock = ggv.block<S, S>(i - center_idx, j - center_idx);
      const auto &minblock = gmi.block<S, S>(i - center_idx, j - center_idx);
      const float &variance = varblock(center_idx, center_idx);
      const float &localmin = minblock.minCoeff();
      const float maxVar = pointsBlock(center_idx, center_idx) >= mConfig.point_count_cell_variance_threshold ? variance : pointsBlock.array().cwiseProduct(varblock.array()).sum() / pointsblockSum;
      const float groundlevel = pointsBlock.cwiseProduct(minblock).sum() / pointsblockSum;
      const float groundDiff = std::max((groundlevel - oldGroundheight) * (2.0f * oldConfidence), 1.0f);

      // Do not update known high confidence estimations upward
      if (oldConfidence > 0.5 && groundlevel >= oldGroundheight + mConfig.outlier_tolerance)
        return;

      if (varThresholdsq > std::pow(maxVar, 2.0) && maxVar > 0 && pointsblockSum > (groundDiff * expectedPointCountperLaserperCell * patchSize) * mConfig.ground_patch_detection_minimum_point_count_threshold)
      {
        const float &newConfidence = std::min(pointsblockSum / mConfig.occupied_cells_point_count_factor, 1.0);
        // calculate ground height
        oldGroundheight = (groundlevel * newConfidence + oldConfidence * oldGroundheight * 2) / (newConfidence + oldConfidence * 2);
        // update confidence
        oldConfidence = std::min((pointsblockSum / (mConfig.occupied_cells_point_count_factor * 2.0f) + oldConfidence) / 2.0, 1.0);
      }
      else if (localmin < oldGroundheight)
      {
        // update ground height
        oldGroundheight = localmin;
        // update confidence
        oldConfidence = std::min(oldConfidence + 0.1f, 0.5f);
      }
    }

    void spiral_ground_interpolation(grid_map::GridMap &map, const tPose &toBase) const
    {
      static grid_map::Matrix &ggl = map["ground"];
      static grid_map::Matrix &gvl = map["groundpatch"];
      const auto &map_size = map.getSize();
      const auto &center_idx = map_size(0) / 2 - 1;

      gvl(center_idx, center_idx) = 1.0f;
      tPoint ps;
      // tf2::doTransform(ps, ps, toBase);
      ps.poseX = ps.poseX + toBase.point.poseX;
      ps.poseY = ps.poseY + toBase.point.poseY;
      ps.poseZ = ps.poseZ + toBase.point.poseZ;
      // Set center to current vehicle height
      ggl(center_idx, center_idx) = ps.poseZ;

      for (int i = center_idx - 1; i >= 1; --i)
      {
        // rectangle_pos = x,y position of rectangle top left corner
        int rectangle_pos = i;

        // rectangle side length
        int side_length = (center_idx - rectangle_pos) * 2;

        // top and left side
        for (short side = 0; side < 2; ++side)
        {
          for (int pos = rectangle_pos; pos < rectangle_pos + side_length; ++pos)
          {
            const int x = side % 2 ? pos : rectangle_pos;
            const int y = side % 2 ? rectangle_pos : pos;

            interpolate_cell(map, x, y);
          }
        }

        // bottom and right side
        rectangle_pos += side_length;
        for (short side = 0; side < 2; ++side)
        {
          for (int pos = rectangle_pos; pos >= rectangle_pos - side_length; --pos)
          {
            int x = side % 2 ? pos : rectangle_pos;
            int y = side % 2 ? rectangle_pos : pos;

            interpolate_cell(map, x, y);
          }
        }
      }
    }

    void GroundSegmentation::interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const
    {
      static const auto &center_idx = map.getSize()(0) / 2 - 1;
      static const size_t blocksize = 3;
      // "groundpatch" layer contains confidence values
      static grid_map::Matrix &gvl = map["groundpatch"];
      // "ground" contains the ground height values
      static grid_map::Matrix &ggl = map["ground"];
      const auto &gvlblock = gvl.block<blocksize, blocksize>(x - blocksize / 2, y - blocksize / 2);

      float &height = ggl(x, y);
      float &occupied = gvl(x, y);
      const float &gvlSum = gvlblock.sum() + std::numeric_limits<float>::min(); // avoid a possible div by 0
      const float avg = (gvlblock.cwiseProduct(ggl.block<blocksize, blocksize>(x - blocksize / 2, y - blocksize / 2))).sum() / gvlSum;

      height = (1.0f - occupied) * avg + occupied * height;

      // Only update confidence in cells above min distance
      if ((std::pow((float)x - center_idx, 2.0) + std::pow((float)y - center_idx, 2.0)) * std::pow(map.getResolution(), 2.0f) > minDistSquared)
        occupied = std::max(occupied - occupied / occupied_cells_decrease_factor, 0.001);
    }

  protected:
    Eigen::MatrixXf expectedPoints;

    // velodyne 128: Average distance in rad on the unit circle of the appr. 220k points per round/128 Lasers
    const float verticalPointAngDist = 0.00174532925 * 2; // 0.2 degrees HDL-64e //0.00174532925; // 0.1 degrees //(2*M_PI)/(220000.0/128.0); // ca. 0.00365567:
    const float minDistSquared = 12.0f;
  };
}
