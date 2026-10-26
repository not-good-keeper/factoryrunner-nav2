// Obstacle detection on a forward-looking depth image, with no ROS types so it can be unit
// tested. Built for the TurtleBot3 Waffle's stock RealSense depth sensor, which is mounted
// level and looks straight ahead.
#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace factoryrunner_vision
{

struct CameraModel
{
  double fx;
  double fy;
  double cx;
  double cy;
  double height_m;  // optical centre above the floor
};

struct DetectorConfig
{
  double min_height_m = 0.03;  // below this a return is floor (or floor noise)
  double max_height_m = 1.0;
  double corridor_half_width_m = 0.35;  // the waffle is ~0.31 m wide; leave a margin
  double max_range_m = 4.0;
  int min_blob_pixels = 30;
  double lidar_plane_height_m = 0.17;  // the 2D LiDAR sees nothing that stays below this
};

struct Obstacle
{
  cv::Rect box;  // in image pixels
  double range_m;  // forward distance to the closest point of the blob
  double lateral_m;  // sideways offset of that point, left positive
  double top_height_m;  // highest point of the blob above the floor
  int pixels;
  bool below_lidar_plane;
};

// 255 where a pixel is an obstacle inside the corridor, 0 elsewhere. depth is CV_32FC1 in
// metres along the optical axis; NaN, inf, zero and out-of-range values are ignored.
cv::Mat obstacle_mask(const cv::Mat & depth, const CameraModel & cam, const DetectorConfig & cfg);

// Connected obstacle blobs, closest first.
std::vector<Obstacle> detect(
  const cv::Mat & depth, const CameraModel & cam,
  const DetectorConfig & cfg);

}  // namespace factoryrunner_vision
