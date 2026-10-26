#include "factoryrunner_vision/depth_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace factoryrunner_vision
{

namespace
{

struct Point3
{
  double forward;
  double lateral;
  double height;
};

// Pinhole back-projection for a level, forward-looking camera: image right is robot right,
// image down is towards the floor.
Point3 back_project(int u, int v, double depth, const CameraModel & cam)
{
  return {
    depth,
    -(u - cam.cx) * depth / cam.fx,
    cam.height_m - (v - cam.cy) * depth / cam.fy};
}

bool valid_depth(float d, const DetectorConfig & cfg)
{
  return std::isfinite(d) && d > 0.0f && d <= cfg.max_range_m;
}

}  // namespace

cv::Mat obstacle_mask(const cv::Mat & depth, const CameraModel & cam, const DetectorConfig & cfg)
{
  CV_Assert(depth.type() == CV_32FC1);
  cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
  for (int v = 0; v < depth.rows; ++v) {
    const float * row = depth.ptr<float>(v);
    auto * out = mask.ptr<uchar>(v);
    for (int u = 0; u < depth.cols; ++u) {
      if (!valid_depth(row[u], cfg)) {
        continue;
      }
      const Point3 p = back_project(u, v, row[u], cam);
      if (p.height >= cfg.min_height_m && p.height <= cfg.max_height_m &&
        std::abs(p.lateral) <= cfg.corridor_half_width_m)
      {
        out[u] = 255;
      }
    }
  }
  // Single stray pixels at depth edges are common; an opening removes them without eating
  // into real blobs.
  cv::morphologyEx(
    mask, mask, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  return mask;
}

std::vector<Obstacle> detect(
  const cv::Mat & depth, const CameraModel & cam,
  const DetectorConfig & cfg)
{
  const cv::Mat mask = obstacle_mask(depth, cam, cfg);
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

  std::vector<Obstacle> found;
  for (int label = 1; label < count; ++label) {  // 0 is the background
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < cfg.min_blob_pixels) {
      continue;
    }
    const cv::Rect box(
      stats.at<int>(label, cv::CC_STAT_LEFT), stats.at<int>(label, cv::CC_STAT_TOP),
      stats.at<int>(label, cv::CC_STAT_WIDTH), stats.at<int>(label, cv::CC_STAT_HEIGHT));

    Obstacle ob{box, std::numeric_limits<double>::infinity(), 0.0, 0.0, area, false};
    for (int v = box.y; v < box.y + box.height; ++v) {
      const int * lrow = labels.ptr<int>(v);
      const float * drow = depth.ptr<float>(v);
      for (int u = box.x; u < box.x + box.width; ++u) {
        if (lrow[u] != label) {
          continue;
        }
        const Point3 p = back_project(u, v, drow[u], cam);
        if (p.forward < ob.range_m) {
          ob.range_m = p.forward;
          ob.lateral_m = p.lateral;
        }
        ob.top_height_m = std::max(ob.top_height_m, p.height);
      }
    }
    ob.below_lidar_plane = ob.top_height_m < cfg.lidar_plane_height_m;
    found.push_back(ob);
  }
  std::sort(
    found.begin(), found.end(),
    [](const Obstacle & a, const Obstacle & b) {return a.range_m < b.range_m;});
  return found;
}

}  // namespace factoryrunner_vision
