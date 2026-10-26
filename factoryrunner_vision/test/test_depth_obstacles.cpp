#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "factoryrunner_vision/depth_obstacles.hpp"

using factoryrunner_vision::CameraModel;
using factoryrunner_vision::DetectorConfig;
using factoryrunner_vision::detect;

namespace
{

// Roughly the Waffle's sensor: 320x240, 60 degree horizontal field of view, ~0.2 m up.
const CameraModel kCam{277.1, 277.1, 160.0, 120.0, 0.2};

// Flat floor everywhere below the horizon, nothing above it.
cv::Mat floor_image()
{
  cv::Mat depth(240, 320, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
  for (int v = static_cast<int>(kCam.cy) + 1; v < depth.rows; ++v) {
    depth.row(v).setTo(static_cast<float>(kCam.height_m * kCam.fy / (v - kCam.cy)));
  }
  return depth;
}

// Paint the front face of a box at `range`, from the floor up to `top`, between two lateral
// offsets (left positive).
void add_box(cv::Mat & depth, double range, double top, double left, double right)
{
  const int v_top = static_cast<int>(std::ceil(kCam.cy + (kCam.height_m - top) * kCam.fy / range));
  const int v_bottom = static_cast<int>(kCam.cy + kCam.height_m * kCam.fy / range);
  const int u_left = static_cast<int>(std::ceil(kCam.cx - left * kCam.fx / range));
  const int u_right = static_cast<int>(kCam.cx - right * kCam.fx / range);
  for (int v = std::max(v_top, 0); v <= std::min(v_bottom, depth.rows - 1); ++v) {
    for (int u = std::max(u_left, 0); u <= std::min(u_right, depth.cols - 1); ++u) {
      depth.at<float>(v, u) = static_cast<float>(range);
    }
  }
}

}  // namespace

TEST(DepthObstacles, FloorAloneIsNotAnObstacle)
{
  EXPECT_TRUE(detect(floor_image(), kCam, DetectorConfig{}).empty());
}

TEST(DepthObstacles, LowBoxIsFoundAndFlaggedAsUnderTheLidar)
{
  cv::Mat depth = floor_image();
  add_box(depth, 1.5, 0.10, 0.15, -0.15);
  const auto found = detect(depth, kCam, DetectorConfig{});
  ASSERT_EQ(found.size(), 1u);
  EXPECT_NEAR(found[0].range_m, 1.5, 1e-3);
  EXPECT_NEAR(found[0].top_height_m, 0.10, 0.02);
  EXPECT_TRUE(found[0].below_lidar_plane);
}

TEST(DepthObstacles, TallBoxIsNotFlagged)
{
  cv::Mat depth = floor_image();
  add_box(depth, 2.0, 0.5, 0.25, -0.25);
  const auto found = detect(depth, kCam, DetectorConfig{});
  ASSERT_EQ(found.size(), 1u);
  EXPECT_GT(found[0].top_height_m, 0.17);
  EXPECT_FALSE(found[0].below_lidar_plane);
}

TEST(DepthObstacles, ObstacleOutsideTheCorridorIsIgnored)
{
  cv::Mat depth = floor_image();
  add_box(depth, 1.5, 0.3, 1.2, 0.8);
  EXPECT_TRUE(detect(depth, kCam, DetectorConfig{}).empty());
}

TEST(DepthObstacles, ClosestObstacleComesFirstAndNanIsSkipped)
{
  cv::Mat depth = floor_image();
  add_box(depth, 3.0, 0.3, 0.3, 0.1);
  add_box(depth, 1.2, 0.3, -0.1, -0.3);
  depth.at<float>(200, 10) = std::numeric_limits<float>::quiet_NaN();
  const auto found = detect(depth, kCam, DetectorConfig{});
  ASSERT_EQ(found.size(), 2u);
  EXPECT_NEAR(found[0].range_m, 1.2, 1e-3);
  EXPECT_NEAR(found[1].range_m, 3.0, 1e-3);
}
