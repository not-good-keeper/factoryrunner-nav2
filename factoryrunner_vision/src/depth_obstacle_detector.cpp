// Watches the Waffle's depth camera for obstacles in the robot's path and reports the ones the
// 2D LiDAR cannot see because they sit below its scan plane. It observes only: nothing here
// feeds Nav2 or commands motion.
//
//   ros2 run factoryrunner_vision depth_obstacle_detector
//
// Publishes /vision/nearest_obstacle (PointStamped in the depth image's frame: x forward,
// y left, z up),
// /vision/obstacles (MarkerArray) and /vision/debug_image (bgr8; red boxes are below the
// LiDAR plane, yellow ones are not).

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "factoryrunner_vision/depth_obstacles.hpp"

namespace factoryrunner_vision
{

class DepthObstacleDetector : public rclcpp::Node
{
public:
  DepthObstacleDetector()
  : Node("depth_obstacle_detector")
  {
    // Measured in the depot world from floor pixels in the bottom rows of live frames
    // (3600 samples, 10th-90th percentile 0.226-0.227 m).
    camera_height_ = declare_parameter("camera_height_m", 0.226);
    cfg_.min_height_m = declare_parameter("min_height_m", cfg_.min_height_m);
    cfg_.max_height_m = declare_parameter("max_height_m", cfg_.max_height_m);
    cfg_.corridor_half_width_m =
      declare_parameter("corridor_half_width_m", cfg_.corridor_half_width_m);
    cfg_.max_range_m = declare_parameter("max_range_m", cfg_.max_range_m);
    cfg_.min_blob_pixels = static_cast<int>(declare_parameter("min_blob_pixels", 30));
    cfg_.lidar_plane_height_m =
      declare_parameter("lidar_plane_height_m", cfg_.lidar_plane_height_m);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {on_info(*msg);});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(msg);});
    nearest_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/vision/nearest_obstacle", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/vision/obstacles", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 10);
  }

private:
  void on_info(const sensor_msgs::msg::CameraInfo & info)
  {
    // k is the row-major 3x3 intrinsic matrix.
    cam_ = CameraModel{info.k[0], info.k[4], info.k[2], info.k[5], camera_height_};
    have_info_ = info.k[0] > 0.0 && info.k[4] > 0.0;
  }

  void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!have_info_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for camera_info");
      return;
    }
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "bad depth image: %s", e.what());
      return;
    }

    const auto found = detect(depth->image, cam_, cfg_);
    publish_nearest(msg->header, found);
    publish_markers(msg->header, found);
    if (debug_pub_->get_subscription_count() > 0) {
      publish_debug(msg->header, depth->image, found);
    }
    report_low_obstacle(found);
  }

  void publish_nearest(const std_msgs::msg::Header & header, const std::vector<Obstacle> & found)
  {
    if (found.empty()) {
      return;
    }
    geometry_msgs::msg::PointStamped p;
    p.header = header;
    p.point.x = found.front().range_m;
    p.point.y = found.front().lateral_m;
    p.point.z = found.front().top_height_m - camera_height_;
    nearest_pub_->publish(p);
  }

  void publish_markers(const std_msgs::msg::Header & header, const std::vector<Obstacle> & found)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int id = 0;
    for (const auto & ob : found) {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "depth_obstacles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose.position.x = ob.range_m + 0.05;
      m.pose.position.y = ob.lateral_m;
      m.pose.position.z = ob.top_height_m / 2.0 - camera_height_;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.1;
      m.scale.y = 0.2;
      m.scale.z = std::max(ob.top_height_m, 0.02);
      m.color.a = 0.8f;
      m.color.r = 1.0f;
      m.color.g = ob.below_lidar_plane ? 0.0f : 0.8f;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      markers.markers.push_back(m);
    }
    markers_pub_->publish(markers);
  }

  void publish_debug(
    const std_msgs::msg::Header & header, const cv::Mat & depth,
    const std::vector<Obstacle> & found)
  {
    cv::Mat finite;
    depth.copyTo(finite);
    cv::patchNaNs(finite, 0.0);
    cv::threshold(finite, finite, cfg_.max_range_m, cfg_.max_range_m, cv::THRESH_TRUNC);
    cv::Mat grey, colour;
    finite.convertTo(grey, CV_8UC1, -255.0 / cfg_.max_range_m, 255.0);  // near is bright
    cv::cvtColor(grey, colour, cv::COLOR_GRAY2BGR);
    for (const auto & ob : found) {
      const cv::Scalar c = ob.below_lidar_plane ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 210, 255);
      cv::rectangle(colour, ob.box, c, 2);
      char label[32];
      std::snprintf(label, sizeof(label), "%.2f m", ob.range_m);
      cv::putText(
        colour, label, ob.box.tl() + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.4, c, 1);
    }
    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", colour).toImageMsg());
  }

  void report_low_obstacle(const std::vector<Obstacle> & found)
  {
    const Obstacle * low = nullptr;
    for (const auto & ob : found) {
      if (ob.below_lidar_plane) {
        low = &ob;
        break;
      }
    }
    if ((low != nullptr) != low_reported_) {
      low_reported_ = low != nullptr;
      if (low) {
        RCLCPP_WARN(
          get_logger(), "obstacle %.2f m ahead, %.2f m tall: below the LiDAR plane",
          low->range_m, low->top_height_m);
      } else {
        RCLCPP_INFO(get_logger(), "no obstacle below the LiDAR plane in view");
      }
    }
  }

  double camera_height_;
  DetectorConfig cfg_;
  CameraModel cam_{0.0, 0.0, 0.0, 0.0, 0.0};
  bool have_info_ = false;
  bool low_reported_ = false;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace factoryrunner_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<factoryrunner_vision::DepthObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
