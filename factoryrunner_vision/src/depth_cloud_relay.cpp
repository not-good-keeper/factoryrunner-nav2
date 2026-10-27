// Republishes the depth camera's point cloud for the costmaps: thinned out, NaN-free and in a
// frame that matches where the sensor really is.
//
//   ros2 run factoryrunner_vision depth_cloud_relay
//
// Gazebo publishes the cloud x-forward from the sensor (about 0.23 m up), but labels it
// camera_depth_frame, which the TurtleBot3 description places lower and as an optical frame.
// Taken at face value, every point would land rotated and too low. The relay re-labels the
// cloud with output_frame (default depth_sensor), which launch/sim.launch.py publishes at the
// measured sensor pose, and keeps every stride-th pixel in each direction.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class DepthCloudRelay : public rclcpp::Node
{
public:
  DepthCloudRelay()
  : Node("depth_cloud_relay")
  {
    frame_ = declare_parameter("output_frame", std::string("depth_sensor"));
    stride_ = std::max<int64_t>(1, declare_parameter("stride", 4));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points_costmap", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {relay(*msg);});
  }

private:
  void relay(const sensor_msgs::msg::PointCloud2 & in)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = in.header.stamp;
    out.header.frame_id = frame_;
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize((in.width / stride_ + 1) * (in.height / stride_ + 1));

    sensor_msgs::PointCloud2ConstIterator<float> ix(in, "x"), iy(in, "y"), iz(in, "z");
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    size_t kept = 0;
    for (uint32_t row = 0; row < in.height; row += stride_) {
      for (uint32_t col = 0; col < in.width; col += stride_) {
        const size_t offset = static_cast<size_t>(row) * in.width + col;
        const float x = *(ix + offset), y = *(iy + offset), z = *(iz + offset);
        // Gazebo marks pixels with no return as +/-inf; the costmaps would choke on them.
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        *(ox + kept) = x;
        *(oy + kept) = y;
        *(oz + kept) = z;
        ++kept;
      }
    }
    modifier.resize(kept);
    pub_->publish(out);
  }

  std::string frame_;
  int64_t stride_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthCloudRelay>());
  rclcpp::shutdown();
  return 0;
}
