// ROS2
#include "rclcpp/rclcpp.hpp"

// String msg
#include "std_msgs/msg/string.hpp"
// Float32MultiArray
#include "std_msgs/msg/float32_multi_array.hpp"
// PointCloud2
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl/conversions.h"
#include "pcl/filters/passthrough.h"

// CoppeliaSim remote zmq API
#include <RemoteAPIClient.h>

// Other
#include <iostream>

#define M_PI 3.14159265358979323846
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

RemoteAPIClient client_;
auto sim_ = client_.getObject().sim();

class RawToPointCloud2 : public rclcpp::Node {
 public:
  RawToPointCloud2() : Node("float32multiarray_to_pointcloud2") {
    // Get parameters
    this->declare_parameter<std::string>("handle_name", "kinect/depth");
    this->declare_parameter<std::string>("input_topic", "cloud_raw");
    this->declare_parameter<std::string>("output_topic", "cloud_out");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<float>("near_clip", 0.02);
    this->declare_parameter<float>("far_clip", 3.5);
    this->declare_parameter<float>("view_angle", 57);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("width", 480);
    this->declare_parameter<bool>("noise", true);
    this->declare_parameter<bool>("color", false);
    this->declare_parameter<int>("R", 255);
    this->declare_parameter<int>("G", 0);
    this->declare_parameter<int>("B", 0);

    this->get_parameter("handle_name", handle_name_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("near_clip", near_clip_);
    this->get_parameter("far_clip", far_clip_);
    this->get_parameter("view_angle", view_angle_);
    this->get_parameter("height", height_);
    this->get_parameter("width", width_);
    this->get_parameter("width", width_);
    this->get_parameter("noise", noise_);
    this->get_parameter("color", color_);
    this->get_parameter("R", R);
    this->get_parameter("G", G);
    this->get_parameter("B", B);

    // assert 0 < color <255
    if (color_) {
      assert(0 <= R && R <= 255 && "R value must be between 0 and 255");
      assert(0 <= G && G <= 255 && "G value must be between 0 and 255");
      assert(0 <= B && B <= 255 && "B value must be between 0 and 255");
    }

    RCLCPP_INFO(this->get_logger(), "Handle name: %s", handle_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", width_, height_);

    // init handle for the depth sensor readings
    handle_ = sim_.getObject(handle_name_);

    // Set up publisher
    pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&RawToPointCloud2::chatterCallback, this));

    scale_ = (far_clip_ - near_clip_) / 1.0;
    // unsigned int datalen = height_ * width_;
    float view_angle_r = degreesToRadians(view_angle_);
    f_ = float(float(std::max(height_, width_)) / 2) /
         float(tan(view_angle_r / 2));
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  }

 private:
  void chatterCallback() {
    auto time = this->now();
    depth_res = sim_.getVisionSensorDepth(handle_);

    float f;
    // output on file as integer composed by 4 bytes
    for (size_t i = 0; i < std::get<0>(depth_res).size(); i += 4) {
      // use memcpy to convert 4 bytes to float
      memcpy(&f, &std::get<0>(depth_res)[i], sizeof(f));
      depth_raw.push_back(f);
    }

    // depth_raw = sim_.unpackFloatTable(std::get<0>(depth_res));

    for (int j = 0; j < height_; j++) {
      y = (j - height_ / 2.0);
      for (int i = 0; i < width_; i++) {
        k = j * width_ + i;
        x = -(i - width_ / 2.0);
        x_scale.push_back(float(x / f_));
        y_scale.push_back(float(y / f_));

        depth = near_clip_ + scale_ * depth_raw[k];

        p.x = depth * x_scale[k];
        p.y = depth * y_scale[k];
        p.z = depth;
        p.r = 0;
        p.g = 0;
        p.b = 0;
        cloud->points.push_back(p);
      }
    }
    depth_raw.clear();
    x_scale.clear();
    y_scale.clear();
    // add Gaussian noise
    if (noise_ || color_) {
      for (size_t i = 0; i < cloud->points.size(); i++) {
        if (color_) {
          cloud->points[i].r = R;
          cloud->points[i].g = G;
          cloud->points[i].b = B;
        }
        if (noise_) {
          cloud->points[i].x += 0.007 * ((float)rand() / RAND_MAX - 0.5);
          cloud->points[i].y += 0.007 * ((float)rand() / RAND_MAX - 0.5);
          cloud->points[i].z += 0.007 * ((float)rand() / RAND_MAX - 0.5);
        }
      }
    }

    pcl::toROSMsg(*cloud.get(), output_);
    output_.header.frame_id = frame_id_;
    // get simulation time
    output_.header.stamp = time;
    pub_->publish(output_);
    cloud->clear();
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 output_;

  std::string handle_name_, input_topic_, output_topic_, frame_id_;
  int height_, width_;
  float near_clip_, far_clip_, view_angle_, scale_, f_;
  bool noise_, color_;
  int R = 0, G = 0, B = 0;
  long int handle_;
  // utils
  float k, x, y, z, depth;
  float xyz[3];
  std::vector<float> x_scale, y_scale;
  pcl::PointXYZRGB p;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::vector<double> depth_raw;
  std::tuple<std::vector<uint8_t>, std::vector<int64_t>> depth_res;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RawToPointCloud2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
