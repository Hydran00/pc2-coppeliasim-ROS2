// ROS2
#include "rclcpp/rclcpp.hpp"

// String msg
#include "std_msgs/msg/string.hpp"
// Float32MultiArray
#include "std_msgs/msg/float32_multi_array.hpp"
// PointCloud2
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include "pcl/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/filters/passthrough.h"

// Other
#include <iostream>

#define M_PI 3.14159265358979323846
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

class RawToPointCloud2 : public rclcpp::Node
{
public:
  RawToPointCloud2()
      : Node("float32multiarray_to_pointcloud2")
  {
    // Get parameters
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
    if (color_)
    {
      assert(0 <= R && R <= 255 && "R value must be between 0 and 255");
      assert(0 <= G && G <= 255 && "G value must be between 0 and 255");
      assert(0 <= B && B <= 255 && "B value must be between 0 and 255");
    }
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", width_, height_);

    // Set up publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);

    // Set up subscriber
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        input_topic_, 1, std::bind(&RawToPointCloud2::chatterCallback, this, std::placeholders::_1));

    scale_ = (far_clip_ - near_clip_) / 1.0;
    // unsigned int datalen = height_ * width_;
    float view_angle_r = degreesToRadians(view_angle_);
    f_ = float(float(std::max(height_, width_)) / 2) / float(tan(view_angle_r / 2));
  }

private:
  void chatterCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> depth_raw = msg->data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int j = 0; j < height_; j++)
    {
      float y = (j - height_ / 2.0);
      for (int i = 0; i < width_; i++)
      {
        int k = j * width_ + i;
        float x = -(i - width_ / 2.0);
        x_scale.push_back(float(x / f_));
        y_scale.push_back(float(y / f_));

        float depth = near_clip_ + scale_ * depth_raw[k];
        float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
        pcl::PointXYZRGB p;

        p.x = xyz[0];
        p.y = xyz[1];
        p.z = xyz[2];
        p.r = 0;
        p.g = 0;
        p.b = 0;
        cloud->points.push_back(p);
      }
    }

    // add Gaussian noise
    if (noise_ || color_)
    {
      for (size_t i = 0; i < cloud->points.size(); i++)
      {
        if (color_)
        {
          cloud->points[i].r = R;
          cloud->points[i].g = G;
          cloud->points[i].b = B;
        }
        if (noise_)
        {
          cloud->points[i].x += 0.015 * ((float)rand() / RAND_MAX - 0.5);
          cloud->points[i].y += 0.015 * ((float)rand() / RAND_MAX - 0.5);
          cloud->points[i].z += 0.015 * ((float)rand() / RAND_MAX - 0.5);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud.get(), output);
    output.header.frame_id = frame_id_;
    // get simulation time
    output.header.stamp = this->now();
    pub_->publish(output);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  std::string input_topic_, output_topic_, frame_id_;
  int height_, width_;
  float near_clip_, far_clip_, view_angle_, scale_, f_;
  std::vector<float> x_scale, y_scale;
  bool noise_, color_;
  int R = 0, G = 0, B = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RawToPointCloud2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
