#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

class PointCloudCropNode : public rclcpp::Node
{
public:
  PointCloudCropNode()
  : Node("pointcloud_crop_node")
  {
    // 파라미터로 박스 경계 설정
    this->declare_parameter("min_x", -1.85);
    this->declare_parameter("min_y", -0.6);
    this->declare_parameter("min_z", -1.5);
    this->declare_parameter("max_x", 0.15);
    this->declare_parameter("max_y", 0.6);
    this->declare_parameter("max_z", 1.0);

    // 입력 및 출력 토픽 설정
    this->declare_parameter("input_topic", "/points");
    this->declare_parameter("output_topic", "/points_modified");

    // 파라미터 가져오기
    this->get_parameter("min_x", min_x_);
    this->get_parameter("min_y", min_y_);
    this->get_parameter("min_z", min_z_);
    this->get_parameter("max_x", max_x_);
    this->get_parameter("max_y", max_y_);
    this->get_parameter("max_z", max_z_);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);

    // Subscriber 및 Publisher 생성
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&PointCloudCropNode::pointcloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "PointCloudCropNode has been started.");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ROS 메시지를 PCL 포맷으로 변환
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *cloud);

    // CropBox 필터 설정
    pcl::CropBox<pcl::PCLPointCloud2> box_filter;
    box_filter.setInputCloud(cloud);

    Eigen::Vector4f min_point(min_x_, min_y_, min_z_, 1.0);
    Eigen::Vector4f max_point(max_x_, max_y_, max_z_, 1.0);

    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.setNegative(true); // 박스 내부 포인트 제거

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    box_filter.filter(*cloud_filtered);

    // 필터링된 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::moveFromPCL(*cloud_filtered, output);
    output.header = msg->header; // 헤더 정보 복사

    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;

  float min_x_, min_y_, min_z_;
  float max_x_, max_y_, max_z_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudCropNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
