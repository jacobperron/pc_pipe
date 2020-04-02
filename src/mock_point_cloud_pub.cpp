#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


class MockPointCloudPublisher : public rclcpp::Node
{
public:
  explicit MockPointCloudPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mock_point_cloud_publisher", options)
  {
    using namespace std::placeholders;
    approximate_sync_.reset(
      new ApproximateSync(
        ApproximatePolicy(10),
        left_image_sub_,
        left_camera_info_sub_,
        right_image_sub_,
        right_camera_info_sub_));
    approximate_sync_->registerCallback(
      std::bind(&MockPointCloudPublisher::image_callback, this, _1, _2, _3, _4));

    image_transport::TransportHints hints(this, "raw");
    const auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    left_image_sub_.subscribe(this, "left/image_filtered", hints.getTransport(), qos);
    left_camera_info_sub_.subscribe(this, "left/camera_info", qos);
    right_image_sub_.subscribe(this, "right/image_filtered", hints.getTransport(), qos);
    right_camera_info_sub_.subscribe(this, "right/camera_info", qos);

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);
  }

private:
  void image_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr & left_image,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info,
      const sensor_msgs::msg::Image::ConstSharedPtr & right_image,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & right_info)
  {
    auto now = this->now();
    std::cerr << "Images received. Delta: " << (now - rclcpp::Time(left_image->header.stamp)).seconds() << std::endl;
    (void)left_info;
    (void)right_image;
    (void)right_info;

    if (points_pub_->get_subscription_count() == 0u) {
      return;
    }
    std::cerr << "Publishing." << std::endl;
    auto points = sensor_msgs::msg::PointCloud2();
    points.width = left_image->width;
    points.height = left_image->height;
    for (size_t i = 0; i < left_image->width * left_image->height; ++i) {
      points.data.push_back(i);
    }
    points_pub_->publish(points);
  }

  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_camera_info_sub_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub_;
};  // class MockPointCloudPublisher

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockPointCloudPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
