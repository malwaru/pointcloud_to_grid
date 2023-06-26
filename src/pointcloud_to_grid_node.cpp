#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/PointCloud2.hpp>
#include <nav_msgs/msg/OccupancyGrid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <pointcloud_to_grid/MyParamsConfig.h>
#include <rclcpp/dynamic_parameters.hpp>

class PointcloudToGridNode : public rclcpp::Node
{
public:
  PointcloudToGridNode()
    : Node("pointcloud_to_grid_node")
  {
    // Create publisher for intensity grid
    pub_igrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map_.mapi_topic_name, 1);

    // Create publisher for height grid
    pub_hgrid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map_.maph_topic_name, 1);

    // Create subscriber for point cloud
    sub_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      grid_map_.cloud_in_topic, 1, std::bind(&PointcloudToGridNode::pointcloudCallback, this, std::placeholders::_1));

    // Create dynamic reconfigure server
    dyn_params_server_ = std::make_shared<rclcpp::DynamicParametersServer>(this);
    dyn_params_server_->add_parameter_callback(
      std::bind(&PointcloudToGridNode::paramsCallback, this, std::placeholders::_1));

    // Initialize grid
    grid_map_.initGrid(intensity_grid_);
    grid_map_.initGrid(height_grid_);
  }

private:
  void paramsCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &param : parameters)
    {
      if (param.get_name() == "cell_size")
        grid_map_.cell_size = param.as_double();
      else if (param.get_name() == "position_x")
        grid_map_.position_x = param.as_double();
      else if (param.get_name() == "position_y")
        grid_map_.position_y = param.as_double();
      else if (param.get_name() == "length_x")
        grid_map_.length_x = param.as_double();
      else if (param.get_name() == "length_y")
        grid_map_.length_y = param.as_double();
      else if (param.get_name() == "intensity_factor")
        grid_map_.intensity_factor = param.as_double();
      else if (param.get_name() == "height_factor")
        grid_map_.height_factor = param.as_double();
      else if (param.get_name() == "mapi_topic_name")
        grid_map_.mapi_topic_name = param.as_string();
      else if (param.get_name() == "maph_topic_name")
        grid_map_.maph_topic_name = param.as_string();
      else if (param.get_name() == "cloud_in_topic")
        grid_map_.cloud_in_topic = param.as_string();
    }

    // Refresh parameters
    grid_map_.paramRefresh();
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Initialize grid vectors
    std::vector<signed char> hpoints(grid_map_.cell_num_x * grid_map_.cell_num_y, -128);
    std::vector<signed
