#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <tf2_ros/buffer.h>


using namespace std;

class PointCloudToGrid : public rclcpp::Node
{

    public:
        PointCloudToGrid() : Node("pointcloud_to_grid")
        {
            // Create a publisher on the topic "grid_map" that will publish a nav_msgs::msg::OccupancyGrid message
            grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
            
            // Create a subscriber on the topic "pointcloud" that will call the callback function "PointCloudCallback" when a message is received
            pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 10, std::bind(&PointCloudToGrid::PointCloudCallback, this, std::placeholders::_1));
            
            // Create a timer that will call the callback function "TimerCallback" every 1000 milliseconds
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PointCloudToGrid::TimerCallback, this));
        }

    private:

        void PointCloudCallback{

            
        }
};


int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<PointCloudToGrid>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}