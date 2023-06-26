from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld =LaunchDescription()

    # Start the tracker


    pointcloud_to_grid=Node(
        package="pointcloud_to_grid",
        executable="pointcloud_to_grid_node.py",
        namespace="pointcloud_to_grid"       
                        )
    

    ld.add_action(pointcloud_to_grid)
  


    return ld