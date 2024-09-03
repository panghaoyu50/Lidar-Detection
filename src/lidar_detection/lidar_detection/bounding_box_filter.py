import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class BoundingBoxFilterNode(Node):
    def __init__(self):
        super().__init__('bounding_box_filter_node')

		# Suscribe to the /velodyne_points topic to receive a PointClound
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.point_cloud_callback,
            10)

        # Publish the filtered PointClound to the /bounding_box topic
        self.publisher = self.create_publisher(PointCloud2, '/bounding_box', 10)

        # Define bounding box with respect to Lidar origin
        # +ve x: front, +ve y right, +ve z: up
        self.min_bound = np.array([0, -2, -1])
        self.max_bound = np.array([5, 2, 1])

    def point_cloud_callback(self, msg):
    	# Read PointCloud message to [(x,y,z)...] format
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [list(point) for point in cloud_points]

        # Create a open3d PointCloud and add points to the PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

		# Filter out all the points lie outside of the box
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(self.min_bound, self.max_bound)
        cropped_pcd = pcd.crop(bounding_box)

		# Create the filtered PointCloud
        filtered_points = np.asarray(cropped_pcd.points)
        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
