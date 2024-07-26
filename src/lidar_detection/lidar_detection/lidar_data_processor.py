import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('point_cloud_filter_node')

		# Suscribe to the /velodyne_points topic to receive a PointClound
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.point_cloud_callback,
            10)

        # Publish the filtered PointClound to the /filtered_point_cloud topic
        self.publisher = self.create_publisher(PointCloud2, '/filtered_point_cloud', 10)

        # Define bounding box with respect to Lidar origin
        # width: 4, length 5, height 2 meters box
        self.min_bound = np.array([-2, 0, -1])
        self.max_bound = np.array([2, 5, 1])

    def point_cloud_callback(self, msg):
    	# Read PointCloud message to [(x,y,z)...] format
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

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
