import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class GroundRemovalNode(Node):
    def __init__(self):
        super().__init__('ground_removal_node')

		# Suscribe to the /bounding_box topic to receive a PointClound
        self.subscription = self.create_subscription(
            PointCloud2,
            '/bounding_box',
            self.point_cloud_callback,
            10)
        
        # Publish the filtered PointClound to the /ground_removal topic
        self.publisher = self.create_publisher(PointCloud2, '/ground_removal', 10)

        # Parameters for PointCloud.segment_plane()
        self.distance_threshold = 0.01
        self.ransac_n = 3
        self.num_iterations = 1000


    def point_cloud_callback(self, msg):
    	# Read PointCloud message to [(x,y,z)...] format
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [list(point) for point in cloud_points]

        # Create a open3d PointCloud and add points to the PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Segment the plane
        plane_model, inliers = pcd.segment_plane(self.distance_threshold, self.ransac_n, self.num_iterations)

        # Remove the plane
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

		# Create the filtered PointCloud
        filtered_points = np.asarray(outlier_cloud.points)
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
