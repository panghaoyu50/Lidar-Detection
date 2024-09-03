import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class ClusteringNode(Node):
    def __init__(self):
        super().__init__('clustering_node')

		# Suscribe to the /ground_removal topic to receive a PointClound
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ground_removal',
            self.point_cloud_callback,
            10)
        
        # Publish the filtered PointClound to the /clusters topic
        self.publisher = self.create_publisher(PointCloud2, '/clusters', 10)

        # Parameters for PointCloud.cluster_dbscan()
        self.eps = 0.02
        self.min_points = 10


    def point_cloud_callback(self, msg):
    	# Read PointCloud message to [(x,y,z)...] format
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [list(point) for point in cloud_points]

        # Create a open3d PointCloud and add points to the PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Perform DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(self.eps, self.min_points))

        # Find unique cluster IDs
        unique_labels = np.unique(labels)

        # Publish each cluster to the /clusters topic
        for label in unique_labels:
            if label != -1:
                # Extract indices of points belonging to the current cluster
                indices = np.where(labels == label)[0]
                
                # Create a new point cloud for the current cluster
                cluster_pcd = pcd.select_by_index(indices)
                cluster_points = np.asarray(cluster_pcd.points)
                cluster_msg = pc2.create_cloud_xyz32(msg.header, cluster_points)

                self.publisher.publish(cluster_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
