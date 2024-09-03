import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import math

class ConeSelectorNode(Node):
    def __init__(self):
        super().__init__('cone_selector_node')

		# Suscribe to the /clusters topic to receive a PointClound
        self.subscription = self.create_subscription(
            PointCloud2,
            '/clusters',
            self.point_cloud_callback,
            10)
        
        # Publish the filtered PointClound to the /cones topic
        self.publisher = self.create_publisher(PointCloud2, '/cones', 10)


        # vertical resolution of the LiDAR
        self.r_v = 0

        # horizontal resolution of the LiDAR
        self.r_h =  0

        # width of the cone
        self.w_c = 0

        # height of the cone
        self.h_c = 0

        # E tolerance
        self.tolerance = 100


    def point_cloud_callback(self, msg):
    	# Read PointCloud message to [(x,y,z)...] format
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [list(point) for point in cloud_points]

        # Create a open3d PointCloud and add points to the PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Calculate the expected number of points for a cone
        x, y, z = pcd.get_center()
        D = math.sqrt(x**2 + y**2 + z**2)
        E = 0.5 * (self.h_c / (2 * D * math.tan(self.r_v / 2))) * (self.w_c / (2 * D * math.tan(self.r_h / 2)))

        # Should somehow check the shape as well

        number_of_points = len(pcd.points)

        # Check if the cluster is a cone and publish it to the /cones topic
        if (E - self.tolerance) <= number_of_points <= (E + self.tolerance):
            cluster_points = np.asarray(pcd.points)
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
