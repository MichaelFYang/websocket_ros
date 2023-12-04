# SocketIO  modules
import socketio
import eventlet
import numpy as np
import json
import time

# ROS modules
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from shape_msgs.msg import Mesh
from scipy.spatial.transform import Rotation

sio = socketio.Server(cors_allowed_origins='*')
app = socketio.WSGIApp(sio)

class RosDataCommunicator:
    def __init__(self, port=5000):
        self.port = port
        eventlet.monkey_patch()  # Important for concurrency

    def worker1(self):
        eventlet.wsgi.server(eventlet.listen(('', self.port)), app)

    def send_update(self, data):
        # Convert ROS data to JSON and emit via socket
        json_data = json.dumps(data)  # Ensure data is serializable
        sio.emit('ros_data_update', json_data)

    def start(self):
        self.worker1()

class SocketRosNode:
    def __init__(self, data_communicator):
        self.data_communicator = data_communicator
        # Initialize the ROS node
        rospy.init_node('socket_ros_node', anonymous=True)

        # Initialize a dictionary to store message data
        self.data = {"odom": None, "point_cloud": None, "mesh": None}

        # Create subscribers
        self.point_cloud_sub = rospy.Subscriber('point_cloud', PointCloud2, self.point_cloud_callback)
        self.odometry_sub = rospy.Subscriber('robot_odom', Odometry, self.odometry_callback)
        self.mesh_sub = rospy.Subscriber('elevation_map', Mesh, self.mesh_callback)
        
    def send_data_update(self):
        # Call this method to send updated data
        if self.data_communicator:
            self.data_communicator.send_update(self.data)
        
    def generate_synthetic_data(self):
        # Generate synthetic point cloud data (N, 3)
        N = 100  # Number of points
        self.data["point_cloud"] = np.random.rand(N, 3)

        # Generate synthetic odometry data (4x4 transformation matrix)
        random_translation = np.random.rand(3)
        random_quaternion = Rotation.random().as_quat()
        random_rotation_matrix = Rotation.from_quat(random_quaternion).as_matrix()

        synthetic_odom = np.eye(4)
        synthetic_odom[:3, :3] = random_rotation_matrix
        synthetic_odom[:3, 3] = random_translation
        self.data["odom"] = synthetic_odom

        # Generate synthetic mesh data (N2, 3)
        N2 = 50  # Number of vertices
        self.data["mesh"] = np.random.rand(N2, 3)

        rospy.loginfo("Generated synthetic data")

    # def point_cloud_callback(self, data):
    #     # Convert PointCloud2 to numpy array (N, 3)
    #     pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    #     self.data["point_cloud"] = pc_np
    #     rospy.loginfo("Updated point cloud data")

    # def odometry_callback(self, data):
    #     # Convert Odometry to 4x4 transformation matrix using scipy
    #     position = data.pose.pose.position
    #     orientation = data.pose.pose.orientation
    #     translation = np.array([position.x, position.y, position.z])
    #     quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
    #     # Create a rotation object from the quaternion and convert it to a matrix
    #     rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

    #     # Create the transformation matrix
    #     transformation_matrix = np.eye(4)
    #     transformation_matrix[:3, :3] = rotation_matrix
    #     transformation_matrix[:3, 3] = translation

    #     self.data["odom"] = transformation_matrix
    #     rospy.loginfo("Updated odometry data")

    # def mesh_callback(self, data):
    #     # Convert Mesh to numpy array of vertices (N2, 3)
    #     vertices = np.array([[vertex.x, vertex.y, vertex.z] for vertex in data.vertices])
    #     self.data["mesh"] = vertices
    #     rospy.loginfo("Updated mesh data")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    communicator = RosDataCommunicator()
    ros_node = SocketRosNode(communicator)
    # Start ROS node in a separate thread or as per your architecture
    # Start communicator
    communicator.start()
    
    while True:
        ros_node.generate_synthetic_data()
        ros_node.send_data_update()
        time.sleep(1)