# SocketIO  modules
import socketio
import eventlet
import json
import time
import numpy as np
import threading

# ROS modules
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from shape_msgs.msg import Mesh
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger, TriggerResponse


class RosDataCommunicator:
    def __init__(self, port=5030):
        self.port = port
        self.socket = socketio.Server(async_mode='eventlet')
        self.app = socketio.WSGIApp(self.socket)
        
        # set up thread
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.worker1)
        self._thread.daemon = True

    def worker1(self):
        try:
            eventlet.wsgi.server(eventlet.listen(('', self.port)), self.app)
        except Exception as e:
            print(f"Server failed to start: {e}")

    def send_update(self, data):
        # Convert ROS data to JSON and emit via socket
        self.socket.emit('point_cloud', json.dumps(data))
        print("sent data")

    def start(self):
        self._thread.start()
        
    def stop(self):
        self.socket.shutdown()

class SocketRosNode:
    def __init__(self, data_communicator):
        self.data_communicator = data_communicator
        # Initialize the ROS node
        rospy.init_node('socket_ros_node', anonymous=True)

        # Initialize a dictionary to store message data
        self.data = {"odom": None, "point_cloud": None, "mesh": None}
        
        # Init test data
        self.data_test = {"data": None}

        # Create subscribers
        self.point_cloud_sub = rospy.Subscriber('point_cloud', PointCloud2, self.point_cloud_callback)
        self.odometry_sub = rospy.Subscriber('robot_odom', Odometry, self.odometry_callback)
        self.mesh_sub = rospy.Subscriber('elevation_map', Mesh, self.mesh_callback)
        
        # Initialize a service server
        self.service = rospy.Service('generate_synthetic_data', Trigger, self.handle_generate_synthetic_data)
        
        # print statrting message
        rospy.loginfo("Socket ROS node started, starting data communicator... \n")
        self.data_communicator.start()

    @staticmethod
    def pointcloud2_to_xyz_array(cloud_msg):
        # Determine the type and offset of each field
        dtype_list = []
        for field in cloud_msg.fields:
            if field.name in ['x', 'y', 'z']:
                dtype = np.float32
                offset = field.offset
                dtype_list.append((field.name, dtype, offset))

        # Create a numpy structured array using the dtype list
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=dtype_list, offset=cloud_msg.point_step)

        # Extract the 'x', 'y', 'z' fields and stack them into a (N, 3) array
        x = cloud_arr['x']
        y = cloud_arr['y']
        z = cloud_arr['z']
        xyz_array = np.column_stack((x, y, z))

        return xyz_array
    
    def handle_generate_synthetic_data(self, request):
        self.generate_synthetic_test_data()
        # send data update
        print("send data update")
        self.send_data_update()
        return TriggerResponse(
            success=True,
            message="Synthetic data generated successfully"
        )
        
    def send_data_update(self):
        # Call this method to send updated data
        if self.data_communicator:
            self.data_communicator.send_update(self.data_test)
        
    def generate_synthetic_data(self):
        # Generate synthetic point cloud data
        N = 100  # Number of points
        point_cloud_np = np.random.rand(N, 3)
        self.data["point_cloud"] = point_cloud_np.tolist()  # Convert to list of lists

        # Generate synthetic odometry data
        random_translation = np.random.rand(3)
        random_quaternion = np.random.rand(4)  # For example purposes
        random_rotation_matrix = np.identity(4)  # Replace with actual computation
        synthetic_odom = np.eye(4)
        synthetic_odom[:3, :3] = random_rotation_matrix[:3, :3]
        synthetic_odom[:3, 3] = random_translation
        self.data["odom"] = synthetic_odom.tolist()  # Convert to list of lists

        # Generate synthetic mesh data
        N2 = 50  # Number of vertices
        mesh_np = np.random.rand(N2, 3)
        self.data["mesh"] = mesh_np.tolist()  # Convert to list of lists
        
    def generate_synthetic_test_data(self):
        # Generate synthetic point cloud data
        num_points = 1000
        radius = 0.2
        # Generate uniform angles
        theta = np.random.uniform(0, 2*np.pi, num_points)
        phi = np.arccos(np.random.uniform(-1, 1, num_points))

        # Convert spherical coordinates to Cartesian coordinates and scale by radius
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)

        # Combine x, y, z into a single array and round
        points = np.stack((x, y, z), axis=-1)
        points = np.round(points, 3)
        
        # flatten to list
        points = points.flatten()
        
        self.data_test["data"] = points.tolist()  # Convert to list of lists
        

    def point_cloud_callback(self, data):
        # Convert PointCloud2 to numpy array (N, 3)
        pc_np = SocketRosNode.pointcloud2_to_xyz_array(data)
        self.data["point_cloud"] = pc_np
        rospy.loginfo("Updated point cloud data")

    def odometry_callback(self, data):
        # Convert Odometry to 4x4 transformation matrix using scipy
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        translation = np.array([position.x, position.y, position.z])
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Create a rotation object from the quaternion and convert it to a matrix
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

        # Create the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        self.data["odom"] = transformation_matrix
        rospy.loginfo("Updated odometry data")

    def mesh_callback(self, data):
        # Convert Mesh to numpy array of vertices (N2, 3)
        vertices = np.array([[vertex.x, vertex.y, vertex.z] for vertex in data.vertices])
        self.data["mesh"] = vertices
        rospy.loginfo("Updated mesh data")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    print("Starting ROS node")
    communicator = RosDataCommunicator()
    ros_node = SocketRosNode(communicator)
    
    while not rospy.is_shutdown():
        print("loop")
        time.sleep(1)
        
    communicator.stop()