# Server
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
from std_srvs.srv import Trigger, TriggerResponse

socket = socketio.Server(async_mode='eventlet')
app = socketio.WSGIApp(socket)

data_test_ = {"data": None}
data_ = {"odom": None, "point_cloud": None, "mesh": None}

class SocketRosNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('socket_ros_node', disable_signals=True)

        # Create subscribers
        self.point_cloud_sub = rospy.Subscriber('point_cloud', PointCloud2, self.point_cloud_callback)
        self.odometry_sub = rospy.Subscriber('robot_odom', Odometry, self.odometry_callback)
        self.mesh_sub = rospy.Subscriber('elevation_map', Mesh, self.mesh_callback)
        
        # Initialize a service server
        self.service = rospy.Service('generate_synthetic_data', Trigger, self.handle_generate_synthetic_data)
        
        # print statrting message
        rospy.loginfo("Socket ROS node started, starting data communicator... \n")

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
        return TriggerResponse(
            success=True,
            message="Synthetic data generated successfully"
        )
        
    def generate_synthetic_data(self):
        # Generate synthetic point cloud data
        N = 100  # Number of points
        point_cloud_np = np.random.rand(N, 3)
        data_["point_cloud"] = point_cloud_np.tolist()  # Convert to list of lists

        # Generate synthetic odometry data
        random_translation = np.random.rand(3)
        random_rotation_matrix = np.identity(4)  # Replace with actual computation
        synthetic_odom = np.eye(4)
        synthetic_odom[:3, :3] = random_rotation_matrix[:3, :3]
        synthetic_odom[:3, 3] = random_translation
        data_["odom"] = synthetic_odom.tolist()  # Convert to list of lists

        # Generate synthetic mesh data
        N2 = 50  # Number of vertices
        mesh_np = np.random.rand(N2, 3)
        data_["mesh"] = mesh_np.tolist()  # Convert to list of lists
        
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
        
        data_test_["data"] = points.tolist()  # Convert to list of lists
        

    def point_cloud_callback(self, data):
        # Convert PointCloud2 to numpy array (N, 3)
        pc_np = SocketRosNode.pointcloud2_to_xyz_array(data)
        data_["point_cloud"] = pc_np
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

        data_["odom"] = transformation_matrix
        rospy.loginfo("Updated odometry data")

    def mesh_callback(self, data):
        # Convert Mesh to numpy array of vertices (N2, 3)
        vertices = np.array([[vertex.x, vertex.y, vertex.z] for vertex in data.vertices])
        data_["mesh"] = vertices
        rospy.loginfo("Updated mesh data")

def worker1():
    eventlet.wsgi.server(eventlet.listen(('', 5030)), app)


# def generate_points(num_points, radius=1.0):
#     # Generate uniform angles
#     theta = np.random.uniform(0, 2*np.pi, num_points)
#     phi = np.arccos(np.random.uniform(-1, 1, num_points))

#     # Convert spherical coordinates to Cartesian coordinates and scale by radius
#     x = radius * np.sin(phi) * np.cos(theta)
#     y = radius * np.sin(phi) * np.sin(theta)
#     z = radius * np.cos(phi)

#     # Combine x, y, z into a single array and round
#     points = np.stack((x, y, z), axis=-1)
#     points = np.round(points, 3)
    
#     # flatten
#     points = points.flatten()

#     return points.tolist()

@socket.on('target_pose')  # Listening for 'client_message' event
def handle_client_message(sid, message):
    pose = decode_transformation_matrix(message)
    print(f"Received message from client {sid}")
    print(pose)
    

def decode_transformation_matrix(json_string):
    # Parse the JSON string
    data = json.loads(json_string)

    # Extract the matrix values
    matrix_values = data['poseTarget']

    # Reshape the values into a 4x4 matrix
    transformation_matrix = np.array(matrix_values).reshape(4, 4)

    return transformation_matrix

def worker2(data):
    while(1):
        # data = {"data": generate_points(1000, radius=0.2)}
        if data["data"] is not None:
            print("send msg")
            socket.emit('point_cloud', json.dumps(data))
            data["data"] = None
        socket.sleep(1)

def main():
    # ros_thread = eventlet.spawn(SocketRosNode)
    SocketRosNode()
    socket.start_background_task(worker2, data_test_)
    worker1()

if __name__ == '__main__':
    main()
    rospy.signal_shutdown("Shutting down ROS node")