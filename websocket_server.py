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

commands_ = {"is_pointcloud": False, "is_mesh": False, "target_pose": None}

init_data_ = {"tf_device_to_odom": None, "tf_odom_to_base": None}

class SocketRosNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('socket_ros_node', disable_signals=True)

        # Create subscribers
        self.point_cloud_sub = rospy.Subscriber('point_cloud', PointCloud2, self.point_cloud_callback)
        self.odometry_sub = rospy.Subscriber('robot_odom', Odometry, self.odometry_callback)
        self.mesh_sub = rospy.Subscriber('elevation_map', Mesh, self.mesh_callback)
        
        # set current odom
        self.current_odom = np.eye(4).flatten().tolist()
        
        # Initialize a service server
        self.service = rospy.Service('generate_synthetic_data', Trigger, self.handle_generate_synthetic_data)
        
        # timer event callback to call the service
        rospy.Timer(rospy.Duration(2.0), self.timer_callback)
        
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
        self.generate_synthetic_data()
        # sysnthetic data generated successfully
        return TriggerResponse(
            success=True,
            message="==================================\nSynthetic data generated successfully"
        )
        
    def timer_callback(self, event):
        # make a service call to on service server generate_synthetic_data
        rospy.wait_for_service('generate_synthetic_data')
        try:
            generate_call = rospy.ServiceProxy('generate_synthetic_data', Trigger)
            resp = generate_call()
            print(resp.message)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
    def generate_synthetic_data(self):
        # Generate synthetic point cloud data
        N = 100  # Number of points
        cloud_pose = np.eye(4).flatten().tolist()  # Replace with actual computation
        point_cloud_np = np.random.rand(N, 3).flatten().tolist()  # Replace with actual computation
        cloud_data = {"frame_pose": cloud_pose, "data": point_cloud_np}  # Convert to list of lists
        data_["point_cloud"] = cloud_data  # Convert to list of lists

        # Generate synthetic odometry data
        # random_translation = np.random.rand(3)
        random_translation = np.array([0.0, 0.0, 0.0])
        random_rotation_matrix = np.identity(4)  # Replace with actual computation
        synthetic_odom = np.eye(4)
        synthetic_odom[:3, :3] = random_rotation_matrix[:3, :3]
        synthetic_odom[:3, 3] = random_translation
        self.current_odom = synthetic_odom
        init_data_["tf_odom_to_base"] = synthetic_odom
        
        # convert to list of lists
        odom_data = {"frame_pose": synthetic_odom.flatten().tolist()}
        data_["odom"] = odom_data  # Convert to list of lists

        # Generate synthetic mesh data
        N = 65 # Number of grid dimensions
        mesh_pose = np.eye(4).flatten().tolist()  # Replace with actual computation
        mesh_np = np.random.rand(N, N).flatten().tolist()  # Replace with actual computation
        data_["mesh"] = {"frame_pose": mesh_pose, "data": mesh_np}  # Convert to list of lists
        

    def point_cloud_callback(self, data):
        # Convert PointCloud2 to numpy array (N, 3)
        pc_np = SocketRosNode.pointcloud2_to_xyz_array(data)
        # check if device to odom TF is received
        if init_data_["tf_device_to_odom"] is None:
            rospy.loginfo("No device to odom TF received yet, skipping point cloud data")
            return
        if data.header.frame_id != "base_link":
            rospy.loginfo("Point cloud frame is not base_link, skipping point cloud data")
            return
        cloud_pose = init_data_["tf_device_to_odom"] @ self.current_odom
        # Convert to list of lists
        pc_np = pc_np.flatten().tolist()
        cloud_pose = cloud_pose.flatten().tolist()
        point_data = {"frame_pose": cloud_pose, "data": pc_np}  # Convert to list of lists
        data_["point_cloud"] = point_data
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
        self.current_odom = transformation_matrix
        init_data_["tf_odom_to_base"] = transformation_matrix
        
        # Convert to list of lists
        transformation_matrix = transformation_matrix.flatten().tolist()
        odom_data = {"frame_pose": transformation_matrix}  # Convert to list of lists
        data_["odom"] = odom_data
        self.current_odom = transformation_matrix
        rospy.loginfo("Updated odometry data")
        
    def convert_mesh_to_numpy(self, mesh):
        # return arary of vertices (N2, 3), convert to numpy array of size 64 * 64 with z information
        # fake data: TODO: replace with actual computation
        mesh_np = np.random.rand(64, 64)
        return mesh_np

    def mesh_callback(self, data):
        # get the current tf of data frame to odom
        if init_data_["tf_device_to_odom"] is None:
            rospy.loginfo("No device to odom TF received yet, skipping mesh data")
            return
        if data.header.frame_id != "base_link":
            # convert the frame to base_link using tf2
            rospy.loginfo("Mesh frame is not base_link, skipping mesh data")
            return
        mesh_pose = init_data_["tf_device_to_odom"] @ self.current_odom
        mesh_np = self.convert_mesh_to_numpy(data)
        # Convert to list of lists
        mesh_np = mesh_np.flatten().tolist()
        mesh_pose = mesh_pose.flatten().tolist()
        mesh_data = {"frame_pose": mesh_pose, "data": mesh_np}
        data_["mesh"] = mesh_data
        rospy.loginfo("Updated mesh data")

def start_socket_server():
    eventlet.wsgi.server(eventlet.listen(('', 5030)), app)

@socket.on('target_pose')  # Listening for 'client_message' event
def handle_client_message1(sid, message):
    commands_["target_pose"] = decode_transformation_matrix(message)
    print(f"Received target message from client {sid}")
    print(commands_["target_pose"])
    
@socket.on('init_data')  # Listening for 'client_message' event
def handle_client_message2(sid, message):
    device_to_base = decode_transformation_matrix(message)
    device_to_odom = np.matmul(np.linalg.inv(init_data_["tf_odom_to_base"]), device_to_base)
    init_data_["tf_device_to_odom"] = device_to_odom
    print(f"Received device transofrm TF from client {sid}")
    print(init_data_["tf_base_to_device"])
    
@socket.on('is_pointcloud')  # Listening for 'client_message' event
def handle_client_message3(sid, message):
    # decode bool message
    commands_["is_pointcloud"] = decode_bool(message)
    print(f"Received point cloud command from client {sid}")
    print(commands_["is_pointcloud"])
    
@socket.on('is_mesh')  # Listening for 'client_message' event
def handle_client_message4(sid, message):
    # decode bool message
    commands_["is_mesh"] = decode_bool(message)
    print(f"Received mesh command from client {sid}")
    print(commands_["is_mesh"])

def decode_transformation_matrix(json_string):
    # Parse the JSON string
    data = json.loads(json_string)

    # Extract the matrix values
    matrix_values = data['poseTarget']

    # Reshape the values into a 4x4 matrix
    transformation_matrix = np.array(matrix_values).reshape(4, 4)

    return transformation_matrix

def decode_bool(json_string):
    # Parse the JSON string
    data = json.loads(json_string)

    # Extract the matrix values
    bool_value = data['bool']

    return bool_value

def pointcloud_worker():
    while(1):
        if commands_["is_pointcloud"] and data_["point_cloud"] is not None:
            print("send point cloud")
            socket.emit('point_cloud', json.dumps(data_["point_cloud"]))
            data_["point_cloud"] = None
        socket.sleep(0.5) # 2 Hz
        
def mesh_worker():
    while(1):
        if commands_["is_mesh"] and data_["mesh"] is not None:
            print("send mesh")
            socket.emit('mesh', json.dumps(data_["mesh"]))
            data_["mesh"] = None
        socket.sleep(0.5) # 2 Hz
        
def odom_worker():
    while(1):
        if data_["odom"] is not None:
            print("send odom")
            socket.emit('odom', json.dumps(data_["odom"]))
            data_["odom"] = None
        socket.sleep(0.05) # 20 Hz

def main():
    # ros_thread = eventlet.spawn(SocketRosNode)
    SocketRosNode()
    socket.start_background_task(pointcloud_worker)
    socket.start_background_task(mesh_worker)
    socket.start_background_task(odom_worker)
    start_socket_server()

if __name__ == '__main__':
    main()
    rospy.signal_shutdown("Shutting down ROS node")