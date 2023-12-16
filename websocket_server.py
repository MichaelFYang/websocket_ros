# Server
import socketio
import eventlet
import numpy as np
import json
import time
import math
from scipy.interpolate import splprep, splev

# ROS modules
import rospy
import ros_numpy
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
# from shape_msgs.msg import Mesh
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

import socket as socket_lib
socket_lib.setdefaulttimeout(1e9)

socket = socketio.Server(async_mode='eventlet', always_connect=True, ping_timeout=1e9, ping_interval=1e9)
app = socketio.WSGIApp(socket)

data_test_ = {"data": None}

data_ = {"odom": None, "point_cloud": None, "mesh": None, "path": None}

commands_ = {"is_pointcloud": False, "is_mesh": False, "is_follow": False, "target_pose": None}

init_data_ = {"tf_device_to_odom": None, "tf_odom_to_base": None}

class SocketRosNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('socket_ros_node', disable_signals=True)
        
        # fake TF to device
        # set current odom
        self.odom_to_base = np.eye(4)
        init_data_["tf_device_to_odom"] = np.eye(4)
        self.previous_command = True

        # Create subscribers
        self.point_cloud_sub = rospy.Subscriber('/point_cloud_filter_rsl/filter_and_merger_rsl', PointCloud2, self.point_cloud_callback)
        self.odometry_sub = rospy.Subscriber('/state_estimator/pose_in_odom', PoseWithCovarianceStamped, self.odometry_callback)
        self.mesh_sub = rospy.Subscriber('/elevation_mapping/elevation_map_raw', GridMap, self.mesh_callback)
        self.plan_path_sub = rospy.Subscriber('/path', Path, self.plan_path_callback)
        
        # create a waypoint publisher
        self.waypoint_pub = rospy.Publisher('/mp_waypoint', PointStamped, queue_size=10)
    
        
        # Initialize a service server
        self.service = rospy.Service('generate_synthetic_data', Trigger, self.handle_generate_synthetic_data)
            
        # timer event callback to call the service
        # rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        rospy.Timer(rospy.Duration(1.0), self.goal_callback) # 1hz
        rospy.Timer(rospy.Duration(1.0), self.path_following_client) # 1hz
        
        self.cur_odom_to_goal = None
        
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
    
    def path_following_client(self, event):
        # if commands_["is_follow"] == self.previous_command:
        #     return
        # call the service to enable path following
        rospy.wait_for_service('/enable_path_following')
        try:
            enable_call = rospy.ServiceProxy('/enable_path_following', SetBool)
            resp = enable_call(commands_["is_follow"])
            print(resp.message)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def waypointPublisher(self, pose):
        # publish the waypoint
        waypoint_msg = PointStamped()
        waypoint_msg.header.frame_id = "odom"
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.point.x = pose[0, 3]
        waypoint_msg.point.y = pose[1, 3]
        waypoint_msg.point.z = pose[2, 3]
        self.waypoint_pub.publish(waypoint_msg)
        
    def _determine_num_points(self, x, y, z, distance_resolution=0.2):
        """
        Determines the number of points for B-spline interpolation based on distance resolution.
        :param x, y, z: Lists of x, y, z coordinates of trajectory points.
        :param distance_resolution: Desired distance between interpolated points.
        :return: Number of points for interpolation.
        """
        total_distance = sum(math.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2 + (z[i+1] - z[i])**2) 
                             for i in range(len(x) - 1))
        return max(int(total_distance / distance_resolution), 2)  # At least two points
    
    def interpolatePath(self, path):
        # Convert to world coordinates and separate into x, y, z lists
        x, y, z = [], [], []
        const_z = path.poses[0].pose.position.z
        N = len(path.poses)
        for i in range(N):
            x.append(path.poses[i].pose.position.x)
            y.append(path.poses[i].pose.position.y)
            z.append(const_z)
            
        # add goal point
        # convert it to base frame
        base_to_goal = np.linalg.inv(self.odom_to_base) @ self.cur_odom_to_goal
        x.append(base_to_goal[0, 3])
        y.append(base_to_goal[1, 3])
        z.append(const_z)

        # Apply B-spline interpolation
        tck, u = splprep([x, y, z], s=0)
        u_new = np.linspace(u.min(), u.max(), self._determine_num_points(x, y, z, 0.1))
        x_new, y_new, z_new = splev(u_new, tck)
        # combine x_new, y_new, z_new to an array
        path_np = np.zeros((len(x_new), 3))
        path_np[:, 0] = x_new
        path_np[:, 1] = y_new
        path_np[:, 2] = z_new
        return path_np
    
        
    def plan_path_callback(self, data):
        if  self.cur_odom_to_goal is None:
            rospy.loginfo_throttle(1, "No goal received yet, skipping path data")
            return
        # write the path to data_["path"], the path is at the base frame, need to convert it to device frame
        # check if device to odom TF is received
        if init_data_["tf_device_to_odom"] is None:
            rospy.loginfo("No device to odom TF received yet, skipping path data")
            return

        if len(data.poses) < 2:
            rospy.loginfo("No point in path received yet, skipping path data")
            return
        
        # print("path received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        path_np = self.interpolatePath(data)
        # convert to device frame
        device_to_base = init_data_["tf_device_to_odom"] @ self.odom_to_base
        path_data = {"frame_pose": device_to_base.flatten().tolist(), "data": path_np.flatten().tolist()}
        data_["path"] = path_data
        # rospy.loginfo("Updated path data")
        
    
    def handle_generate_synthetic_data(self, request):
        self.generate_synthetic_data()
        # sysnthetic data generated successfully
        return TriggerResponse(
            success=True,
            message="==================================\nSynthetic data generated successfully"
        )
        
    def goal_callback(self, event):
        if commands_["target_pose"] is not None:
            # target pose is device to goal, and need odom_to_goal
            odom_to_goal = np.linalg.inv(init_data_["tf_device_to_odom"]) @ commands_["target_pose"]
            # z shift 0.5m (robot height)
            odom_to_goal[2, 3] += 0.5
            self.waypointPublisher(odom_to_goal)
            # # write the path to data_["path"]
            # device_to_base = init_data_["tf_device_to_odom"] @ self.odom_to_base
            # device_to_target = commands_["target_pose"]
            # device_to_base_pos = device_to_base[:3, 3]
            # device_to_target_pos = device_to_target[:3, 3]
            # # generate a path with 5 points
            # path = np.zeros((5, 3))
            # for i in range(5):
            #     path[i, :] = device_to_base_pos + (device_to_target_pos - device_to_base_pos) * i / 4
            #     # add some noise
            #     path[i, :] += np.random.normal(0, 0.1, 3)
            # # convert to list of lists
            # path = path.flatten().tolist()
            # data_["path"] = path
            self.cur_odom_to_goal  = odom_to_goal
            commands_["target_pose"] = None
        
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
        N = 5000  # Number of points
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
        self.odom_to_base = synthetic_odom
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
        # Convert PointCloud2 to numpy array (N, 3) using ros_numpy
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        # print("size of the points: ", pc_np.shape[0])
        pc_np = pc_np[::5, :]
        # round pc_np to 3 decimal places
        pc_np = np.round(pc_np, 2)
        # check if device to odom TF is received
        if init_data_["tf_device_to_odom"] is None:
            rospy.loginfo("No device to odom TF received yet, skipping point cloud data")
            return
        if data.header.frame_id != "lidar": # need to transer to base link
            rospy.loginfo("Point cloud frame is not base_link, skipping point cloud data")
            return
        
        # get the transformation from base to lidar [Camel] 
        base_to_lidar = np.eye(4)
        base_to_lidar[:3, :3] = Rotation.from_quat([0.000, 0.000, -0.707, 0.707]).as_matrix()
        base_to_lidar[:3, 3] = np.array([-0.364, 0.000, 0.142])

        cloud_pose = init_data_["tf_device_to_odom"] @ self.odom_to_base @ base_to_lidar
        # Convert to list of lists
        pc_np = pc_np.flatten().tolist()
        cloud_pose = cloud_pose.flatten().tolist()
        point_data = {"frame_pose": cloud_pose, "data": pc_np}  # Convert to list of lists
        data_["point_cloud"] = point_data
        # rospy.loginfo("Updated point cloud data")

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
        self.odom_to_base = transformation_matrix.copy()
        init_data_["tf_odom_to_base"] = transformation_matrix.copy()
        # print("tf_device_to_odom: ", init_data_["tf_device_to_odom"])
        
        transformation_matrix = init_data_["tf_device_to_odom"] @ self.odom_to_base
        # print("odom trans update info: ", translation)
        # transformation_matrix = init_data_["tf_device_to_odom"]
        transformation_matrix = transformation_matrix.flatten().tolist()
        odom_data = {"frame_pose": transformation_matrix}  # Convert to list of lists
        data_["odom"] = odom_data
        # rospy.loginfo("Updated odometry data")
        
    def convert_mesh_to_numpy(self, msg):
        # return arary of vertices (N2, 3), convert to numpy array of size 64 * 64 with z information
        # fake data: TODO: replace with actual computation
        LAYER = 0
        res = msg.info.resolution
        dim_x = int(msg.info.length_x / msg.info.resolution)
        dim_y = int(msg.info.length_y / msg.info.resolution)
        mesh_np = np.array(msg.data[LAYER].data).reshape(dim_x, dim_y)
        # crop and downsample
        mesh_np = mesh_np[50:150:2, 50:150:2]
        dim_x = mesh_np.shape[0]
        dim_y = mesh_np.shape[1]
        res *= 2
        # remove the nan values
        mesh_np = np.nan_to_num(mesh_np, nan=-1e3)
        # round to 2 decimal places
        mesh_np = np.round(mesh_np, 2)        
        # mesh_np = np.random.rand(64, 64)
        return mesh_np, res, dim_x, dim_y
    
    def geometry_msgs_pose_to_transformation_matrix(self, pose):
        # convert geometry_msgs/PoseStamped to 4x4 transformation matrix
        position = pose.position
        orientation = pose.orientation
        translation = np.array([position.x, position.y, position.z])
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        pose_matrix = np.eye(4)
        pose_matrix[:3, 3] = translation
        # Create a rotation object from the quaternion and convert it to a matrix
        rotation_matrix = Rotation.from_quat(quaternion).as_matrix()
        # Create the transformation matrix
        pose_matrix[:3, :3] = rotation_matrix
        return pose_matrix

    def mesh_callback(self, data):
        # get the current tf of data frame to odom
        if init_data_["tf_device_to_odom"] is None:
            rospy.loginfo("No device to odom TF received yet, skipping mesh data")
            return
        # convert geometry_msgs/PoseStamped to 4x4 transformation matrix
        odom_to_ele = self.geometry_msgs_pose_to_transformation_matrix(data.info.pose)        
        mesh_pose = init_data_["tf_device_to_odom"] @ odom_to_ele
        mesh_np, res, dim_x, dim_y = self.convert_mesh_to_numpy(data)
        # Convert to list of lists
        mesh_np = mesh_np.flatten().tolist()
        mesh_pose = mesh_pose.flatten().tolist()
        mesh_data = {"frame_pose": mesh_pose, "res": res, "dim_x": dim_x, "dim_y": dim_y, "data": mesh_np}
        # mesh_data = {"frame_pose": mesh_pose, "data": mesh_np}
        data_["mesh"] = mesh_data
        # rospy.loginfo("Updated mesh data")

def start_socket_server():
    eventlet.wsgi.server(eventlet.listen(('', 5030)), app)

@socket.on('target_pose')  # Listening for 'client_message' event
def handle_client_message1(sid, message):
    commands_["target_pose"] = decode_transformation_matrix(message)
    commands_["is_follow"] = False
    print(f"Received target message from client {sid}")
    print(commands_["target_pose"])
    
@socket.on('init_data')  # Listening for 'client_message' event
def handle_client_message2(sid, message):
    device_to_base = decode_transformation_matrix(message)
    print("tf_odom: ",  init_data_["tf_odom_to_base"])
    print("recieved pose:", device_to_base)
    # device_to_odom = np.linalg.inv(init_data_["tf_odom_to_base"]) @ device_to_base
    device_to_odom = device_to_base @ np.linalg.inv(init_data_["tf_odom_to_base"])
    init_data_["tf_device_to_odom"] = device_to_odom
    # init_data_["tf_device_to_odom"] = device_to_base
    print(f"Received device transofrm TF from client {sid}")
    print(init_data_["tf_device_to_odom"])
    
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
    
@socket.on('go_to_target')  # Listening for 'client_message' event
def handle_client_message5(sid, message):
    # decode bool message
    commands_["is_follow"] = decode_bool(message)
    print(f"Received follow command from client {sid}")
    print(commands_["is_follow"])

def decode_transformation_matrix(json_string):
    # Parse the JSON string
    data = json.loads(json_string)

    # Extract the matrix values
    matrix_values = data['pose']

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
        socket.sleep(0.5) # 5 Hz
        
def mesh_worker():
    while(1):
        if commands_["is_mesh"] and data_["mesh"] is not None:
            print("send mesh")
            socket.emit('mesh', json.dumps(data_["mesh"]))
            data_["mesh"] = None
        socket.sleep(0.5) # 5 Hz
        
def path_worker():
    while(1):
        if data_["path"] is not None:
            print("send path")
            socket.emit('path', json.dumps(data_["path"]))
            # print(data_["path"])
            data_["path"] = None
        socket.sleep(0.5) # 5 Hz
        
def odom_worker():
    while(1):
        if data_["odom"] is not None and init_data_["tf_device_to_odom"] is not None:
            # print("send odom to device")
            socket.emit('odom', json.dumps(data_["odom"]))
            data_["odom"] = None
        socket.sleep(0.1) # 10 Hz

def main():
    # ros_thread = eventlet.spawn(SocketRosNode)
    SocketRosNode()
    while not rospy.is_shutdown():
        try:
            socket.start_background_task(pointcloud_worker)
            socket.start_background_task(mesh_worker)
            socket.start_background_task(odom_worker)
            socket.start_background_task(path_worker)
            start_socket_server()
        except:
            pass


if __name__ == '__main__':
    main()
    rospy.signal_shutdown("Shutting down ROS node")