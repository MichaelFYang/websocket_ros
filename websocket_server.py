# Server
import socketio
import eventlet
import numpy as np
import json
import time

socket = socketio.Server(async_mode='eventlet')
app = socketio.WSGIApp(socket)

def worker1():
    eventlet.wsgi.server(eventlet.listen(('', 5030)), app)


def generate_points(num_points, radius=1.0):
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
    
    # flatten
    points = points.flatten()

    return points.tolist()

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

def worker2():
    
    while(1):
        print("send msg")
        data = {"data": generate_points(1000, radius=0.2)}
        socket.emit('point_cloud', json.dumps(data))
        socket.sleep(1)

def main():
    socket.start_background_task(worker2)
    worker1()

if __name__ == '__main__':
    main()