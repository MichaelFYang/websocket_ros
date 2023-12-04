# Client
import socketio

sio = socketio.Client()

@sio.event
def connect():
    print('connection established')

@sio.event
def clock(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.event
def disconnect():
    print('disconnected from server')
    
@sio.on('point_cloud')
def point_cloud(data):
    print('point cloud received with ', data)

sio.connect('http://localhost:5030')
sio.wait()