#!/usr/bin/env python3
"""
ROS Bridge Node for Arma 3
This node runs on Linux (ROS environment) and communicates with Arma 3 via TCP/IP.
It receives images and UAV status from Arma 3, and sends control commands back.
"""

import rospy
import socket
import struct
import threading
import queue
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2

class Arma3ROSBridge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arma3_ros_bridge', anonymous=False)
        
        # Parameters
        self.listen_port = rospy.get_param('~listen_port', 5555)
        self.num_uavs = rospy.get_param('~num_uavs', 3)
        
        # Socket for communication
        self.server_socket = None
        self.client_socket = None
        self.connected = False
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pubs = []
        self.pose_pubs = []
        self.vel_pubs = []
        self.pointcloud_pub = rospy.Publisher('/arma3/pointcloud', PointCloud2, queue_size=10)
        
        for i in range(6):  # 6 cameras
            pub = rospy.Publisher(f'/arma3/camera_{i}/image_raw', Image, queue_size=10)
            self.image_pubs.append(pub)
        
        for i in range(self.num_uavs):
            pose_pub = rospy.Publisher(f'/arma3/uav_{i}/pose', PoseStamped, queue_size=10)
            vel_pub = rospy.Publisher(f'/arma3/uav_{i}/velocity', TwistStamped, queue_size=10)
            self.pose_pubs.append(pose_pub)
            self.vel_pubs.append(vel_pub)
        
        # Subscribers for control commands
        for i in range(self.num_uavs):
            rospy.Subscriber(f'/ego_planner/uav_{i}/command', PoseStamped, 
                           lambda msg, uav_id=i: self.command_callback(msg, uav_id))
        
        # Queues
        self.command_queue = queue.Queue()
        
        rospy.loginfo("Arma3 ROS Bridge initialized")
    
    def start_server(self):
        """Start TCP server to listen for Arma 3 connection"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.listen_port))
            self.server_socket.listen(1)
            
            rospy.loginfo(f"Waiting for Arma 3 connection on port {self.listen_port}...")
            
            self.client_socket, addr = self.server_socket.accept()
            self.connected = True
            
            rospy.loginfo(f"Arma 3 connected from {addr}")
            
        except Exception as e:
            rospy.logerr(f"Failed to start server: {e}")
    
    def receive_message(self, size):
        """Receive exactly 'size' bytes from socket"""
        data = b''
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def receive_images(self):
        """Thread for receiving images from Arma 3"""
        rospy.loginfo("Image receive thread started")
        
        while not rospy.is_shutdown() and self.connected:
            try:
                # Receive message type
                msg_type = self.receive_message(4)
                if not msg_type:
                    rospy.logwarn("Connection closed by Arma 3")
                    self.connected = False
                    break
                
                if msg_type.startswith(b'IMG'):
                    # Parse image ID
                    image_id = int(msg_type[3:4])
                    
                    # Receive timestamp
                    timestamp_data = self.receive_message(8)
                    timestamp = struct.unpack('!Q', timestamp_data)[0]
                    
                    # Receive image size
                    size_data = self.receive_message(4)
                    image_size = struct.unpack('!I', size_data)[0]
                    
                    # Receive image data
                    image_data = self.receive_message(image_size)
                    
                    # Decode JPEG image
                    nparr = np.frombuffer(image_data, np.uint8)
                    cv_image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    # Convert to ROS Image message
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                    ros_image.header.stamp = rospy.Time.now()
                    ros_image.header.frame_id = f"camera_{image_id}"
                    
                    # Publish image
                    self.image_pubs[image_id].publish(ros_image)
                    
                    rospy.logdebug(f"Received and published image {image_id}")
                    
                elif msg_type == b'STAT':
                    # Receive UAV status
                    data = self.receive_message(40)
                    uav_id, px, py, pz, vx, vy, vz, roll, pitch, yaw = struct.unpack('!I3f3f3f', data)
                    
                    # Publish pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "world"
                    pose_msg.pose.position.x = px
                    pose_msg.pose.position.y = py
                    pose_msg.pose.position.z = pz
                    
                    # Convert Euler angles to quaternion
                    from tf.transformations import quaternion_from_euler
                    q = quaternion_from_euler(roll, pitch, yaw)
                    pose_msg.pose.orientation.x = q[0]
                    pose_msg.pose.orientation.y = q[1]
                    pose_msg.pose.orientation.z = q[2]
                    pose_msg.pose.orientation.w = q[3]
                    
                    self.pose_pubs[uav_id].publish(pose_msg)
                    
                    # Publish velocity
                    vel_msg = TwistStamped()
                    vel_msg.header.stamp = rospy.Time.now()
                    vel_msg.header.frame_id = "world"
                    vel_msg.twist.linear.x = vx
                    vel_msg.twist.linear.y = vy
                    vel_msg.twist.linear.z = vz
                    
                    self.vel_pubs[uav_id].publish(vel_msg)
                    
                    rospy.logdebug(f"Received and published status for UAV {uav_id}")
                    
            except Exception as e:
                rospy.logerr(f"Error receiving data: {e}")
                self.connected = False
                break
    
    def send_commands(self):
        """Thread for sending commands to Arma 3"""
        rospy.loginfo("Command send thread started")
        
        while not rospy.is_shutdown() and self.connected:
            try:
                # Get command from queue
                command = self.command_queue.get(timeout=1.0)
                
                if command['type'] == 'MOVE':
                    # Construct text protocol message
                    x, y, z = command['target_position']
                    uav_id = command['uav_id']
                    message = f"MOVE:{uav_id},{x},{y},{z}\n"
                    
                    self.client_socket.sendall(message.encode('utf-8'))
                    rospy.loginfo(f"Sent MOVE command for UAV {uav_id}: [{x}, {y}, {z}]")
                    
                elif command['type'] == 'STOP':
                    # Send STOP command
                    message = "STOP\n"
                    self.client_socket.sendall(message.encode('utf-8'))
                    rospy.loginfo("Sent STOP command")
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error sending command: {e}")
                self.connected = False
                break
    
    def command_callback(self, msg, uav_id):
        """Callback for receiving control commands from EGO-Planner"""
        command = {
            'type': 'MOVE',
            'uav_id': uav_id,
            'target_position': (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            'target_velocity': (0.0, 0.0, 0.0)  # Default velocity
        }
        
        try:
            self.command_queue.put(command, block=False)
        except queue.Full:
            rospy.logwarn("Command queue full, dropping command")
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Starting Arma3 ROS Bridge...")
        
        # Start server
        self.start_server()
        
        if not self.connected:
            rospy.logerr("Failed to connect to Arma 3. Exiting.")
            return
        
        # Start threads
        receive_thread = threading.Thread(target=self.receive_images)
        send_thread = threading.Thread(target=self.send_commands)
        
        receive_thread.start()
        send_thread.start()
        
        # Keep node alive
        rospy.spin()
        
        # Cleanup
        receive_thread.join()
        send_thread.join()
        
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

if __name__ == '__main__':
    try:
        bridge = Arma3ROSBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
