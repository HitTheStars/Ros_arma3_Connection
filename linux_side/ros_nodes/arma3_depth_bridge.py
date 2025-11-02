#!/usr/bin/env python3
"""
Arma 3 深度图像桥接 ROS 节点（修正版）
接收来自 Windows 端的深度图像，并发布为 ROS sensor_msgs/Image 消息

订阅话题：无（通过 TCP/IP 接收）
发布话题：
    /arma3/depth (sensor_msgs/Image) - 深度图像（uint16，毫米）
    /arma3/odom (nav_msgs/Odometry) - 无人机位姿

作者：修正版，符合 EGO-Planner-v2 的输入要求
"""

import rospy
import socket
import threading
import struct
import json
import base64
import numpy as np
import cv2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge

class Arma3DepthBridge:
    """Arma 3 深度图像桥接节点"""
    
    def __init__(self):
        rospy.init_node('arma3_depth_bridge', anonymous=False)
        
        # 参数
        self.server_ip = rospy.get_param('~server_ip', '0.0.0.0')
        self.server_port = rospy.get_param('~server_port', 5555)
        
        # 发布器
        self.depth_pub = rospy.Publisher('/arma3/depth', Image, queue_size=10)
        self.odom_pub = rospy.Publisher('/arma3/odom', Odometry, queue_size=10)
        
        # 订阅器（用于接收控制指令）
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TCP 服务器
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.connected = False
        
        # 统计
        self.frame_count = 0
        self.last_print_time = rospy.Time.now()
        
    def start_server(self):
        """启动 TCP 服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.server_ip, self.server_port))
            self.server_socket.listen(1)
            
            rospy.loginfo(f"[Arma3 Depth Bridge] TCP server started on {self.server_ip}:{self.server_port}")
            rospy.loginfo("[Arma3 Depth Bridge] Waiting for Windows bridge to connect...")
            
            # 接受连接
            self.client_socket, self.client_address = self.server_socket.accept()
            self.connected = True
            rospy.loginfo(f"[Arma3 Depth Bridge] Client connected from {self.client_address}")
            
            # 启动接收线程
            recv_thread = threading.Thread(target=self.receive_loop)
            recv_thread.daemon = True
            recv_thread.start()
            
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Server error: {e}")
    
    def receive_loop(self):
        """接收数据的循环"""
        while self.connected and not rospy.is_shutdown():
            try:
                # 接收消息长度（4 字节）
                length_data = self.client_socket.recv(4)
                if not length_data:
                    break
                
                msg_length = struct.unpack('!I', length_data)[0]
                
                # 接收完整消息
                data = b''
                while len(data) < msg_length:
                    chunk = self.client_socket.recv(min(msg_length - len(data), 4096))
                    if not chunk:
                        break
                    data += chunk
                
                if len(data) == msg_length:
                    message = data.decode('utf-8')
                    self.handle_message(message)
                    
            except Exception as e:
                rospy.logerr(f"[Arma3 Depth Bridge] Receive error: {e}")
                break
        
        self.connected = False
        rospy.logwarn("[Arma3 Depth Bridge] Client disconnected")
    
    def handle_message(self, message):
        """处理接收到的消息"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'DEPTH_IMAGE':
                self.handle_depth_image(data)
            elif msg_type == 'STATUS':
                self.handle_status(data)
                
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Error handling message: {e}")
    
    def handle_depth_image(self, data):
        """处理深度图像消息"""
        try:
            # 解码图像数据
            img_base64 = data.get('data')
            img_bytes = base64.b64decode(img_base64)
            
            # 解码 PNG
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            depth_image = cv2.imdecode(img_array, cv2.IMREAD_UNCHANGED)
            
            if depth_image is None:
                rospy.logwarn("[Arma3 Depth Bridge] Failed to decode depth image")
                return
            
            # 转换为 ROS Image 消息
            ros_image = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "arma3_camera"
            
            # 发布
            self.depth_pub.publish(ros_image)
            
            # 统计
            self.frame_count += 1
            if (rospy.Time.now() - self.last_print_time).to_sec() >= 1.0:
                valid_pixels = np.count_nonzero(depth_image)
                rospy.loginfo(f"[Arma3 Depth Bridge] Received depth image {self.frame_count}: {depth_image.shape[1]}x{depth_image.shape[0]}, {valid_pixels} valid pixels")
                self.last_print_time = rospy.Time.now()
                
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Error handling depth image: {e}")
    
    def handle_status(self, data):
        """处理状态消息"""
        try:
            # 构造 Odometry 消息
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "world"
            odom.child_frame_id = "arma3_uav"
            
            odom.pose.pose.position.x = data.get('x', 0.0)
            odom.pose.pose.position.y = data.get('y', 0.0)
            odom.pose.pose.position.z = data.get('z', 0.0)
            
            # TODO: 添加方向（四元数）
            odom.pose.pose.orientation.w = 1.0
            
            # 发布
            self.odom_pub.publish(odom)
            
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Error handling status: {e}")
    
    def goal_callback(self, msg):
        """接收目标点并发送到 Arma 3"""
        if not self.connected:
            return
        
        try:
            # 构造 GOAL 消息
            command = {
                'type': 'GOAL',
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            }
            
            # 发送
            self.send_message(command)
            rospy.loginfo(f"[Arma3 Depth Bridge] Sent GOAL: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")
            
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Error sending goal: {e}")
    
    def send_message(self, message):
        """发送消息到 Windows 端"""
        if not self.connected:
            return
        
        try:
            msg_str = json.dumps(message)
            msg_bytes = msg_str.encode('utf-8')
            length_prefix = struct.pack('!I', len(msg_bytes))
            self.client_socket.sendall(length_prefix + msg_bytes)
        except Exception as e:
            rospy.logerr(f"[Arma3 Depth Bridge] Send error: {e}")
            self.connected = False
    
    def run(self):
        """运行节点"""
        self.start_server()
        rospy.spin()
    
    def shutdown(self):
        """关闭节点"""
        self.connected = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

def main():
    try:
        bridge = Arma3DepthBridge()
        rospy.on_shutdown(bridge.shutdown)
        bridge.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
