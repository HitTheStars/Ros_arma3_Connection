#!/usr/bin/env python3
"""
Arma 3 Depth Image Receiver (ROS Node)
Linux 端深度图像接收节点

功能：
1. 通过 TCP/IP 接收来自 Windows 端的深度图像
2. 解码并发布为 ROS sensor_msgs/Image 消息
3. 完全兼容 EGO-Planner 的输入要求

作者: Manus AI
日期: 2025-11-03
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import socket
import json
import numpy as np
import cv2
import base64
import threading

class Arma3DepthReceiver:
    """Arma 3 深度图像接收节点"""
    
    def __init__(self):
        """初始化 ROS 节点"""
        rospy.init_node('arma3_depth_receiver', anonymous=False)
        
        # 参数
        self.tcp_port = rospy.get_param('~tcp_port', 5555)
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/depth/camera_info')
        
        # 相机内参（与 EGO-Planner 的 camera.yaml 一致）
        self.cam_width = rospy.get_param('~cam_width', 640)
        self.cam_height = rospy.get_param('~cam_height', 480)
        self.cam_fx = rospy.get_param('~cam_fx', 386.02557373046875)
        self.cam_fy = rospy.get_param('~cam_fy', 386.02557373046875)
        self.cam_cx = rospy.get_param('~cam_cx', 321.8554382324219)
        self.cam_cy = rospy.get_param('~cam_cy', 241.2396240234375)
        
        # 发布器
        self.depth_pub = rospy.Publisher(self.depth_topic, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TCP 服务器
        self.server_socket = None
        self.client_socket = None
        self.connected = False
        
        # 统计
        self.frame_count = 0
        self.total_latency = 0
        
        rospy.loginfo(f"[DepthReceiver] 初始化完成")
        rospy.loginfo(f"[DepthReceiver] TCP 端口: {self.tcp_port}")
        rospy.loginfo(f"[DepthReceiver] 深度话题: {self.depth_topic}")
        rospy.loginfo(f"[DepthReceiver] 相机信息话题: {self.camera_info_topic}")
    
    def create_camera_info(self, timestamp):
        """
        创建 CameraInfo 消息
        
        Args:
            timestamp: ROS 时间戳
            
        Returns:
            camera_info: CameraInfo 消息
        """
        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = "camera_depth_optical_frame"
        
        camera_info.width = self.cam_width
        camera_info.height = self.cam_height
        
        # 相机内参矩阵 K
        camera_info.K = [
            self.cam_fx, 0.0, self.cam_cx,
            0.0, self.cam_fy, self.cam_cy,
            0.0, 0.0, 1.0
        ]
        
        # 投影矩阵 P
        camera_info.P = [
            self.cam_fx, 0.0, self.cam_cx, 0.0,
            0.0, self.cam_fy, self.cam_cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # 畸变系数（假设无畸变）
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = "plumb_bob"
        
        # 旋转矩阵 R（单位矩阵）
        camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        return camera_info
    
    def start_tcp_server(self):
        """启动 TCP 服务器"""
        rospy.loginfo(f"[DepthReceiver] 启动 TCP 服务器，端口: {self.tcp_port}")
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.tcp_port))
        self.server_socket.listen(1)
        
        rospy.loginfo(f"[DepthReceiver] 等待 Windows 客户端连接...")
        
        self.client_socket, client_address = self.server_socket.accept()
        self.connected = True
        
        rospy.loginfo(f"[DepthReceiver] 客户端已连接: {client_address}")
    
    def receive_message(self):
        """
        接收一条完整的 JSON 消息（以换行符分隔）
        
        Returns:
            message: 解析后的消息字典，失败返回 None
        """
        try:
            # 接收数据直到遇到换行符
            buffer = b''
            while True:
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    # 连接断开
                    self.connected = False
                    return None
                
                buffer += chunk
                
                # 检查是否包含换行符
                if b'\n' in buffer:
                    # 分割消息
                    message_bytes, buffer = buffer.split(b'\n', 1)
                    
                    # 解析 JSON
                    message = json.loads(message_bytes.decode('utf-8'))
                    return message
        
        except Exception as e:
            rospy.logerr(f"[DepthReceiver] 接收消息失败: {e}")
            self.connected = False
            return None
    
    def process_depth_message(self, message):
        """
        处理深度图像消息
        
        Args:
            message: 深度图像消息字典
        """
        try:
            # 提取消息内容
            timestamp_sec = message['timestamp']
            width = message['width']
            height = message['height']
            encoding = message['encoding']
            depth_b64 = message['data']
            
            # Base64 解码
            depth_bytes = base64.b64decode(depth_b64)
            
            # PNG 解码
            depth_array = np.frombuffer(depth_bytes, dtype=np.uint8)
            depth_map = cv2.imdecode(depth_array, cv2.IMREAD_UNCHANGED)
            
            if depth_map is None:
                rospy.logerr("[DepthReceiver] PNG 解码失败")
                return
            
            # 转换为 ROS 时间戳
            ros_timestamp = rospy.Time.from_sec(timestamp_sec)
            
            # 创建 ROS Image 消息
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding=encoding)
            depth_msg.header.stamp = ros_timestamp
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            
            # 发布深度图像
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            camera_info = self.create_camera_info(ros_timestamp)
            self.camera_info_pub.publish(camera_info)
            
            # 更新统计
            self.frame_count += 1
            latency = (rospy.Time.now().to_sec() - timestamp_sec) * 1000  # ms
            self.total_latency += latency
            
            # 每 30 帧打印一次统计
            if self.frame_count % 30 == 0:
                avg_latency = self.total_latency / self.frame_count
                rospy.loginfo(f"[DepthReceiver] 统计 (最近 {self.frame_count} 帧):")
                rospy.loginfo(f"  - 平均端到端延迟: {avg_latency:.1f} ms")
                rospy.loginfo(f"  - 实际 FPS: {1000 / avg_latency:.1f}")
        
        except Exception as e:
            rospy.logerr(f"[DepthReceiver] 处理深度消息失败: {e}")
    
    def run(self):
        """运行主循环"""
        rospy.loginfo("[DepthReceiver] 启动深度图像接收节点...")
        
        # 启动 TCP 服务器
        self.start_tcp_server()
        
        rospy.loginfo("[DepthReceiver] 开始接收深度图像...")
        
        while not rospy.is_shutdown() and self.connected:
            # 接收消息
            message = self.receive_message()
            
            if message is None:
                rospy.logwarn("[DepthReceiver] 连接断开，等待重新连接...")
                self.client_socket.close()
                self.start_tcp_server()
                continue
            
            # 处理消息
            if message['type'] == 'DEPTH':
                self.process_depth_message(message)
            else:
                rospy.logwarn(f"[DepthReceiver] 未知消息类型: {message['type']}")
        
        # 清理
        self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("[DepthReceiver] 清理资源...")
        
        if self.client_socket:
            self.client_socket.close()
        
        if self.server_socket:
            self.server_socket.close()
        
        rospy.loginfo("[DepthReceiver] 节点已退出")


if __name__ == '__main__':
    try:
        receiver = Arma3DepthReceiver()
        receiver.run()
    except rospy.ROSInterruptException:
        pass
