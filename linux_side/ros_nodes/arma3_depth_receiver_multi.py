#!/usr/bin/env python3
"""
Arma 3 Multi-UAV Depth Receiver Node
Arma 3 多 UAV 深度接收节点

功能：
1. 为每个 UAV 创建独立的 TCP 服务器
2. 接收来自批处理深度估计服务的深度图
3. 发布到对应的 ROS 话题 /uavN/camera/depth/image_rect_raw

使用方法：
rosrun arma3_ros_bridge arma3_depth_receiver_multi.py _num_uavs:=6 _base_port:=5560
"""

import rospy
import socket
import json
import base64
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread
import sys

class DepthReceiver:
    """单个 UAV 的深度接收器"""
    
    def __init__(self, uav_id, port):
        """
        初始化深度接收器
        
        Args:
            uav_id: UAV ID
            port: TCP 端口
        """
        self.uav_id = uav_id
        self.port = port
        self.bridge = CvBridge()
        self.running = True
        
        # ROS publisher
        topic_name = f"/uav{uav_id}/camera/depth/image_rect_raw"
        self.depth_pub = rospy.Publisher(topic_name, Image, queue_size=10)
        
        # 统计信息
        self.images_received = 0
        self.start_time = rospy.Time.now().to_sec()
        
        # TCP server
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server.bind(('0.0.0.0', port))
            self.server.listen(5)
            rospy.loginfo(f"UAV {uav_id} depth receiver started on port {port}")
        except Exception as e:
            rospy.logerr(f"UAV {uav_id}: Failed to bind to port {port}: {e}")
            raise
    
    def receive_and_publish(self):
        """接收深度图并发布"""
        rospy.loginfo(f"UAV {self.uav_id}: Waiting for depth maps...")
        
        while not rospy.is_shutdown() and self.running:
            try:
                conn, addr = self.server.accept()
                
                # 接收消息长度（4 字节）
                msg_len_bytes = conn.recv(4)
                if len(msg_len_bytes) < 4:
                    conn.close()
                    continue
                
                msg_len = int.from_bytes(msg_len_bytes, byteorder='big')
                
                # 接收消息内容
                data = b''
                while len(data) < msg_len:
                    chunk = conn.recv(min(msg_len - len(data), 1024 * 1024))
                    if not chunk:
                        break
                    data += chunk
                
                conn.close()
                
                if len(data) < msg_len:
                    continue
                
                # 解析 JSON
                message = json.loads(data.decode('utf-8'))
                depth_b64 = message['depth']
                timestamp = message['timestamp']
                width = message.get('width', 518)
                height = message.get('height', 518)
                
                # 解码深度图
                depth_data = base64.b64decode(depth_b64)
                depth_array = np.frombuffer(depth_data, np.uint8)
                depth_map = cv2.imdecode(depth_array, cv2.IMREAD_GRAYSCALE)
                
                if depth_map is None:
                    rospy.logwarn(f"UAV {self.uav_id}: Failed to decode depth map")
                    continue
                
                # 转换为 ROS Image 消息
                depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="mono8")
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = f"uav{self.uav_id}_camera"
                
                # 发布
                self.depth_pub.publish(depth_msg)
                
                self.images_received += 1
                elapsed = rospy.Time.now().to_sec() - self.start_time
                rate = self.images_received / elapsed if elapsed > 0 else 0
                
                rospy.loginfo(f"UAV {self.uav_id}: Published depth map "
                             f"(size: {depth_map.shape}, "
                             f"total: {self.images_received}, "
                             f"rate: {rate:.2f} Hz)")
                
            except Exception as e:
                if self.running:
                    rospy.logerr(f"UAV {self.uav_id}: Error receiving depth: {e}")
    
    def run(self):
        """启动接收线程"""
        Thread(target=self.receive_and_publish, daemon=True).start()
    
    def stop(self):
        """停止接收器"""
        self.running = False
        try:
            self.server.close()
        except:
            pass

class MultiUAVDepthReceiver:
    """多 UAV 深度接收管理器"""
    
    def __init__(self, num_uavs, base_port):
        """
        初始化多 UAV 深度接收器
        
        Args:
            num_uavs: UAV 数量
            base_port: 基础端口（每个 UAV 使用 base_port + uav_id）
        """
        self.num_uavs = num_uavs
        self.base_port = base_port
        self.receivers = []
        
        rospy.loginfo("="*60)
        rospy.loginfo("Multi-UAV Depth Receiver Node")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Configuration:")
        rospy.loginfo(f"  Number of UAVs: {num_uavs}")
        rospy.loginfo(f"  Base port: {base_port}")
        rospy.loginfo(f"  Port range: {base_port}-{base_port + num_uavs - 1}")
        rospy.loginfo("="*60)
        
        # 为每个 UAV 创建接收器
        for uav_id in range(num_uavs):
            port = base_port + uav_id
            try:
                receiver = DepthReceiver(uav_id, port)
                receiver.run()
                self.receivers.append(receiver)
                rospy.loginfo(f"✓ UAV {uav_id} receiver initialized on port {port}")
            except Exception as e:
                rospy.logerr(f"✗ Failed to initialize UAV {uav_id} receiver: {e}")
        
        if len(self.receivers) == 0:
            rospy.logerr("No receivers initialized. Exiting.")
            sys.exit(1)
        
        rospy.loginfo("="*60)
        rospy.loginfo(f"✓ Started {len(self.receivers)} depth receivers")
        rospy.loginfo("Waiting for depth maps from Windows batch service...")
        rospy.loginfo("="*60)
    
    def print_stats(self):
        """定期打印统计信息"""
        rate = rospy.Rate(0.1)  # 每 10 秒
        
        while not rospy.is_shutdown():
            rate.sleep()
            
            rospy.loginfo("\n" + "="*60)
            rospy.loginfo("STATISTICS:")
            
            total_images = 0
            for receiver in self.receivers:
                elapsed = rospy.Time.now().to_sec() - receiver.start_time
                rate = receiver.images_received / elapsed if elapsed > 0 else 0
                total_images += receiver.images_received
                
                rospy.loginfo(f"  UAV {receiver.uav_id}: "
                             f"{receiver.images_received} images, "
                             f"{rate:.2f} Hz")
            
            rospy.loginfo(f"  Total: {total_images} images")
            rospy.loginfo("="*60 + "\n")
    
    def run(self):
        """运行接收器"""
        # 启动统计线程
        Thread(target=self.print_stats, daemon=True).start()
        
        # 保持运行
        rospy.spin()
        
        # 清理
        for receiver in self.receivers:
            receiver.stop()

def main():
    # 初始化 ROS 节点
    rospy.init_node('arma3_depth_receiver_multi', anonymous=False)
    
    # 从参数服务器获取配置
    num_uavs = rospy.get_param('~num_uavs', 6)
    base_port = rospy.get_param('~base_port', 5560)
    
    try:
        # 创建并运行多 UAV 接收器
        receiver = MultiUAVDepthReceiver(num_uavs, base_port)
        receiver.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down multi-UAV depth receiver...")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
