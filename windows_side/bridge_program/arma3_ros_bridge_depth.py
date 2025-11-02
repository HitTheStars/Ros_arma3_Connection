"""
Arma 3 <-> ROS 深度图像桥接程序（修正版）
直接发送深度图像到 ROS，符合 EGO-Planner-v2 的输入要求

功能：
1. 从 Arma 3 截取多视角图像（6 个摄像头）
2. 进行立体视觉处理，生成视差图
3. 将视差图转换为深度图
4. 通过 TCP/IP 将深度图发送到 ROS（sensor_msgs/Image 格式）
5. 从 ROS 接收控制指令

作者：基于 codingWithArma3 项目改编，修正为符合 EGO-Planner-v2 的输入要求
"""

import time
import threading
import socket
import json
import struct
import numpy as np
import cv2
from PIL import ImageGrab
import io
import os
from queue import Queue

# ==================== 配置参数 ====================

# ROS 服务器配置
SERVER_IP = '192.168.1.100'  # 修改为你的 Linux VM IP
SERVER_PORT = 5555

# 图像采集配置
SCREENSHOT_REGIONS = [
    (0, 255, 455, 511),      # 左上
    (455, 255, 911, 511),    # 中上
    (911, 255, 1365, 511),   # 右上
    (0, 511, 455, 767),      # 左下
    (455, 511, 911, 767),    # 中下
    (911, 511, 1365, 767)    # 右下
]

# 立体视觉配置（来自 codingWithArma3）
STEREO_CONFIG = {
    'image_size': (960, 540),
    'baseline': 9.88485423343530,  # 基线距离（cm）
    'focal_length': 480.838407716,  # 焦距（像素）
    'left_intrinsics': np.array([
        [480.848082806618, 0, 479.349790865097],
        [0, 480.317300981934, 271.489960306777],
        [0, 0, 1]
    ]),
    'left_distortion': np.array([0.00260081621252845, -0.00139921398092175, 0, 0, 0]),
    'right_intrinsics': np.array([
        [480.828732625387, 0, 479.279109375706],
        [0, 480.312446180564, 271.357524349737],
        [0, 0, 1]
    ]),
    'right_distortion': np.array([0.00356112246793776, -0.00272917897030244, 0, 0, 0]),
    'R': np.array([
        [0.999999995878492, -5.46494494328387e-06, 9.06264286308695e-05],
        [5.43763246338141e-06, 0.999999954572720, 0.000301371845712365],
        [-9.06280714945010e-05, -0.000301371351677048, 0.999999950480929]
    ]),
    'T': np.array([-9.88485423343530, -0.0643088837682704, 0.147124435288212])
}

# ==================== 立体视觉处理类 ====================

class StereoVisionProcessor:
    """立体视觉处理器（修正版：输出深度图而不是点云）"""
    
    def __init__(self, config=STEREO_CONFIG):
        self.config = config
        self.setup_rectification()
        
        # 创建立体匹配器
        self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
    def setup_rectification(self):
        """设置立体校正参数"""
        R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(
            self.config['left_intrinsics'],
            self.config['left_distortion'],
            self.config['right_intrinsics'],
            self.config['right_distortion'],
            self.config['image_size'],
            self.config['R'],
            self.config['T']
        )
        
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            self.config['left_intrinsics'],
            self.config['left_distortion'],
            R1, P1,
            self.config['image_size'],
            cv2.CV_16SC2
        )
        
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            self.config['right_intrinsics'],
            self.config['right_distortion'],
            R2, P2,
            self.config['image_size'],
            cv2.CV_16SC2
        )
    
    def compute_disparity(self, left_img, right_img):
        """计算视差图"""
        # 校正图像
        left_rectified = cv2.remap(left_img, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_img, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
        
        # 转换为灰度图
        if len(left_rectified.shape) == 3:
            left_gray = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_rectified
            right_gray = right_rectified
        
        # 计算视差
        disparity = self.stereo.compute(left_gray, right_gray)
        disparity = disparity.astype(np.float32) / 16.0
        
        return disparity
    
    def disparity_to_depth(self, disparity):
        """
        将视差图转换为深度图（符合 EGO-Planner 的输入要求）
        
        深度图格式：
        - 类型：uint16（毫米）或 float32（米）
        - 每个像素存储该点的深度值
        """
        # 使用 Q 矩阵进行重投影
        points_3D = cv2.reprojectImageTo3D(disparity, self.Q, handleMissingValues=True)
        
        # 提取深度（Z 坐标）
        depth = points_3D[:, :, 2]
        
        # 转换单位：从 cm 到 m
        depth = depth / 100.0
        
        # 处理无效值
        depth[depth < 0] = 0  # 负深度无效
        depth[depth > 100] = 0  # 超过 100 米的深度视为无效
        depth[np.isinf(depth)] = 0
        depth[np.isnan(depth)] = 0
        
        # 转换为 uint16（毫米）- EGO-Planner 使用的格式
        depth_mm = (depth * 1000).astype(np.uint16)
        
        return depth_mm

# ==================== 图像采集类 ====================

class ImageCaptureThread(threading.Thread):
    """图像采集线程"""
    
    def __init__(self, regions=SCREENSHOT_REGIONS, interval=0.5):
        super().__init__()
        self.regions = regions
        self.interval = interval
        self.running = True
        self.latest_images = {}
        self.lock = threading.Lock()
        
    def run(self):
        """运行图像采集"""
        print("[Image Capture] Started")
        
        while self.running:
            try:
                images = {}
                for i, region in enumerate(self.regions):
                    screenshot = ImageGrab.grab(bbox=region)
                    # 转换为 numpy 数组
                    img_array = np.array(screenshot)
                    # PIL 使用 RGB，OpenCV 使用 BGR
                    img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
                    images[i] = img_array
                
                with self.lock:
                    self.latest_images = images
                
                time.sleep(self.interval)
                
            except Exception as e:
                print(f"[Image Capture] Error: {e}")
                time.sleep(1)
    
    def get_latest_images(self):
        """获取最新的图像"""
        with self.lock:
            return self.latest_images.copy()
    
    def stop(self):
        """停止采集"""
        self.running = False

# ==================== TCP 通信类 ====================

class TCPClient:
    """TCP 客户端，用于与 ROS 服务器通信"""
    
    def __init__(self, server_ip, server_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.connected = False
        self.receive_queue = Queue()
        
    def connect(self):
        """连接到服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            print(f"[TCP Client] Connected to {self.server_ip}:{self.server_port}")
            
            # 启动接收线程
            recv_thread = threading.Thread(target=self._receive_loop)
            recv_thread.daemon = True
            recv_thread.start()
            
            return True
        except Exception as e:
            print(f"[TCP Client] Connection failed: {e}")
            return False
    
    def _receive_loop(self):
        """接收数据的循环"""
        while self.connected:
            try:
                # 接收消息长度（4 字节）
                length_data = self.socket.recv(4)
                if not length_data:
                    break
                
                msg_length = struct.unpack('!I', length_data)[0]
                
                # 接收完整消息
                data = b''
                while len(data) < msg_length:
                    chunk = self.socket.recv(min(msg_length - len(data), 4096))
                    if not chunk:
                        break
                    data += chunk
                
                if len(data) == msg_length:
                    message = data.decode('utf-8')
                    self.receive_queue.put(message)
                    
            except Exception as e:
                print(f"[TCP Client] Receive error: {e}")
                break
        
        self.connected = False
    
    def send_depth_image(self, depth_image, timestamp):
        """
        发送深度图像到 ROS
        
        消息格式（JSON）：
        {
            "type": "DEPTH_IMAGE",
            "timestamp": 1234567890.123,
            "encoding": "16UC1",  // uint16, 1 channel
            "height": 540,
            "width": 960,
            "data": [base64 encoded binary data]
        }
        """
        if not self.connected:
            return False
        
        try:
            # 编码深度图像为 PNG
            success, encoded_img = cv2.imencode('.png', depth_image)
            if not success:
                return False
            
            # 转换为 base64
            import base64
            img_base64 = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            
            # 构造消息
            message = {
                'type': 'DEPTH_IMAGE',
                'timestamp': timestamp,
                'encoding': '16UC1',  # uint16, 1 channel
                'height': depth_image.shape[0],
                'width': depth_image.shape[1],
                'data': img_base64
            }
            
            # 发送
            msg_str = json.dumps(message)
            msg_bytes = msg_str.encode('utf-8')
            length_prefix = struct.pack('!I', len(msg_bytes))
            self.socket.sendall(length_prefix + msg_bytes)
            
            return True
        except Exception as e:
            print(f"[TCP Client] Send error: {e}")
            self.connected = False
            return False
    
    def get_received_message(self):
        """获取接收到的消息"""
        if not self.receive_queue.empty():
            return self.receive_queue.get()
        return None
    
    def close(self):
        """关闭连接"""
        self.connected = False
        if self.socket:
            self.socket.close()

# ==================== 主桥接类 ====================

class Arma3ROSBridge:
    """Arma 3 与 ROS 的深度图像桥接程序"""
    
    def __init__(self):
        self.image_capture = ImageCaptureThread()
        self.stereo_processor = StereoVisionProcessor()
        self.tcp_client = TCPClient(SERVER_IP, SERVER_PORT)
        self.running = False
        
    def start(self):
        """启动桥接程序"""
        print("=" * 60)
        print("Arma 3 <-> ROS Depth Image Bridge (EGO-Planner Compatible)")
        print("=" * 60)
        
        # 连接到 ROS 服务器
        print(f"\n[Bridge] Connecting to ROS server at {SERVER_IP}:{SERVER_PORT}...")
        if not self.tcp_client.connect():
            print("[Bridge] Failed to connect to ROS server. Exiting.")
            return
        
        # 启动图像采集
        print("[Bridge] Starting image capture...")
        self.image_capture.start()
        
        # 等待第一批图像
        time.sleep(2)
        
        # 主循环
        self.running = True
        print("\n[Bridge] Bridge started successfully!")
        print("[Bridge] Sending depth images to ROS (sensor_msgs/Image format)")
        print("[Bridge] Press Ctrl+C to stop.\n")
        
        try:
            frame_count = 0
            while self.running:
                frame_count += 1
                
                # 获取最新图像
                images = self.image_capture.get_latest_images()
                
                if len(images) >= 6:
                    # 处理立体视觉（使用图像 1 和 2 作为左右视图）
                    left_img = images.get(1)  # 中上
                    right_img = images.get(2)  # 右上
                    
                    if left_img is not None and right_img is not None:
                        # 计算视差
                        disparity = self.stereo_processor.compute_disparity(left_img, right_img)
                        
                        # 转换为深度图
                        depth_image = self.stereo_processor.disparity_to_depth(disparity)
                        
                        # 发送深度图像
                        timestamp = time.time()
                        success = self.tcp_client.send_depth_image(depth_image, timestamp)
                        
                        if success and frame_count % 10 == 0:
                            valid_pixels = np.count_nonzero(depth_image)
                            print(f"[Bridge] Frame {frame_count}: Sent depth image ({depth_image.shape[1]}x{depth_image.shape[0]}, {valid_pixels} valid pixels)")
                
                # 检查接收到的控制指令
                command = self.tcp_client.get_received_message()
                if command:
                    self.handle_command(command)
                
                time.sleep(0.1)  # 10 Hz
                
        except KeyboardInterrupt:
            print("\n[Bridge] Stopping...")
        finally:
            self.stop()
    
    def handle_command(self, command):
        """处理接收到的控制指令"""
        try:
            cmd_data = json.loads(command)
            cmd_type = cmd_data.get('type')
            
            if cmd_type == 'MOVE':
                x, y, z = cmd_data.get('x'), cmd_data.get('y'), cmd_data.get('z')
                print(f"[Bridge] Received MOVE command: ({x}, {y}, {z})")
                
            elif cmd_type == 'GOAL':
                x, y, z = cmd_data.get('x'), cmd_data.get('y'), cmd_data.get('z')
                print(f"[Bridge] Received GOAL command: ({x}, {y}, {z})")
                
        except Exception as e:
            print(f"[Bridge] Error handling command: {e}")
    
    def stop(self):
        """停止桥接程序"""
        self.running = False
        self.image_capture.stop()
        self.tcp_client.close()
        print("[Bridge] Stopped.")

# ==================== 主程序 ====================

def main():
    bridge = Arma3ROSBridge()
    bridge.start()

if __name__ == "__main__":
    main()
