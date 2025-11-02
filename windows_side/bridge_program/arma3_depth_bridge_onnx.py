#!/usr/bin/env python3
"""
Arma 3 Depth Estimation Bridge (ONNX + DirectML)
Windows 端深度估计桥接程序

功能：
1. 从 Arma 3 截取 RGB 图像
2. 使用 Depth Anything V2 ONNX 模型生成深度图像
3. 通过 TCP/IP 发送深度图像到 Linux ROS 节点
4. 接收来自 ROS 的控制指令并发送到 Arma 3

作者: Manus AI
日期: 2025-11-03
"""

import cv2
import numpy as np
import socket
import json
import time
import threading
import queue
from PIL import ImageGrab
import io
import base64
import onnxruntime as ort

class DepthAnythingV2:
    """Depth Anything V2 ONNX 推理类"""
    
    def __init__(self, model_path, input_size=(518, 518)):
        """
        初始化深度估计模型
        
        Args:
            model_path: ONNX 模型路径
            input_size: 模型输入尺寸 (width, height)
        """
        print(f"[DepthAnythingV2] 加载模型: {model_path}")
        
        # 创建 ONNX Runtime 会话（使用 DirectML 加速）
        providers = ['DmlExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        
        # 获取输入输出信息
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        self.input_size = input_size
        print(f"[DepthAnythingV2] 模型加载成功")
        print(f"[DepthAnythingV2] 输入尺寸: {input_size}")
        print(f"[DepthAnythingV2] 使用设备: {providers[0]}")
    
    def preprocess(self, image):
        """
        预处理输入图像
        
        Args:
            image: BGR 图像 (OpenCV 格式)
            
        Returns:
            preprocessed: 预处理后的张量 (1, 3, H, W)
        """
        # BGR -> RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Resize to model input size
        image = cv2.resize(image, self.input_size, interpolation=cv2.INTER_CUBIC)
        
        # Normalize to [0, 1]
        image = image.astype(np.float32) / 255.0
        
        # ImageNet normalization
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        image = (image - mean) / std
        
        # HWC -> CHW
        image = np.transpose(image, (2, 0, 1))
        
        # Add batch dimension
        image = np.expand_dims(image, axis=0)
        
        return image
    
    def postprocess(self, depth, target_size):
        """
        后处理深度图
        
        Args:
            depth: 模型输出的深度图 (1, 1, H, W)
            target_size: 目标尺寸 (width, height)
            
        Returns:
            depth_map: 深度图 (H, W), uint16 格式，单位：毫米
        """
        # Remove batch and channel dimensions
        depth = depth.squeeze()
        
        # Resize to target size
        depth = cv2.resize(depth, target_size, interpolation=cv2.INTER_LINEAR)
        
        # Normalize to [0, 1]
        depth_min = depth.min()
        depth_max = depth.max()
        depth = (depth - depth_min) / (depth_max - depth_min + 1e-8)
        
        # Convert to millimeters (0-65535 mm = 0-65.535 m)
        # 假设最大深度为 50 米
        depth = depth * 50000  # 50 meters in millimeters
        depth = depth.astype(np.uint16)
        
        return depth
    
    def infer(self, image):
        """
        推理深度图
        
        Args:
            image: BGR 图像 (OpenCV 格式)
            
        Returns:
            depth_map: 深度图 (H, W), uint16 格式
        """
        start_time = time.time()
        
        # 保存原始尺寸
        original_size = (image.shape[1], image.shape[0])  # (width, height)
        
        # 预处理
        input_tensor = self.preprocess(image)
        
        # 推理
        depth = self.session.run([self.output_name], {self.input_name: input_tensor})[0]
        
        # 后处理
        depth_map = self.postprocess(depth, original_size)
        
        inference_time = (time.time() - start_time) * 1000  # ms
        
        return depth_map, inference_time


class ImageCaptureThread(threading.Thread):
    """图像采集线程"""
    
    def __init__(self, capture_region, target_size=(640, 480), fps=10):
        """
        初始化图像采集线程
        
        Args:
            capture_region: 截图区域 (left, top, right, bottom)
            target_size: 目标尺寸 (width, height)
            fps: 采集帧率
        """
        super().__init__(daemon=True)
        self.capture_region = capture_region
        self.target_size = target_size
        self.fps = fps
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False
    
    def run(self):
        """运行图像采集循环"""
        self.running = True
        interval = 1.0 / self.fps
        
        print(f"[ImageCapture] 开始采集图像，区域: {self.capture_region}, FPS: {self.fps}")
        
        while self.running:
            start_time = time.time()
            
            try:
                # 截图
                screenshot = ImageGrab.grab(bbox=self.capture_region)
                
                # 转换为 OpenCV 格式
                frame = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
                
                # Resize
                if frame.shape[1] != self.target_size[0] or frame.shape[0] != self.target_size[1]:
                    frame = cv2.resize(frame, self.target_size)
                
                # 放入队列（非阻塞）
                try:
                    self.frame_queue.put(frame, block=False)
                except queue.Full:
                    # 队列满，丢弃旧帧
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put(frame, block=False)
                    except:
                        pass
                
            except Exception as e:
                print(f"[ImageCapture] 错误: {e}")
            
            # 控制帧率
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
    
    def get_frame(self):
        """获取最新的帧"""
        try:
            return self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return None
    
    def stop(self):
        """停止采集"""
        self.running = False


class Arma3DepthBridge:
    """Arma 3 深度估计桥接程序"""
    
    def __init__(self, config_path="config.json"):
        """
        初始化桥接程序
        
        Args:
            config_path: 配置文件路径
        """
        # 加载配置
        with open(config_path, 'r') as f:
            self.config = json.load(f)
        
        print("[Bridge] 初始化 Arma 3 深度估计桥接程序...")
        
        # 初始化深度估计模型
        model_path = self.config['depth_model']['model_path']
        input_size = tuple(self.config['depth_model']['input_size'])
        self.depth_model = DepthAnythingV2(model_path, input_size)
        
        # 初始化图像采集线程
        capture_region = tuple(self.config['capture']['region'])
        target_size = tuple(self.config['capture']['target_size'])
        fps = self.config['capture']['fps']
        self.capture_thread = ImageCaptureThread(capture_region, target_size, fps)
        
        # 初始化 TCP 客户端
        self.tcp_host = self.config['tcp']['host']
        self.tcp_port = self.config['tcp']['port']
        self.tcp_socket = None
        self.connected = False
        
        # 统计信息
        self.frame_count = 0
        self.total_inference_time = 0
        self.total_transmission_time = 0
    
    def connect_to_ros(self):
        """连接到 ROS 服务器"""
        print(f"[Bridge] 连接到 ROS 服务器 {self.tcp_host}:{self.tcp_port}...")
        
        while not self.connected:
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.connect((self.tcp_host, self.tcp_port))
                self.connected = True
                print(f"[Bridge] 已连接到 ROS 服务器")
            except Exception as e:
                print(f"[Bridge] 连接失败: {e}，5秒后重试...")
                time.sleep(5)
    
    def send_depth_image(self, depth_map, timestamp):
        """
        发送深度图像到 ROS
        
        Args:
            depth_map: 深度图 (H, W), uint16 格式
            timestamp: 时间戳
        """
        if not self.connected:
            return
        
        start_time = time.time()
        
        try:
            # 压缩深度图为 PNG（16-bit）
            success, encoded = cv2.imencode('.png', depth_map, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not success:
                print("[Bridge] PNG 编码失败")
                return
            
            # Base64 编码
            depth_b64 = base64.b64encode(encoded.tobytes()).decode('utf-8')
            
            # 构造消息
            message = {
                'type': 'DEPTH',
                'timestamp': timestamp,
                'width': depth_map.shape[1],
                'height': depth_map.shape[0],
                'encoding': '16UC1',  # ROS 深度图像编码
                'data': depth_b64
            }
            
            # 发送消息（JSON + 换行符）
            message_str = json.dumps(message) + '\n'
            self.tcp_socket.sendall(message_str.encode('utf-8'))
            
            transmission_time = (time.time() - start_time) * 1000  # ms
            self.total_transmission_time += transmission_time
            
        except Exception as e:
            print(f"[Bridge] 发送失败: {e}")
            self.connected = False
    
    def receive_control_commands(self):
        """接收来自 ROS 的控制指令（非阻塞）"""
        # TODO: 实现非阻塞接收和 Arma 3 控制
        pass
    
    def run(self):
        """运行主循环"""
        print("[Bridge] 启动桥接程序...")
        
        # 连接到 ROS
        self.connect_to_ros()
        
        # 启动图像采集线程
        self.capture_thread.start()
        
        print("[Bridge] 开始处理循环...")
        print("[Bridge] 按 Ctrl+C 退出")
        
        try:
            while True:
                # 获取最新帧
                frame = self.capture_thread.get_frame()
                if frame is None:
                    continue
                
                # 深度估计
                depth_map, inference_time = self.depth_model.infer(frame)
                
                # 发送深度图像
                timestamp = time.time()
                self.send_depth_image(depth_map, timestamp)
                
                # 更新统计
                self.frame_count += 1
                self.total_inference_time += inference_time
                
                # 每 30 帧打印一次统计
                if self.frame_count % 30 == 0:
                    avg_inference = self.total_inference_time / self.frame_count
                    avg_transmission = self.total_transmission_time / self.frame_count
                    total_latency = avg_inference + avg_transmission
                    
                    print(f"[Bridge] 统计 (最近 {self.frame_count} 帧):")
                    print(f"  - 平均推理时间: {avg_inference:.1f} ms")
                    print(f"  - 平均传输时间: {avg_transmission:.1f} ms")
                    print(f"  - 总延迟: {total_latency:.1f} ms")
                    print(f"  - 实际 FPS: {1000 / total_latency:.1f}")
                
        except KeyboardInterrupt:
            print("\n[Bridge] 收到退出信号...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("[Bridge] 清理资源...")
        
        # 停止图像采集
        self.capture_thread.stop()
        
        # 关闭 TCP 连接
        if self.tcp_socket:
            self.tcp_socket.close()
        
        print("[Bridge] 桥接程序已退出")


if __name__ == "__main__":
    bridge = Arma3DepthBridge("config.json")
    bridge.run()
