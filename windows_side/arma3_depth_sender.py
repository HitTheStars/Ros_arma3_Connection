"""
Arma 3 Depth Image Sender (Windows Side)
Windows 端深度图像生成与发送模块

功能：
1. 从 Arma 3 获取 RGB 图像（通过 TCP 或共享内存）
2. 使用 Depth Anything V2 ONNX 模型生成深度图像
3. 利用 DirectML 加速（RTX 4060 GPU）
4. 压缩深度图像并通过 TCP/IP 发送到 Linux ROS VM

硬件要求：
- Windows 10/11
- NVIDIA RTX 4060 或更高
- ONNX Runtime with DirectML

作者: Manus AI
日期: 2025-11-03
"""

import cv2
import numpy as np
import onnxruntime as ort
import socket
import json
import base64
import time
import argparse
from pathlib import Path


class DepthAnythingV2Estimator:
    """Depth Anything V2 深度估计器（ONNX + DirectML）"""
    
    def __init__(self, model_path, input_size=(518, 518)):
        """
        初始化深度估计器
        
        Args:
            model_path: ONNX 模型路径
            input_size: 输入图像尺寸 (height, width)
        """
        print(f"[DepthEstimator] 加载 ONNX 模型: {model_path}")
        
        # 创建 ONNX Runtime 会话（使用 DirectML 加速）
        providers = [
            ('DmlExecutionProvider', {
                'device_id': 0,
            }),
            'CPUExecutionProvider'
        ]
        
        self.session = ort.InferenceSession(
            model_path,
            providers=providers
        )
        
        # 打印使用的执行提供器
        print(f"[DepthEstimator] 使用的执行提供器: {self.session.get_providers()}")
        
        # 获取输入输出信息
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        self.input_size = input_size
        
        print(f"[DepthEstimator] 输入尺寸: {input_size}")
        print(f"[DepthEstimator] 模型加载完成")
    
    def preprocess(self, rgb_image):
        """
        预处理 RGB 图像
        
        Args:
            rgb_image: BGR 格式的输入图像 (H, W, 3)
            
        Returns:
            input_tensor: 预处理后的输入张量 (1, 3, H, W)
            original_size: 原始图像尺寸 (H, W)
        """
        original_size = rgb_image.shape[:2]
        
        # BGR -> RGB
        rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        # Resize
        resized = cv2.resize(rgb, (self.input_size[1], self.input_size[0]))
        
        # Normalize to [0, 1]
        normalized = resized.astype(np.float32) / 255.0
        
        # ImageNet normalization
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        normalized = (normalized - mean) / std
        
        # HWC -> CHW
        transposed = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        input_tensor = np.expand_dims(transposed, axis=0).astype(np.float32)
        
        return input_tensor, original_size
    
    def postprocess(self, depth_output, original_size):
        """
        后处理深度输出
        
        Args:
            depth_output: 模型输出的深度图 (1, 1, H, W) 或 (1, H, W)
            original_size: 原始图像尺寸 (H, W)
            
        Returns:
            depth_map: 16-bit 深度图 (H, W)，单位：毫米
        """
        # 去除 batch 和 channel 维度
        if len(depth_output.shape) == 4:
            depth = depth_output[0, 0]
        else:
            depth = depth_output[0]
        
        # Resize 到原始尺寸
        depth_resized = cv2.resize(depth, (original_size[1], original_size[0]))
        
        # 归一化到 [0, 1]
        depth_min = depth_resized.min()
        depth_max = depth_resized.max()
        depth_normalized = (depth_resized - depth_min) / (depth_max - depth_min + 1e-8)
        
        # 转换为 16-bit（模拟深度范围 0-65.535 米）
        # 假设最大深度为 100 米
        max_depth_mm = 100000  # 100 米 = 100,000 毫米
        depth_map = (depth_normalized * 65535).astype(np.uint16)
        
        return depth_map
    
    def estimate(self, rgb_image):
        """
        估计深度图
        
        Args:
            rgb_image: BGR 格式的输入图像
            
        Returns:
            depth_map: 16-bit 深度图 (H, W)
            inference_time: 推理时间（毫秒）
        """
        start_time = time.time()
        
        # 预处理
        input_tensor, original_size = self.preprocess(rgb_image)
        
        # 推理
        depth_output = self.session.run(
            [self.output_name],
            {self.input_name: input_tensor}
        )[0]
        
        # 后处理
        depth_map = self.postprocess(depth_output, original_size)
        
        inference_time = (time.time() - start_time) * 1000  # ms
        
        return depth_map, inference_time


class Arma3DepthSender:
    """Arma 3 深度图像发送器"""
    
    def __init__(self, model_path, linux_ip, linux_port=5555, input_source='camera'):
        """
        初始化深度发送器
        
        Args:
            model_path: ONNX 模型路径
            linux_ip: Linux ROS VM 的 IP 地址
            linux_port: Linux ROS VM 的 TCP 端口
            input_source: 输入源 ('camera', 'arma3', 'video', 'images')
        """
        print(f"[DepthSender] 初始化深度发送器...")
        
        # 深度估计器
        self.estimator = DepthAnythingV2Estimator(model_path)
        
        # TCP 客户端
        self.linux_ip = linux_ip
        self.linux_port = linux_port
        self.socket = None
        
        # 输入源
        self.input_source = input_source
        self.capture = None
        
        # 统计
        self.frame_count = 0
        self.total_inference_time = 0
        self.total_compression_time = 0
        self.total_transmission_time = 0
        
        print(f"[DepthSender] Linux IP: {linux_ip}:{linux_port}")
        print(f"[DepthSender] 输入源: {input_source}")
    
    def connect_to_linux(self):
        """连接到 Linux ROS VM"""
        print(f"[DepthSender] 连接到 Linux ROS VM: {self.linux_ip}:{self.linux_port}")
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.linux_ip, self.linux_port))
        
        print(f"[DepthSender] 连接成功")
    
    def init_input_source(self, source_param=0):
        """
        初始化输入源
        
        Args:
            source_param: 输入源参数
                - 'camera': 摄像头 ID（默认 0）
                - 'video': 视频文件路径
                - 'images': 图像文件夹路径
                - 'arma3': Arma 3 TCP 端口
        """
        if self.input_source == 'camera':
            print(f"[DepthSender] 打开摄像头: {source_param}")
            self.capture = cv2.VideoCapture(source_param)
            
            if not self.capture.isOpened():
                raise RuntimeError(f"无法打开摄像头 {source_param}")
            
            # 设置分辨率
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
        elif self.input_source == 'video':
            print(f"[DepthSender] 打开视频文件: {source_param}")
            self.capture = cv2.VideoCapture(source_param)
            
            if not self.capture.isOpened():
                raise RuntimeError(f"无法打开视频文件 {source_param}")
        
        elif self.input_source == 'arma3':
            print(f"[DepthSender] 等待 Arma 3 连接（端口 {source_param}）...")
            # TODO: 实现 Arma 3 TCP 服务器
            raise NotImplementedError("Arma 3 输入源尚未实现")
        
        else:
            raise ValueError(f"不支持的输入源: {self.input_source}")
    
    def get_next_frame(self):
        """
        获取下一帧 RGB 图像
        
        Returns:
            rgb_image: BGR 格式的图像，失败返回 None
        """
        if self.input_source in ['camera', 'video']:
            ret, frame = self.capture.read()
            if not ret:
                return None
            return frame
        
        elif self.input_source == 'arma3':
            # TODO: 从 Arma 3 接收图像
            raise NotImplementedError("Arma 3 输入源尚未实现")
        
        return None
    
    def compress_depth(self, depth_map):
        """
        压缩深度图像（PNG 格式）
        
        Args:
            depth_map: 16-bit 深度图 (H, W)
            
        Returns:
            compressed_bytes: 压缩后的字节数据
            compression_time: 压缩时间（毫秒）
        """
        start_time = time.time()
        
        # PNG 编码（无损压缩）
        success, encoded = cv2.imencode('.png', depth_map, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        
        if not success:
            raise RuntimeError("PNG 编码失败")
        
        compressed_bytes = encoded.tobytes()
        compression_time = (time.time() - start_time) * 1000  # ms
        
        return compressed_bytes, compression_time
    
    def send_depth(self, depth_map, timestamp):
        """
        发送深度图像到 Linux
        
        Args:
            depth_map: 16-bit 深度图 (H, W)
            timestamp: 时间戳（秒）
            
        Returns:
            transmission_time: 传输时间（毫秒）
        """
        start_time = time.time()
        
        # 压缩
        compressed_bytes, compression_time = self.compress_depth(depth_map)
        
        # Base64 编码
        depth_b64 = base64.b64encode(compressed_bytes).decode('utf-8')
        
        # 构造消息
        message = {
            'type': 'DEPTH',
            'timestamp': timestamp,
            'width': depth_map.shape[1],
            'height': depth_map.shape[0],
            'encoding': '16UC1',
            'data': depth_b64
        }
        
        # JSON 序列化
        message_json = json.dumps(message) + '\n'
        message_bytes = message_json.encode('utf-8')
        
        # 发送
        self.socket.sendall(message_bytes)
        
        transmission_time = (time.time() - start_time) * 1000  # ms
        
        return transmission_time, compression_time
    
    def run(self, fps=10, duration=None):
        """
        运行主循环
        
        Args:
            fps: 目标帧率
            duration: 运行时长（秒），None 表示无限运行
        """
        print(f"[DepthSender] 启动深度发送器...")
        print(f"[DepthSender] 目标帧率: {fps} FPS")
        
        # 连接到 Linux
        self.connect_to_linux()
        
        # 初始化输入源
        self.init_input_source()
        
        # 计算帧间隔
        frame_interval = 1.0 / fps
        
        print(f"[DepthSender] 开始发送深度图像...")
        print(f"[DepthSender] 按 Ctrl+C 退出")
        
        start_time = time.time()
        
        try:
            while True:
                loop_start = time.time()
                
                # 检查是否超时
                if duration is not None and (loop_start - start_time) > duration:
                    print(f"[DepthSender] 达到运行时长 {duration} 秒，退出")
                    break
                
                # 获取 RGB 图像
                rgb_image = self.get_next_frame()
                
                if rgb_image is None:
                    print(f"[DepthSender] 输入结束")
                    break
                
                # 生成深度图
                depth_map, inference_time = self.estimator.estimate(rgb_image)
                
                # 发送深度图
                timestamp = time.time()
                transmission_time, compression_time = self.send_depth(depth_map, timestamp)
                
                # 更新统计
                self.frame_count += 1
                self.total_inference_time += inference_time
                self.total_compression_time += compression_time
                self.total_transmission_time += transmission_time
                
                # 每 30 帧打印一次统计
                if self.frame_count % 30 == 0:
                    avg_inference = self.total_inference_time / self.frame_count
                    avg_compression = self.total_compression_time / self.frame_count
                    avg_transmission = self.total_transmission_time / self.frame_count
                    avg_total = avg_inference + avg_compression + avg_transmission
                    
                    print(f"\n[DepthSender] 统计 (最近 {self.frame_count} 帧):")
                    print(f"  - 平均推理时间: {avg_inference:.1f} ms")
                    print(f"  - 平均压缩时间: {avg_compression:.1f} ms")
                    print(f"  - 平均传输时间: {avg_transmission:.1f} ms")
                    print(f"  - 平均总时间: {avg_total:.1f} ms")
                    print(f"  - 实际 FPS: {1000 / avg_total:.1f}")
                
                # 控制帧率
                loop_time = time.time() - loop_start
                sleep_time = frame_interval - loop_time
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print(f"\n[DepthSender] 用户中断")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print(f"[DepthSender] 清理资源...")
        
        if self.capture:
            self.capture.release()
        
        if self.socket:
            self.socket.close()
        
        print(f"[DepthSender] 退出")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Arma 3 Depth Image Sender')
    
    parser.add_argument('--model', type=str, required=True,
                        help='ONNX 模型路径')
    parser.add_argument('--linux-ip', type=str, required=True,
                        help='Linux ROS VM 的 IP 地址')
    parser.add_argument('--linux-port', type=int, default=5555,
                        help='Linux ROS VM 的 TCP 端口（默认 5555）')
    parser.add_argument('--input', type=str, default='camera',
                        choices=['camera', 'video', 'arma3'],
                        help='输入源类型（默认 camera）')
    parser.add_argument('--source', type=str, default='0',
                        help='输入源参数（摄像头 ID、视频路径等）')
    parser.add_argument('--fps', type=int, default=10,
                        help='目标帧率（默认 10）')
    parser.add_argument('--duration', type=float, default=None,
                        help='运行时长（秒），不指定则无限运行')
    
    args = parser.parse_args()
    
    # 检查模型文件
    if not Path(args.model).exists():
        print(f"[ERROR] 模型文件不存在: {args.model}")
        return
    
    # 解析输入源参数
    if args.input == 'camera':
        source_param = int(args.source)
    else:
        source_param = args.source
    
    # 创建发送器
    sender = Arma3DepthSender(
        model_path=args.model,
        linux_ip=args.linux_ip,
        linux_port=args.linux_port,
        input_source=args.input
    )
    
    # 初始化输入源
    sender.init_input_source(source_param)
    
    # 运行
    sender.run(fps=args.fps, duration=args.duration)


if __name__ == '__main__':
    main()
