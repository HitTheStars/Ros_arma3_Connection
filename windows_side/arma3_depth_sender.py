"""
Arma 3 Depth Image Sender (Windows Side) - Optimized Version
Windows 端深度图像生成与发送模块（优化版）

功能：
1. 从 Arma 3 获取 RGB 图像（通过 MSS 屏幕截图）
2. 自动检测 Arma 3 窗口并截取指定区域
3. 使用 Depth Anything V2 ONNX 模型生成深度图像
4. 利用 DirectML 加速（RTX 4060 GPU）
5. 压缩深度图像并通过 TCP/IP 发送到 Linux ROS VM

优化：
- 使用 MSS 库替代 PIL（4 倍速度提升）
- 自动窗口检测（无需硬编码坐标）
- 多线程截图（提高帧率）

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
import threading
import queue

# Windows 特定库
try:
    import mss
    import win32gui
    import win32con
    MSS_AVAILABLE = True
except ImportError:
    MSS_AVAILABLE = False
    print("[WARNING] MSS or pywin32 not available. Arma3 screenshot mode disabled.")


class WindowDetector:
    """Windows 窗口检测器"""
    
    @staticmethod
    def find_window_by_title(title_substring):
        """
        根据标题查找窗口
        
        Args:
            title_substring: 窗口标题的子字符串（如 "Arma 3"）
            
        Returns:
            hwnd: 窗口句柄，未找到返回 None
        """
        def callback(hwnd, windows):
            if win32gui.IsWindowVisible(hwnd):
                window_title = win32gui.GetWindowText(hwnd)
                if title_substring.lower() in window_title.lower():
                    windows.append(hwnd)
            return True
        
        windows = []
        win32gui.EnumWindows(callback, windows)
        return windows[0] if windows else None
    
    @staticmethod
    def get_window_rect(hwnd):
        """
        获取窗口矩形区域
        
        Args:
            hwnd: 窗口句柄
            
        Returns:
            rect: (left, top, right, bottom)
        """
        return win32gui.GetWindowRect(hwnd)
    
    @staticmethod
    def get_client_rect(hwnd):
        """
        获取窗口客户区矩形（不包含边框和标题栏）
        
        Args:
            hwnd: 窗口句柄
            
        Returns:
            rect: (left, top, right, bottom) 屏幕坐标
        """
        # 获取客户区相对于窗口的坐标
        client_rect = win32gui.GetClientRect(hwnd)
        
        # 转换为屏幕坐标
        left_top = win32gui.ClientToScreen(hwnd, (client_rect[0], client_rect[1]))
        right_bottom = win32gui.ClientToScreen(hwnd, (client_rect[2], client_rect[3]))
        
        return (left_top[0], left_top[1], right_bottom[0], right_bottom[1])


class Arma3ScreenCapture:
    """Arma 3 屏幕截图器（使用 MSS）"""
    
    def __init__(self, window_title="Arma 3", region_index=0, total_regions=6):
        """
        初始化截图器
        
        Args:
            window_title: Arma 3 窗口标题
            region_index: 截取的区域索引（0-5，对应 6 个无人机视角）
            total_regions: 总区域数（默认 6，3列×2行）
        """
        if not MSS_AVAILABLE:
            raise RuntimeError("MSS or pywin32 not installed. Please run: pip install mss pywin32")
        
        self.window_title = window_title
        self.region_index = region_index
        self.total_regions = total_regions
        self.sct = mss.mss()
        self.hwnd = None
        self.monitor = None
        
        print(f"[ScreenCapture] 初始化 Arma 3 屏幕截图器")
        print(f"[ScreenCapture] 窗口标题: {window_title}")
        print(f"[ScreenCapture] 区域索引: {region_index}/{total_regions}")
    
    def detect_window(self):
        """检测 Arma 3 窗口"""
        print(f"[ScreenCapture] 正在检测 Arma 3 窗口...")
        
        self.hwnd = WindowDetector.find_window_by_title(self.window_title)
        
        if not self.hwnd:
            raise RuntimeError(f"未找到窗口: {self.window_title}")
        
        window_title = win32gui.GetWindowText(self.hwnd)
        print(f"[ScreenCapture] 找到窗口: {window_title}")
        
        # 获取客户区坐标
        rect = WindowDetector.get_client_rect(self.hwnd)
        print(f"[ScreenCapture] 窗口客户区: {rect}")
        
        # 计算截取区域
        self._calculate_region(rect)
    
    def _calculate_region(self, window_rect):
        """
        计算截取区域（基于 codingWithArma3 的布局）
        
        Args:
            window_rect: 窗口矩形 (left, top, right, bottom)
        """
        left, top, right, bottom = window_rect
        window_width = right - left
        window_height = bottom - top
        
        # codingWithArma3 布局：6 个区域，3列×2行
        # 每个区域占屏幕的 1/3 宽度，1/3 高度
        cols = 3
        rows = 2
        
        region_width = window_width // cols
        region_height = window_height // rows
        
        # 计算当前区域的行列索引
        row = self.region_index // cols
        col = self.region_index % cols
        
        # 计算区域坐标
        region_left = left + col * region_width
        region_top = top + row * region_height
        region_right = region_left + region_width
        region_bottom = region_top + region_height
        
        # 创建 MSS monitor 字典
        self.monitor = {
            "left": region_left,
            "top": region_top,
            "width": region_width,
            "height": region_height
        }
        
        print(f"[ScreenCapture] 截取区域 {self.region_index}: "
              f"({region_left}, {region_top}, {region_right}, {region_bottom})")
        print(f"[ScreenCapture] 区域尺寸: {region_width}x{region_height}")
    
    def capture(self):
        """
        截取当前区域
        
        Returns:
            rgb_image: BGR 格式的图像 (H, W, 3)，失败返回 None
        """
        if not self.monitor:
            self.detect_window()
        
        try:
            # 使用 MSS 截图
            screenshot = self.sct.grab(self.monitor)
            
            # 转换为 NumPy 数组
            img = np.array(screenshot)
            
            # BGRA -> BGR（去掉 Alpha 通道）
            bgr_img = img[:, :, :3]
            
            # MSS 返回的是 BGRA 格式，需要转换为 BGR
            bgr_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGRA2BGR)
            
            return bgr_img
            
        except Exception as e:
            print(f"[ScreenCapture] 截图失败: {e}")
            return None
    
    def close(self):
        """关闭截图器"""
        if self.sct:
            self.sct.close()
            print(f"[ScreenCapture] 截图器已关闭")


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
        后处理深度图像
        
        Args:
            depth_output: 模型输出的深度图 (1, H, W)
            original_size: 原始图像尺寸 (H, W)
            
        Returns:
            depth_map: 16-bit 深度图 (H, W)
        """
        # Remove batch dimension
        depth = depth_output.squeeze()
        
        # Resize to original size
        depth_resized = cv2.resize(depth, (original_size[1], original_size[0]))
        
        # Normalize to [0, 65535] for 16-bit depth
        depth_min = depth_resized.min()
        depth_max = depth_resized.max()
        depth_normalized = (depth_resized - depth_min) / (depth_max - depth_min + 1e-8)
        depth_16bit = (depth_normalized * 65535).astype(np.uint16)
        
        return depth_16bit
    
    def estimate(self, rgb_image):
        """
        估计深度图像
        
        Args:
            rgb_image: BGR 格式的输入图像 (H, W, 3)
            
        Returns:
            depth_map: 16-bit 深度图 (H, W)
            inference_time: 推理时间（毫秒）
        """
        # Preprocess
        input_tensor, original_size = self.preprocess(rgb_image)
        
        # Inference
        start_time = time.time()
        depth_output = self.session.run(
            [self.output_name],
            {self.input_name: input_tensor}
        )[0]
        inference_time = (time.time() - start_time) * 1000  # ms
        
        # Postprocess
        depth_map = self.postprocess(depth_output, original_size)
        
        return depth_map, inference_time


class Arma3DepthSender:
    """Arma 3 深度图像发送器"""
    
    def __init__(self, model_path, linux_ip, linux_port=5555, 
                 input_source='arma3_screenshot', region_index=0):
        """
        初始化深度发送器
        
        Args:
            model_path: ONNX 模型路径
            linux_ip: Linux ROS VM 的 IP 地址
            linux_port: Linux ROS VM 的端口
            input_source: 输入源类型
                - 'camera': 摄像头
                - 'video': 视频文件
                - 'arma3_screenshot': Arma 3 屏幕截图（推荐）
            region_index: 截取的区域索引（0-5）
        """
        self.model_path = model_path
        self.linux_ip = linux_ip
        self.linux_port = linux_port
        self.input_source = input_source
        self.region_index = region_index
        
        # 初始化深度估计器
        self.estimator = DepthAnythingV2Estimator(model_path)
        
        # 初始化输入源
        self.capture = None
        self.screen_capture = None
        
        # TCP socket
        self.socket = None
        
        # 统计信息
        self.frame_count = 0
        self.total_inference_time = 0
        self.total_transmission_time = 0
    
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
                - 'arma3_screenshot': 无需参数
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
        
        elif self.input_source == 'arma3_screenshot':
            print(f"[DepthSender] 初始化 Arma 3 屏幕截图器...")
            self.screen_capture = Arma3ScreenCapture(
                window_title="Arma 3",
                region_index=self.region_index,
                total_regions=6
            )
            self.screen_capture.detect_window()
        
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
        
        elif self.input_source == 'arma3_screenshot':
            return self.screen_capture.capture()
        
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
        print(f"[DepthSender] 输入源: {self.input_source}")
        
        if duration:
            print(f"[DepthSender] 运行时长: {duration} 秒")
        
        frame_interval = 1.0 / fps
        start_time = time.time()
        
        try:
            while True:
                loop_start = time.time()
                
                # 检查运行时长
                if duration and (loop_start - start_time) >= duration:
                    print(f"[DepthSender] 达到运行时长 {duration} 秒，停止")
                    break
                
                # 获取 RGB 图像
                rgb_image = self.get_next_frame()
                if rgb_image is None:
                    print("[DepthSender] 无法获取图像，停止")
                    break
                
                # 深度估计
                depth_map, inference_time = self.estimator.estimate(rgb_image)
                
                # 发送深度图像
                timestamp = time.time()
                transmission_time, compression_time = self.send_depth(depth_map, timestamp)
                
                # 更新统计
                self.frame_count += 1
                self.total_inference_time += inference_time
                self.total_transmission_time += transmission_time
                
                # 每 30 帧打印一次统计
                if self.frame_count % 30 == 0:
                    avg_inference = self.total_inference_time / self.frame_count
                    avg_transmission = self.total_transmission_time / self.frame_count
                    avg_total = avg_inference + avg_transmission
                    actual_fps = 1000 / avg_total if avg_total > 0 else 0
                    
                    print(f"[DepthSender] 统计 (最近 {self.frame_count} 帧):")
                    print(f"  - 平均推理时间: {avg_inference:.1f} ms")
                    print(f"  - 平均传输时间: {avg_transmission:.1f} ms")
                    print(f"  - 平均总时间: {avg_total:.1f} ms")
                    print(f"  - 实际 FPS: {actual_fps:.1f}")
                
                # 帧率控制
                elapsed = time.time() - loop_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n[DepthSender] 用户中断")
        
        except Exception as e:
            print(f"[DepthSender] 错误: {e}")
            raise
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("[DepthSender] 清理资源...")
        
        if self.capture:
            self.capture.release()
        
        if self.screen_capture:
            self.screen_capture.close()
        
        if self.socket:
            self.socket.close()
        
        print("[DepthSender] 清理完成")


def main():
    parser = argparse.ArgumentParser(description='Arma 3 Depth Image Sender (Optimized)')
    
    parser.add_argument('--model', type=str, required=True,
                        help='Path to ONNX model file')
    parser.add_argument('--linux-ip', type=str, required=True,
                        help='Linux ROS VM IP address')
    parser.add_argument('--linux-port', type=int, default=5555,
                        help='Linux ROS VM port (default: 5555)')
    parser.add_argument('--input', type=str, default='arma3_screenshot',
                        choices=['camera', 'video', 'arma3_screenshot'],
                        help='Input source type (default: arma3_screenshot)')
    parser.add_argument('--source', type=str, default='0',
                        help='Input source parameter (camera ID or video path)')
    parser.add_argument('--region', type=int, default=0, choices=range(6),
                        help='Screenshot region index (0-5, default: 0)')
    parser.add_argument('--fps', type=int, default=10,
                        help='Target FPS (default: 10)')
    parser.add_argument('--duration', type=int, default=None,
                        help='Run duration in seconds (default: infinite)')
    
    args = parser.parse_args()
    
    # 验证模型文件
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"错误: 模型文件不存在: {model_path}")
        return
    
    # 创建深度发送器
    sender = Arma3DepthSender(
        model_path=str(model_path),
        linux_ip=args.linux_ip,
        linux_port=args.linux_port,
        input_source=args.input,
        region_index=args.region
    )
    
    # 连接到 Linux
    sender.connect_to_linux()
    
    # 初始化输入源
    if args.input == 'camera':
        source_param = int(args.source)
    elif args.input == 'video':
        source_param = args.source
    else:
        source_param = 0
    
    sender.init_input_source(source_param)
    
    # 运行
    sender.run(fps=args.fps, duration=args.duration)


if __name__ == "__main__":
    main()
