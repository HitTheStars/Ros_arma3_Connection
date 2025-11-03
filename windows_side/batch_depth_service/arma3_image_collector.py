"""
Arma 3 Image Collector
Arma 3 图像采集器

功能：
1. 监控 Arma 3 Screenshots 文件夹
2. 自动识别 UAV ID
3. 将图像发送到批处理深度估计服务

使用方法：
python arma3_image_collector.py [screenshots_dir] [server_host] [server_port]
"""

import os
import time
import cv2
import base64
import socket
import json
import sys
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class ImageCollector(FileSystemEventHandler):
    def __init__(self, screenshots_dir, server_host, server_port, num_uavs=6):
        """
        初始化图像采集器
        
        Args:
            screenshots_dir: Screenshots 文件夹路径
            server_host: 深度估计服务的主机地址
            server_port: 深度估计服务的端口
            num_uavs: UAV 数量
        """
        self.screenshots_dir = screenshots_dir
        self.server_host = server_host
        self.server_port = server_port
        self.num_uavs = num_uavs
        
        # 连接到深度估计服务
        self.socket = None
        self.connect()
        
        # 统计信息
        self.images_sent = 0
        self.start_time = time.time()
    
    def connect(self):
        """连接到深度估计服务"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_host, self.server_port))
                print(f"✓ Connected to depth estimation service at {self.server_host}:{self.server_port}")
                return True
            except Exception as e:
                print(f"Connection attempt {attempt + 1}/{max_retries} failed: {e}")
                if attempt < max_retries - 1:
                    print(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print("\nFailed to connect to depth estimation service.")
                    print("Please ensure the service is running:")
                    print("  python batch_depth_service.py")
                    sys.exit(1)
    
    def on_created(self, event):
        """当新文件创建时触发"""
        if event.is_directory:
            return
        
        filepath = event.src_path
        filename = os.path.basename(filepath)
        
        # 检查是否是 UAV 图像文件（格式：uav0_frame.png, uav1_frame.png, ...）
        if filename.startswith("uav") and filename.endswith(".png"):
            # 提取 UAV ID
            try:
                # 从文件名提取 ID：uav0_frame.png -> 0
                uav_id_str = filename.split("_")[0][3:]  # 去掉 "uav" 前缀
                uav_id = int(uav_id_str)
                
                if uav_id < 0 or uav_id >= self.num_uavs:
                    print(f"⚠ Invalid UAV ID: {uav_id} (expected 0-{self.num_uavs-1})")
                    return
                    
            except (ValueError, IndexError):
                print(f"⚠ Failed to parse UAV ID from filename: {filename}")
                return
            
            # 等待文件写入完成
            time.sleep(0.05)
            
            # 读取图像
            try:
                image = cv2.imread(filepath)
                if image is None:
                    print(f"⚠ Failed to read image: {filepath}")
                    return
            except Exception as e:
                print(f"⚠ Error reading image {filepath}: {e}")
                return
            
            # 编码为 PNG
            try:
                _, image_png = cv2.imencode('.png', image)
                image_b64 = base64.b64encode(image_png).decode('utf-8')
            except Exception as e:
                print(f"⚠ Error encoding image: {e}")
                return
            
            # 构造消息
            message = json.dumps({
                'uav_id': uav_id,
                'image': image_b64,
                'timestamp': time.time()
            })
            
            # 发送
            try:
                # 发送消息长度（4 字节）
                msg_len = len(message)
                self.socket.sendall(msg_len.to_bytes(4, byteorder='big'))
                # 发送消息内容
                self.socket.sendall(message.encode('utf-8'))
                
                self.images_sent += 1
                elapsed = time.time() - self.start_time
                rate = self.images_sent / elapsed if elapsed > 0 else 0
                
                print(f"✓ Sent image from UAV {uav_id} "
                      f"(size: {image.shape}, "
                      f"total: {self.images_sent}, "
                      f"rate: {rate:.2f} img/s)")
                
            except Exception as e:
                print(f"✗ Error sending image from UAV {uav_id}: {e}")
                print("Attempting to reconnect...")
                try:
                    self.socket.close()
                except:
                    pass
                self.connect()
            
            # 删除已处理的文件
            try:
                os.remove(filepath)
            except Exception as e:
                print(f"⚠ Failed to delete file {filepath}: {e}")
    
    def run(self):
        """启动文件监控"""
        # 检查目录是否存在
        if not os.path.exists(self.screenshots_dir):
            print(f"\nError: Screenshots directory not found:")
            print(f"  {self.screenshots_dir}")
            print("\nPlease create the directory or check the path.")
            print("Default path: ~/Documents/Arma 3/Screenshots")
            sys.exit(1)
        
        print("="*60)
        print("ARMA 3 IMAGE COLLECTOR")
        print("="*60)
        print(f"Configuration:")
        print(f"  Screenshots dir: {self.screenshots_dir}")
        print(f"  Server: {self.server_host}:{self.server_port}")
        print(f"  Number of UAVs: {self.num_uavs}")
        print("="*60)
        
        observer = Observer()
        observer.schedule(self, self.screenshots_dir, recursive=False)
        observer.start()
        
        print(f"\n✓ Monitoring {self.screenshots_dir}")
        print("Waiting for images from Arma 3...")
        print("Press Ctrl+C to stop.\n")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nStopping...")
            observer.stop()
        
        observer.join()
        
        # 关闭连接
        try:
            self.socket.close()
        except:
            pass
        
        print("Image collector stopped.")

def main():
    # 默认配置
    screenshots_dir = os.path.expanduser("~/Documents/Arma 3/Screenshots")
    server_host = "localhost"
    server_port = 5557
    num_uavs = 6
    
    # 从命令行参数读取配置
    if len(sys.argv) > 1:
        screenshots_dir = sys.argv[1]
    if len(sys.argv) > 2:
        server_host = sys.argv[2]
    if len(sys.argv) > 3:
        server_port = int(sys.argv[3])
    if len(sys.argv) > 4:
        num_uavs = int(sys.argv[4])
    
    # 展开用户目录
    screenshots_dir = os.path.expanduser(screenshots_dir)
    
    try:
        collector = ImageCollector(
            screenshots_dir=screenshots_dir,
            server_host=server_host,
            server_port=server_port,
            num_uavs=num_uavs
        )
        
        collector.run()
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
