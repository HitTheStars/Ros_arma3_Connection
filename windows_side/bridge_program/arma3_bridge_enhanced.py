#!/usr/bin/env python3
"""
Enhanced Arma 3 Bridge Program
Integrates the screenshot.py functionality with TCP/IP communication to ROS
"""

import socket
import struct
import time
import threading
import json
from PIL import ImageGrab
import io
import os

class Arma3BridgeEnhanced:
    """
    Enhanced bridge program that captures images from Arma 3 and sends them to ROS
    Integrates the multi-view screenshot functionality
    """
    
    def __init__(self, server_ip='192.168.1.100', server_port=5555):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.running = False
        
        # Screenshot regions (from screenshot.py)
        self.regions = [
            (0, 255, 455, 511),      # Region 1
            (455, 255, 911, 511),    # Region 2
            (911, 255, 1365, 511),   # Region 3
            (0, 511, 455, 767),      # Region 4
            (455, 511, 911, 767),    # Region 5
            (911, 511, 1365, 767)    # Region 6
        ]
        
        # UAV state (will be updated from Arma 3)
        self.uav_states = [
            {'id': i, 'pos': [0, 0, 0], 'vel': [0, 0, 0], 'yaw': 0}
            for i in range(6)
        ]
        
        # Create output folder for local saving (optional)
        self.output_folder = 'screenshots'
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
    
    def connect(self):
        """Connect to ROS server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            print(f"Connected to ROS server at {self.server_ip}:{self.server_port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def capture_and_send_images(self):
        """Capture images from all 6 regions and send to ROS"""
        images_data = []
        
        for i, region in enumerate(self.regions):
            try:
                # Capture screenshot
                screenshot = ImageGrab.grab(bbox=region)
                
                # Convert to JPEG and compress
                img_buffer = io.BytesIO()
                screenshot.save(img_buffer, format='JPEG', quality=85)
                img_bytes = img_buffer.getvalue()
                
                images_data.append({
                    'camera_id': i,
                    'size': len(img_bytes),
                    'data': img_bytes
                })
                
                # Optional: Save locally for debugging
                # screenshot.save(os.path.join(self.output_folder, f'cam_{i}.jpg'))
                
            except Exception as e:
                print(f"Error capturing region {i}: {e}")
                continue
        
        # Send all images to ROS
        if images_data:
            self.send_images_packet(images_data)
    
    def send_images_packet(self, images_data):
        """
        Send images packet to ROS
        Packet format:
        - Header: "IMGS" (4 bytes)
        - Timestamp: double (8 bytes)
        - Num cameras: int (4 bytes)
        - For each camera:
          - Camera ID: int (4 bytes)
          - Image size: int (4 bytes)
          - Image data: bytes (variable)
        """
        if not self.socket:
            return
        
        try:
            # Build packet
            packet = bytearray()
            
            # Header
            packet.extend(b'IMGS')
            
            # Timestamp
            packet.extend(struct.pack('d', time.time()))
            
            # Number of cameras
            packet.extend(struct.pack('I', len(images_data)))
            
            # Each camera's data
            for img_data in images_data:
                packet.extend(struct.pack('I', img_data['camera_id']))
                packet.extend(struct.pack('I', img_data['size']))
                packet.extend(img_data['data'])
            
            # Send packet
            self.socket.sendall(packet)
            
        except Exception as e:
            print(f"Error sending images: {e}")
    
    def send_status_packet(self):
        """
        Send UAV status packet to ROS
        Packet format:
        - Header: "STAT" (4 bytes)
        - Timestamp: double (8 bytes)
        - Num UAVs: int (4 bytes)
        - For each UAV:
          - UAV ID: int (4 bytes)
          - Position: 3 floats (12 bytes)
          - Velocity: 3 floats (12 bytes)
          - Yaw: float (4 bytes)
        """
        if not self.socket:
            return
        
        try:
            packet = bytearray()
            
            # Header
            packet.extend(b'STAT')
            
            # Timestamp
            packet.extend(struct.pack('d', time.time()))
            
            # Number of UAVs
            packet.extend(struct.pack('I', len(self.uav_states)))
            
            # Each UAV's state
            for uav in self.uav_states:
                packet.extend(struct.pack('I', uav['id']))
                packet.extend(struct.pack('fff', *uav['pos']))
                packet.extend(struct.pack('fff', *uav['vel']))
                packet.extend(struct.pack('f', uav['yaw']))
            
            # Send packet
            self.socket.sendall(packet)
            
        except Exception as e:
            print(f"Error sending status: {e}")
    
    def receive_commands(self):
        """Receive control commands from ROS"""
        if not self.socket:
            return None
        
        try:
            # Receive header (4 bytes)
            header = self.socket.recv(4)
            if not header:
                return None
            
            if header == b'MOVE':
                # Receive movement command
                # Format: UAV ID (4 bytes) + X, Y, Z (12 bytes)
                data = self.socket.recv(16)
                uav_id, x, y, z = struct.unpack('Ifff', data)
                
                return {
                    'type': 'MOVE',
                    'uav_id': uav_id,
                    'target': [x, y, z]
                }
            
        except Exception as e:
            print(f"Error receiving commands: {e}")
            return None
    
    def main_loop(self):
        """Main loop for capturing and sending data"""
        self.running = True
        
        # Image capture interval (seconds)
        image_interval = 0.5  # 2 Hz
        status_interval = 0.1  # 10 Hz
        
        last_image_time = 0
        last_status_time = 0
        
        while self.running:
            current_time = time.time()
            
            # Capture and send images
            if current_time - last_image_time >= image_interval:
                self.capture_and_send_images()
                last_image_time = current_time
            
            # Send status
            if current_time - last_status_time >= status_interval:
                self.send_status_packet()
                last_status_time = current_time
            
            # Receive commands (non-blocking)
            # In a real implementation, this should be in a separate thread
            
            time.sleep(0.01)  # 100 Hz loop
    
    def run(self):
        """Run the bridge program"""
        print("Starting Enhanced Arma 3 Bridge...")
        
        if not self.connect():
            print("Failed to connect to ROS server. Exiting.")
            return
        
        try:
            self.main_loop()
        except KeyboardInterrupt:
            print("\nStopping bridge...")
        finally:
            if self.socket:
                self.socket.close()
            print("Bridge stopped.")

if __name__ == '__main__':
    # Configuration
    SERVER_IP = '192.168.1.100'  # Change to your Linux VM IP
    SERVER_PORT = 5555
    
    # Create and run bridge
    bridge = Arma3BridgeEnhanced(SERVER_IP, SERVER_PORT)
    bridge.run()
