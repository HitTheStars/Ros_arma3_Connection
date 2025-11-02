#!/usr/bin/env python3
"""
Arma 3 Bridge Program
This program runs on Windows and acts as a bridge between Arma 3 and ROS (Linux VM).
It captures images from Arma 3, compresses them, and sends them to ROS via TCP/IP.
It also receives control commands from ROS and forwards them to Arma 3.
"""

import socket
import struct
import time
import json
import threading
import queue
from PIL import ImageGrab
import io
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class Arma3Bridge:
    def __init__(self, ros_host='192.168.1.100', ros_port=5555):
        """
        Initialize the Arma 3 Bridge
        
        Args:
            ros_host: IP address of the Linux VM running ROS
            ros_port: Port number for TCP communication
        """
        self.ros_host = ros_host
        self.ros_port = ros_port
        self.socket = None
        self.connected = False
        
        # Image capture configuration
        self.regions = [
            (0, 255, 455, 511),      # Region 1
            (455, 255, 911, 511),    # Region 2
            (911, 255, 1365, 511),   # Region 3
            (0, 511, 455, 767),      # Region 4
            (455, 511, 911, 767),    # Region 5
            (911, 511, 1365, 767)    # Region 6
        ]
        
        # Queues for thread communication
        self.image_queue = queue.Queue(maxsize=10)
        self.command_queue = queue.Queue()
        
        # Control flags
        self.running = False
        
    def connect(self):
        """Establish TCP connection to ROS"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ros_host, self.ros_port))
            self.connected = True
            logger.info(f"Connected to ROS at {self.ros_host}:{self.ros_port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ROS: {e}")
            return False
    
    def disconnect(self):
        """Close TCP connection"""
        if self.socket:
            self.socket.close()
            self.connected = False
            logger.info("Disconnected from ROS")
    
    def capture_images(self):
        """Capture images from all regions"""
        images = []
        timestamp = time.time_ns()
        
        for i, region in enumerate(self.regions):
            try:
                screenshot = ImageGrab.grab(bbox=region)
                
                # Compress image to JPEG
                buffer = io.BytesIO()
                screenshot.save(buffer, format='JPEG', quality=85)
                image_data = buffer.getvalue()
                
                images.append({
                    'id': i,
                    'timestamp': timestamp,
                    'data': image_data
                })
            except Exception as e:
                logger.error(f"Failed to capture image {i}: {e}")
        
        return images
    
    def send_image(self, image_id, timestamp, image_data):
        """
        Send image data to ROS
        
        Message format:
        [4 bytes: 'IMG0'-'IMG5']
        [8 bytes: timestamp (nanoseconds)]
        [4 bytes: image size N]
        [N bytes: JPEG image data]
        """
        try:
            # Prepare message type
            msg_type = f'IMG{image_id}'.encode('ascii')
            
            # Pack message
            header = struct.pack('!4sQI', msg_type, timestamp, len(image_data))
            message = header + image_data
            
            # Send message
            self.socket.sendall(message)
            logger.debug(f"Sent image {image_id}, size: {len(image_data)} bytes")
            
        except Exception as e:
            logger.error(f"Failed to send image {image_id}: {e}")
            self.connected = False
    
    def send_status(self, uav_id, position, velocity, attitude):
        """
        Send UAV status to ROS
        
        Message format:
        [4 bytes: 'STAT']
        [4 bytes: UAV ID]
        [12 bytes: position (x, y, z)]
        [12 bytes: velocity (vx, vy, vz)]
        [12 bytes: attitude (roll, pitch, yaw)]
        """
        try:
            msg_type = b'STAT'
            message = struct.pack('!4sI3f3f3f',
                                msg_type,
                                uav_id,
                                *position,
                                *velocity,
                                *attitude)
            self.socket.sendall(message)
            logger.debug(f"Sent status for UAV {uav_id}")
            
        except Exception as e:
            logger.error(f"Failed to send status: {e}")
            self.connected = False
    
    def receive_commands(self):
        """Receive control commands from ROS"""
        while self.running and self.connected:
            try:
                # Receive message type (4 bytes)
                msg_type = self.socket.recv(4)
                if not msg_type:
                    logger.warning("Connection closed by ROS")
                    self.connected = False
                    break
                
                if msg_type == b'MOVE':
                    # Receive MOVE command
                    data = self.socket.recv(28)
                    uav_id, tx, ty, tz, vx, vy, vz = struct.unpack('!I3f3f', data)
                    
                    command = {
                        'type': 'MOVE',
                        'uav_id': uav_id,
                        'target_position': (tx, ty, tz),
                        'target_velocity': (vx, vy, vz)
                    }
                    self.command_queue.put(command)
                    logger.info(f"Received MOVE command for UAV {uav_id}")
                    
                elif msg_type == b'STOP':
                    # Receive STOP command
                    command = {'type': 'STOP'}
                    self.command_queue.put(command)
                    logger.info("Received STOP command")
                    
                elif msg_type == b'PING':
                    # Respond to PING
                    self.socket.sendall(b'PONG')
                    logger.debug("Received PING, sent PONG")
                    
            except Exception as e:
                logger.error(f"Error receiving commands: {e}")
                self.connected = False
                break
    
    def image_capture_thread(self):
        """Thread for capturing images"""
        logger.info("Image capture thread started")
        
        while self.running:
            try:
                images = self.capture_images()
                
                # Put images in queue (drop old frames if queue is full)
                try:
                    self.image_queue.put(images, block=False)
                except queue.Full:
                    logger.warning("Image queue full, dropping frame")
                    try:
                        self.image_queue.get_nowait()
                        self.image_queue.put(images, block=False)
                    except:
                        pass
                
                time.sleep(0.5)  # Capture at 2 Hz
                
            except Exception as e:
                logger.error(f"Error in image capture thread: {e}")
    
    def image_send_thread(self):
        """Thread for sending images"""
        logger.info("Image send thread started")
        
        while self.running:
            try:
                # Get images from queue
                images = self.image_queue.get(timeout=1.0)
                
                if self.connected:
                    # Send all images
                    for img in images:
                        self.send_image(img['id'], img['timestamp'], img['data'])
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in image send thread: {e}")
    
    def command_receive_thread(self):
        """Thread for receiving commands"""
        logger.info("Command receive thread started")
        self.receive_commands()
    
    def run(self):
        """Main run loop"""
        logger.info("Starting Arma 3 Bridge...")
        
        # Connect to ROS
        if not self.connect():
            logger.error("Failed to connect to ROS. Exiting.")
            return
        
        # Start threads
        self.running = True
        
        capture_thread = threading.Thread(target=self.image_capture_thread)
        send_thread = threading.Thread(target=self.image_send_thread)
        receive_thread = threading.Thread(target=self.command_receive_thread)
        
        capture_thread.start()
        send_thread.start()
        receive_thread.start()
        
        try:
            # Main loop
            while self.running:
                # Check connection status
                if not self.connected:
                    logger.warning("Connection lost. Attempting to reconnect...")
                    time.sleep(5)
                    self.connect()
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.running = False
            capture_thread.join()
            send_thread.join()
            receive_thread.join()
            self.disconnect()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Arma 3 Bridge Program')
    parser.add_argument('--host', type=str, default='192.168.1.100',
                      help='IP address of Linux VM running ROS')
    parser.add_argument('--port', type=int, default=5555,
                      help='Port number for TCP communication')
    
    args = parser.parse_args()
    
    bridge = Arma3Bridge(ros_host=args.host, ros_port=args.port)
    bridge.run()
