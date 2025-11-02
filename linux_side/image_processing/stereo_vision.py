#!/usr/bin/env python3
"""
Stereo Vision Processing Node
This node receives multi-view images from Arma 3 and generates point clouds.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import message_filters

class StereoVisionProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('stereo_vision_processor', anonymous=False)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
        # Camera parameters (these should be calibrated for your setup)
        self.focal_length = 500.0  # pixels
        self.baseline = 0.1  # meters (distance between cameras)
        
        # Subscribers for images
        self.image_subs = []
        self.latest_images = [None] * 6
        
        for i in range(6):
            sub = rospy.Subscriber(f'/arma3/camera_{i}/image_raw', Image, 
                                  lambda msg, idx=i: self.image_callback(msg, idx))
            self.image_subs.append(sub)
        
        # Publisher for point cloud
        self.pointcloud_pub = rospy.Publisher('/arma3/pointcloud', PointCloud2, queue_size=10)
        
        # Processing timer
        self.timer = rospy.Timer(rospy.Duration(0.5), self.process_images)
        
        rospy.loginfo("Stereo Vision Processor initialized")
    
    def image_callback(self, msg, idx):
        """Store latest image for each camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_images[idx] = cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image {idx}: {e}")
    
    def compute_disparity(self, left_img, right_img):
        """Compute disparity map from stereo pair"""
        # Convert to grayscale
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        # Compute disparity
        disparity = self.stereo.compute(left_gray, right_gray)
        
        # Normalize for visualization
        disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        return disparity, disparity_normalized
    
    def disparity_to_pointcloud(self, disparity, left_img):
        """Convert disparity map to 3D point cloud"""
        h, w = disparity.shape
        
        # Create meshgrid of pixel coordinates
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # Compute depth from disparity
        # depth = (focal_length * baseline) / disparity
        # Avoid division by zero
        disparity_safe = np.where(disparity > 0, disparity, 0.1)
        depth = (self.focal_length * self.baseline) / disparity_safe
        
        # Compute 3D coordinates
        # Assuming principal point at image center
        cx = w / 2.0
        cy = h / 2.0
        
        x = (u - cx) * depth / self.focal_length
        y = (v - cy) * depth / self.focal_length
        z = depth
        
        # Get color from left image
        colors = left_img.reshape(-1, 3)
        
        # Stack coordinates
        points = np.stack([x.flatten(), y.flatten(), z.flatten()], axis=1)
        
        # Filter out invalid points (too far or too close)
        valid_mask = (depth.flatten() > 0.5) & (depth.flatten() < 100.0)
        points = points[valid_mask]
        colors = colors[valid_mask]
        
        return points, colors
    
    def create_pointcloud_msg(self, points, colors):
        """Create ROS PointCloud2 message"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        
        # Define point cloud fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]
        
        # Pack RGB into single uint32
        rgb_packed = np.zeros(len(points), dtype=np.uint32)
        rgb_packed = (colors[:, 2].astype(np.uint32) << 16 | 
                     colors[:, 1].astype(np.uint32) << 8 | 
                     colors[:, 0].astype(np.uint32))
        
        # Combine points and colors
        cloud_data = np.zeros(len(points), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        cloud_data['rgb'] = rgb_packed
        
        # Create PointCloud2 message
        pc_msg = pc2.create_cloud(header, fields, cloud_data)
        
        return pc_msg
    
    def process_images(self, event):
        """Process images to generate point cloud"""
        # Check if we have all images
        if any(img is None for img in self.latest_images):
            rospy.logdebug("Waiting for all images...")
            return
        
        try:
            # Use first two images as stereo pair (can be improved)
            left_img = self.latest_images[0]
            right_img = self.latest_images[1]
            
            # Compute disparity
            disparity, _ = self.compute_disparity(left_img, right_img)
            
            # Convert to point cloud
            points, colors = self.disparity_to_pointcloud(disparity, left_img)
            
            # Create and publish PointCloud2 message
            pc_msg = self.create_pointcloud_msg(points, colors)
            self.pointcloud_pub.publish(pc_msg)
            
            rospy.loginfo(f"Published point cloud with {len(points)} points")
            
        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Stereo Vision Processor running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = StereoVisionProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
