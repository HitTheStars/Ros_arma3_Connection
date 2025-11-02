#!/usr/bin/env python3
"""
EGO-Planner Interface Node
This node interfaces between Arma 3 and EGO-Planner.
It receives point clouds and UAV states, and publishes goal positions for EGO-Planner.
"""

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import tf

class EGOPlannerInterface:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ego_planner_interface', anonymous=False)
        
        # Parameters
        self.num_uavs = rospy.get_param('~num_uavs', 3)
        
        # Current poses of UAVs
        self.current_poses = [None] * self.num_uavs
        
        # Goal positions
        self.goal_positions = [None] * self.num_uavs
        
        # Subscribers
        for i in range(self.num_uavs):
            rospy.Subscriber(f'/arma3/uav_{i}/pose', PoseStamped,
                           lambda msg, idx=i: self.pose_callback(msg, idx))
        
        rospy.Subscriber('/arma3/pointcloud', PointCloud2, self.pointcloud_callback)
        
        # Publishers for EGO-Planner
        self.odom_pubs = []
        self.goal_pubs = []
        
        for i in range(self.num_uavs):
            # Publish odometry for EGO-Planner
            odom_pub = rospy.Publisher(f'/drone_{i}_visual_slam/odom', Odometry, queue_size=10)
            self.odom_pubs.append(odom_pub)
            
            # Publish goal for EGO-Planner
            goal_pub = rospy.Publisher(f'/drone_{i}_planning/goal', PoseStamped, queue_size=10)
            self.goal_pubs.append(goal_pub)
        
        # Publisher for visualization
        self.marker_pub = rospy.Publisher('/ego_planner/goals', MarkerArray, queue_size=10)
        
        # Timer for publishing odometry
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        
        rospy.loginfo("EGO-Planner Interface initialized")
    
    def pose_callback(self, msg, idx):
        """Store current pose of UAV"""
        self.current_poses[idx] = msg
    
    def pointcloud_callback(self, msg):
        """Receive point cloud from stereo vision"""
        # Point cloud is automatically available to EGO-Planner via /arma3/pointcloud topic
        rospy.logdebug("Received point cloud")
    
    def publish_odometry(self, event):
        """Publish odometry messages for EGO-Planner"""
        for i in range(self.num_uavs):
            if self.current_poses[i] is None:
                continue
            
            # Convert PoseStamped to Odometry
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "world"
            odom.child_frame_id = f"drone_{i}"
            
            odom.pose.pose = self.current_poses[i].pose
            
            # Publish odometry
            self.odom_pubs[i].publish(odom)
    
    def set_goal(self, uav_id, x, y, z):
        """Set goal position for a UAV"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        
        self.goal_positions[uav_id] = goal
        self.goal_pubs[uav_id].publish(goal)
        
        rospy.loginfo(f"Set goal for UAV {uav_id}: ({x}, {y}, {z})")
        
        # Visualize goal
        self.visualize_goals()
    
    def visualize_goals(self):
        """Visualize goal positions as markers"""
        marker_array = MarkerArray()
        
        for i in range(self.num_uavs):
            if self.goal_positions[i] is None:
                continue
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "goals"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = self.goal_positions[i].pose
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("EGO-Planner Interface running...")
        
        # Example: Set some default goals
        rospy.sleep(2.0)  # Wait for initialization
        
        # Set goals for each UAV (these can be changed via service calls or topics)
        self.set_goal(0, 100, 0, 100)
        self.set_goal(1, 100, 50, 100)
        self.set_goal(2, 100, -50, 100)
        
        rospy.spin()

if __name__ == '__main__':
    try:
        interface = EGOPlannerInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass
