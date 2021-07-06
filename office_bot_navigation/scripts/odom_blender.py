#!/usr/bin/env python

# Problem: I want an odom message which uses the pose from scan_matching (which is good)
# and the velocity from the create (which is also good)
# Solution: this node subs to the create odom and scan matcher pose, and fuses the two in a 
# single output odom message

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Blender:
    def __init__(self):

        self.odom_pub = rospy.Publisher("odom_fused", Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odom)
        self.pose_sub = rospy.Subscriber("pose_stamped", PoseStamped, self.update_pose_and_publish)
        self.last_odom = None
    
    def update_odom(self, odom_msg:Odometry):
        self.last_odom = odom_msg
    
    def update_pose_and_publish(self, pose_stamped_msg:PoseStamped):
        if self.last_odom is None:
            rospy.loginfo("waiting for odom input")
            return

        fused_odom = Odometry()
        fused_odom.header = pose_stamped_msg.header
        fused_odom.child_frame_id = self.last_odom.child_frame_id
        fused_odom.twist = self.last_odom.twist
        fused_odom.pose.pose = pose_stamped_msg.pose

        self.odom_pub.publish(fused_odom)

        

if __name__ == '__main__':
    rospy.init_node('odom_blender')

    blender = Blender()
    rospy.spin()
