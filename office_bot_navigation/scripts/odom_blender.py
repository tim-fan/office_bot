#!/usr/bin/env python

# Problem: I want an odom message which uses the pose from cartographer (which is good)
# and the velocity from the create (which is also good)
# Solution: this node subs to the create odom and cartographer pose (from tf), and fuses the two in a 
# single output odom message

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros


class Blender:
    def __init__(self):

        self.odom_pub = rospy.Publisher("odom_fused", Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odom)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def update_odom(self, odom_msg:Odometry):
        """
        Fuse recieved odom with odom pose from tf, then republish
        """
        try:
            current_odom_tf = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return

        # print("-------------odom_msg:")
        # print(f"{odom_msg.pose.pose}")
        # print("-------------odom_tf:")
        # print(f"{current_odom_tf.transform}" )
        # print("")

        odom_msg.pose.pose.position = current_odom_tf.transform.translation
        odom_msg.pose.pose.orientation = current_odom_tf.transform.rotation

        # print("-------------odom_msg fused:")
        # print(f"{odom_msg.pose.pose}")

        self.odom_pub.publish(odom_msg)

    
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
