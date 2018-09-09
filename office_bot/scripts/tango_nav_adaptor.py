#!/usr/bin/env python

#node for using tango_ros_streamer tf messages in ros navigation stack
# does four things:
# 1. convert link names:
#       area_description -> map
#       start_of_service -> odom
#       device -> tango
# 2. update tf timestamps
#       - don't really understand the issue, but timestamps of tf messages
#         from tango_ros_streamer lag by 10s of seconds or more.
#         As a basic hack/workaround, this node just re-stamps them with
#         current ROS time
# 3. set tf poses to fixed height (assume 2d world)
# 4. generate odom messages from tf messages
#     

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos




class TfConverter:
    
    frameIdConversions = { #from_id:to_id
        'area_description' : 'map',
        'start_of_service' : 'odom',
        'device' : 'tango'
    }

    def __init__(self):
        self.br = tf.TransformBroadcaster()
	self.deviceHeight = 0.2 #todo: make this a param

    def convert(self, tfMsg):
        """
        Update frame ids, height and timestamp, and republish
        """

	if tfMsg.child_frame_id == "start_of_service": #adf -> sos
        #todo: how to handle non-yaw rotations in this transform? Set to zero?
		tfMsg.transform.translation.z = 0
	elif tfMsg.child_frame_id == "device": #sos -> device
		tfMsg.transform.translation.z = self.deviceHeight

        tfMsg.header.frame_id = TfConverter.frameIdConversions[tfMsg.header.frame_id]
        tfMsg.child_frame_id = TfConverter.frameIdConversions[tfMsg.child_frame_id]
        tfMsg.header.stamp = rospy.Time.now()

        try:
            self.br.sendTransformMessage(tfMsg)
        except rospy.ROSException:
            #catching error publishing to closed topic
            #which occurs when log playback loops
            self.br = tf.TransformBroadcaster()
    
class OdomPublisher:
    
    def __init__(self):
        self.lastPos = None
        self.odomPublisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()     
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    
    def publish(self):
        if self.lastPos and self.lastPos.header.stamp > rospy.Time.now():
            rospy.logwarn('detected backward jump in time, resetting tf buffer')
            self.tfBuffer = tf2_ros.Buffer()     
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        try:
            currentPos = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0))
            
            #print(currentPos)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            currentPos = self.lastPos
            rospy.logwarn(e)
        
        if (self.lastPos is not None):
            dt = (currentPos.header.stamp - self.lastPos.header.stamp).to_sec()
            #print(dt)

            
            if dt <= 0:
                #wait for an updated tf
                pass
                
            else:
                
                
                #build and publish an odometry message based on observed motions
                deltaXGlobal = currentPos.transform.translation.x - self.lastPos.transform.translation.x
                deltaYGlobal = currentPos.transform.translation.y - self.lastPos.transform.translation.y
                prevYaw = OdomPublisher.yawFromTransform(self.lastPos)
                deltaX = deltaXGlobal *  cos(prevYaw)  + deltaYGlobal * sin(prevYaw)
                deltaY = deltaXGlobal * -sin(prevYaw) + deltaYGlobal * cos(prevYaw)
                deltaYaw = OdomPublisher.yawFromTransform(currentPos) - prevYaw
                
                wrapAngle = lambda x : ((x + pi) % (2*pi)) - pi #wrap to +/- pi
                deltaYaw = wrapAngle(deltaYaw)
                
                odomMsg = Odometry()
                odomMsg.header = currentPos.header
                odomMsg.pose.pose = OdomPublisher.transformToPose(currentPos.transform)
                odomMsg.child_frame_id = currentPos.child_frame_id
                odomMsg.twist.twist.linear.x = deltaX / dt
                odomMsg.twist.twist.linear.y = deltaY / dt
                odomMsg.twist.twist.angular.z = deltaYaw / dt
                
                self.odomPublisher.publish(odomMsg)

        self.lastPos = currentPos
    
    @staticmethod
    def yawFromTransform(t): #type (geometry_msgs.msg._TransformStamped.TransformStamped) -> Float
        """
        Extract yaw value associated with given transformStamped msg
        """
        quaternion_array = lambda q : [q.x, q.y, q.z, q.w]
        orientation = quaternion_array(t.transform.rotation)
        roll, pitch, yaw = euler_from_quaternion(orientation)
        return yaw
        
    @staticmethod
    def transformToPose(t): # (geometry_msgs.msg._Transform.Transform) -> geometry_msgs.msg._Pose.Pose
        """
        These two types seem to represent the same thing, but with different field names.
        """
        p = Pose()
        p.orientation.x = t.rotation.x
        p.orientation.y = t.rotation.y
        p.orientation.z = t.rotation.z
        p.orientation.w = t.rotation.w
        p.position.x = t.translation.x
        p.position.y = t.translation.y
        p.position.z = t.translation.z
        return p

def nav_adaptor():
    
    rospy.init_node('tango_nav_adaptor')
    tfConverter = TfConverter()
    rospy.Subscriber("tango/transform/area_description_T_start_of_service", TransformStamped, tfConverter.convert)
    rospy.Subscriber("tango/transform/start_of_service_T_device", TransformStamped, tfConverter.convert)
    
    odomPublishRate = rospy.Rate(10) # 10hz
    odomPublisher = OdomPublisher()

    rospy.logwarn('This node is deprecated - see tango_nav_adaptor for more efficient cpp implementation')

    while not rospy.is_shutdown():
        odomPublisher.publish()
        try:
            odomPublishRate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)
   
    rospy.spin() 

if __name__ == '__main__':
    nav_adaptor()

