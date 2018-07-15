#!/usr/bin/env python

#node for generating odom messages based on the odom -> base_link transform
# numerically differentiates the transform in order to generate the required twist method.

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos
from angles import normalize_angle

    
class OdomPublisher:
    
    def __init__(self, odomFrame = "odom", baseFrame="base_link"):
        self.lastPos = None
        self.odomPublisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()     
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.odomFrame = odomFrame
        self.baseFrame = baseFrame

    
    def publish(self):
        
        #handle backward jumps in time (playing logs on loop)
        #ToDo: confirm this works
        if self.lastPos is not None and self.lastPos.header.stamp > rospy.Time.now():
            rospy.logwarn('detected backward jump in time, resetting tf buffer')
            self.tfBuffer = tf2_ros.Buffer()     
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
            elf.lastPos = None
            return
        
        #get current pos, handling tf errors
        try:
            currentPos = self.tfBuffer.lookup_transform(self.odomFrame, self.baseFrame, rospy.Time(0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(e)
            return
        
        #cannot publish until two positions have been received
        if self.lastPos is None:
            self.lastPos = currentPos
            return
            
        #cannot determine vel if last pos and current have same time stamp
        if self.lastPos.header.stamp == currentPos.header.stamp:
            return
            
        #determine current velocity
        currentVel = OdomPublisher.getVel(currentPos, self.lastPos)
        
        #now build odom msg and publish
        odomMsg = Odometry()
        odomMsg.header = currentPos.header
        odomMsg.pose.pose = OdomPublisher.transformToPose(currentPos.transform)
        odomMsg.child_frame_id = currentPos.child_frame_id
        odomMsg.twist.twist = currentVel        
        self.odomPublisher.publish(odomMsg)

        #save current pos for next time
        self.lastPos = currentPos
    
    @staticmethod
    def getVel(currentPos, lastPos):
        # type: (TransformStamped, TransformStamped) -> Twist
        """
        Return twist representing velocity implied by pose/time difference
        """
        
        dt = (currentPos.header.stamp - lastPos.header.stamp).to_sec()
        
        #find pose difference (global frame)
        deltaXGlobal = currentPos.transform.translation.x - lastPos.transform.translation.x
        deltaYGlobal = currentPos.transform.translation.y - lastPos.transform.translation.y
        prevYaw = OdomPublisher.yawFromTransform(lastPos)
        deltaYaw = OdomPublisher.yawFromTransform(currentPos) - prevYaw
        deltaYaw = normalize_angle(deltaYaw)
        
        #convert x,y differences in global frame to x,y differences in the frame of lastPos        
        deltaX = deltaXGlobal *  cos(prevYaw)  + deltaYGlobal * sin(prevYaw)
        deltaY = deltaXGlobal * -sin(prevYaw) + deltaYGlobal * cos(prevYaw)
        
        #generate output twist message        
        twist = Twist()
        twist.linear.x = deltaX / dt
        twist.linear.y = deltaY / dt
        twist.angular.z = deltaYaw / dt
        return twist
        

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


def publish_odom():    
    rospy.init_node('odom_from_tf')
    publishRateParam = rospy.get_param("~publish_rate", 10) #how frequently to publish odom messages
    odomFrame = rospy.get_param('~odom_frame', 'odom')
    baseFrame = rospy.get_param('~base_frame', 'base_link')
    publishRate = rospy.Rate(publishRateParam)
    odomPublisher = OdomPublisher(odomFrame=odomFrame, baseFrame=baseFrame)
    while not rospy.is_shutdown():
        odomPublisher.publish()
        try:
            publishRate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    publish_odom()

