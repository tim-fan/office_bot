#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import tf2_kdl
import geometry_msgs.msg


#node subscribes to robot position as published by gazebo and republishes to tf (map -> odom)
#this avoids the need to run localisation during simulation

def odomToTfStamped(odomMsg):
    tfStamped = geometry_msgs.msg.TransformStamped()
    tfStamped.header.stamp = odomMsg.header.stamp
    tfStamped.header.frame_id = odomMsg.header.frame_id
    tfStamped.child_frame_id = odomMsg.child_frame_id
    tfStamped.transform.translation.x = odomMsg.pose.pose.position.x
    tfStamped.transform.translation.y = odomMsg.pose.pose.position.y
    tfStamped.transform.translation.z = odomMsg.pose.pose.position.z
    tfStamped.transform.rotation.x = odomMsg.pose.pose.orientation.x
    tfStamped.transform.rotation.y = odomMsg.pose.pose.orientation.y
    tfStamped.transform.rotation.z = odomMsg.pose.pose.orientation.z
    tfStamped.transform.rotation.w = odomMsg.pose.pose.orientation.w
    return tfStamped

def kdlToTfStamped(kdlFrame, stamp, frame_id, child_frame_id):
    tfStamped = geometry_msgs.msg.TransformStamped()
    tfStamped.header.stamp = stamp
    tfStamped.header.frame_id = frame_id
    tfStamped.child_frame_id = child_frame_id
    tfStamped.transform.translation.x = kdlFrame.p[0]
    tfStamped.transform.translation.y = kdlFrame.p[0]
    tfStamped.transform.translation.z = kdlFrame.p[0]

    q = kdlFrame.M.GetQuaternion()
    tfStamped.transform.rotation.x = q[0]
    tfStamped.transform.rotation.y = q[1]
    tfStamped.transform.rotation.z = q[2]
    tfStamped.transform.rotation.w = q[3]
    return tfStamped

class TfPublisher:

    def __init__(self):
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def publishLocalisation(self, locationMsg):

        pubTime = locationMsg.header.stamp

        mapToBaseTf = odomToTfStamped(locationMsg)
        try:
            odomToBaseTf = self.tfBuffer.lookup_transform('odom', 'base_footprint', pubTime, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return

        mapToBaseKdl = tf2_kdl.transform_to_kdl(mapToBaseTf)
        odomToBaseKdl = tf2_kdl.transform_to_kdl(odomToBaseTf)

        mapToOdomKdl = mapToBaseKdl * odomToBaseKdl.Inverse()
        mapToOdomTf = kdlToTfStamped(mapToOdomKdl, pubTime, 'map', 'odom')

        self.tfBroadcaster.sendTransform(mapToOdomTf)

    
def publisher():

    rospy.init_node('location_publisher', anonymous=True)

    tfPub = TfPublisher()
    rospy.Subscriber("simple_controller/absolute_position", Odometry, tfPub.publishLocalisation)

    rospy.spin()

if __name__ == '__main__':
    publisher()
