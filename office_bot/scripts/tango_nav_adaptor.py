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
# 

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos
from tf_conversions import toMsg
from tf2_kdl import transform_to_kdl


#~ class TfConverter:
    
    #~ frameIdConversions = { #from_id:to_id
        #~ 'area_description' : 'map',
        #~ 'start_of_service' : 'odom',
        #~ 'device' : 'tango'
    #~ }

    #~ def __init__(self):
        #~ self.br = tf.TransformBroadcaster()
    #~ self.deviceHeight = 0.2 #todo: make this a param

    #~ def convert(self, tfMsg):
        #~ """
        #~ Update frame ids, height and timestamp, and republish
        #~ """
        #~ tfMsg.header.stamp = rospy.Time.now()
        
        #~ isOdomTf = tfMsg.child_frame_id == "device"
        #~ convert = convertOdom if isOdomTf else convertLocalisation
        #~ tfToSend = convert(tfMsg)
        #~ try:
            #~ self.br.sendTransformMessage(tfToSend)
        #~ except rospy.ROSException:
            #~ #catching error publishing to closed topic
            #~ #which occurs when log playback loops
            #~ self.br = tf.TransformBroadcaster()
        
        #~ if tfMsg.child_frame_id == "start_of_service":
            #~ tfMsg.transform.translation.z = 0
        #~ elif tfMsg.child_frame_id == "device":
            #~ tfMsg.transform.translation.z = self.deviceHeight

            #~ tfMsg.header.frame_id = TfConverter.frameIdConversions[tfMsg.header.frame_id]
            #~ tfMsg.child_frame_id = TfConverter.frameIdConversions[tfMsg.child_frame_id]
            

    #~ def convertLocalisation(self, tfMsg):
        #~ """
        #~ Convert a start_
        #~ """

    #~ def convertLocalisation(self, tfMsg):
        #~ tfMsg.header.frame_id = 'map' #convert from 'area_description'
        #~ tfMsg.child_frame_id = 'odom' #convert from 'start of service'
        #~ tfMsg.transform.translation.z = 0
        #~ return tfMsg
    

    


#~ def nav_adaptor():
    
    #~ rospy.init_node('tango_nav_adaptor')
    #~ staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
#~ static_transformStamped = geometry_msgs.msg.TransformStamped()
    
    #~ tfConverter = TfConverter()
    #~ rospy.Subscriber("tango/transform/area_description_T_start_of_service", TransformStamped, tfConverter.convert)
    #~ rospy.Subscriber("tango/transform/start_of_service_T_device", TransformStamped, tfConverter.convert)
    
    #~ odomPublishRate = rospy.Rate(10) # 10hz
    #~ odomPublisher = OdomPublisher()
    #~ while not rospy.is_shutdown():
        #~ odomPublisher.publish()
        #~ try:
            #~ odomPublishRate.sleep()
        #~ except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            #~ rospy.logwarn(e)

class TangoNavAdaptor:
    def __init__(self):
        
        ######## UNFINISHED SCRIPT!!! ###############
        # part way through rewrite
        # attempting to publish map->base like how AMCL handles map->odom
        # refer https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl_node.cpp#L1398
        
        #set up tf broadcasters/listeners
        self.staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer() 
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        #join the tango and nav tf trees
        self.publishMapToAreaDescription()

    def publishMapToAreaDescription(self):
        mapToStartOfServiceTf = TransformStamped()
        mapToStartOfServiceTf.header.stamp = rospy.Time.now()
        mapToStartOfServiceTf.header.frame_id = 'map'
        mapToStartOfServiceTf.child_frame_id = 'area_description'
        mapToStartOfServiceTf.transform.rotation.w = 1
        self.staticBroadcaster.sendTransform(mapToStartOfServiceTf)
        
    def lookupTf(self, parentFrame, childFrame):
        try:
            return self.tfBuffer.lookup_transform(parentFrame, childFrame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(e)
            return None

    def publishMapToOdom(self):
        """
        look up area_description -> start_of_service (as published by tango)
        and publish as map -> odom
        """
        mapToOdomTf = self.lookupTf('area_description', 'start_of_service')
        if mapToOdomTf is None:
            rospy.logwarn('Failed to lookup area_description -> start_of_service tf')
            return
        mapToOdomTf.header.frame_id = 'map'
        mapToOdomTf.child_frame_id = 'odom'
        self.tfBroadcaster.sendTransform(mapToOdomTf)
        print(mapToOdomTf)
    
    def publishOdomToBaseLink(self):
        """
        Publish tf between odom and base_link.
        Looks up the odom -> device (tango) tf, and subtracts the base_link -> tango
        tf.
        Requires base_link -> tango to be published (probably from urdf)
        """
        odomToTangoTf = self.lookupTf('start_of_service', 'device')
        baseToTangoTf = self.lookupTf('base_link', 'tango')
        print(type(baseToTangoTf))
        
        pass
        

def nav_adaptor():
    """
    Publish transforms required by nav stack, based on those provided by tango
    """
    
    rospy.init_node('tango_nav_adaptor')
    
    adaptor = TangoNavAdaptor()
    
    #now publish map -> odom and odom -> base_link tfs at given rate
    publishRate = rospy.Rate(50) #Hz

    while not rospy.is_shutdown():
        adaptor.publishMapToOdom()
        adaptor.publishOdomToBaseLink()
        try:
            publishRate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            rospy.logwarn(e)
    

if __name__ == '__main__':
    nav_adaptor()

