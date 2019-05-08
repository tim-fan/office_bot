#!/usr/bin/env python

#node to command the robot to follow people around
#subscribes to person detections for input, and publishes to cmd_vel to control the robot

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist
from math import atan2, pi

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

def generateMotionCmd(fiducialMsg, tfBuffer):
    """
    Given the current state of fiducial tracking, determine the appropriate control output
    """

    followDistance = 0.8
    cmdVel = Twist() #the output command

    fiducialsDetected = len(fiducialMsg.transforms) > 0 

    if fiducialsDetected:
        # follow the largest visible fiducial
        largestFiducialSize = min(f.fiducial_area for f in fiducialMsg.transforms)
        fiducialToFollow = filter(lambda f : f.fiducial_area == largestFiducialSize, fiducialMsg.transforms)[0]

        #get position of fiducial in robot frame
        fiducialFrame = 'fiducial_{}'.format(fiducialToFollow.fiducial_id)
        fiducialTfStamped = TransformStamped(
            header = fiducialMsg.header,
            child_frame_id = fiducialFrame,
            transform = fiducialToFollow.transform
        )
        
        try:
            # not sure why, but I can't seem to transform another transform message - need to convert to pose first
            fiducialPoseRobotCentric = tfBuffer.transform(
                object_stamped=PoseStamped(
                    header = fiducialTfStamped.header,
                    pose = transformToPose(fiducialTfStamped.transform)
                ),
                target_frame='base_link'
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #failed to transform pose for some reason - just return a zero vel command
            rospy.logwarn(e)
            return cmdVel

        #find the current angle to the fiducial
        angleToFiducial = atan2(fiducialPoseRobotCentric.pose.position.y, fiducialPoseRobotCentric.pose.position.x)
        targetAngle = pi #aim to have fiducial behind the robot
        wrapAngle = lambda x : ((x + pi) % (2*pi)) - pi
        angleError = wrapAngle(angleToFiducial - targetAngle)
        
        #use proportional control to aim at person
        pGainAngVel = 1.0
        cmdVel.angular.z = angleError * pGainAngVel

        distanceToFiducial = fiducialPoseRobotCentric.pose.position.x
        distanceError = distanceToFiducial - followDistance
        pGainVel = 1.0
        maxVel = 1.0
        correctionVel = distanceError * pGainVel
        #apply max/min
        correctionVel = min(correctionVel, maxVel)
        correctionVel = max(correctionVel, -maxVel)
        
        cmdVel.linear.x = correctionVel

        
    return cmdVel


if __name__ == '__main__':
    
    try:
        rospy.init_node('fiducial_follower')
        
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        cmdPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        controlCallback = lambda trackingMsg : cmdPub.publish(generateMotionCmd(trackingMsg, tfBuffer))
        trackingSub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, controlCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
