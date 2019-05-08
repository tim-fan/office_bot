#!/usr/bin/env python

#node to command the robot to follow people around
#subscribes to person detections for input, and publishes to cmd_vel to control the robot

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from spencer_tracking_msgs.msg import TrackedPersons
from geometry_msgs.msg import Twist
from math import atan2

def publishMotionCmd(trackedPersonsMsg, tfBuffer, publisher):
    """
    Given the current state of person tracking, determine the appropriate control output
    """

    followDistance = 0.8
    cmdVel = Twist() #the output command

    peopleDetected = len(trackedPersonsMsg.tracks) > 0

    if peopleDetected:
        #choose the track with the lowest id to follow (ie the person who was seen first)
        minTrackingId = min(track.track_id for track in trackedPersonsMsg.tracks)
        trackToFollow = filter(lambda track : track.track_id == minTrackingId, trackedPersonsMsg.tracks)[0]

        #get position of person in robot frame
        try:
            personPoseRobotCentric = tfBuffer.transform(
                object_stamped=PoseStamped(
                    header=trackedPersonsMsg.header,
                    pose=trackToFollow.pose.pose
                ),
                target_frame='base_link'
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #failed to transform pose for some reason - just return a zero vel command
            rospy.logwarn(e)
            return cmdVel

        #find the current angle to the person
        angleToPerson = atan2(personPoseRobotCentric.pose.position.y, personPoseRobotCentric.pose.position.x)
        
        #use proportional control to aim at person
        pGainAngVel = 1.0
        cmdVel.angular.z = angleToPerson * pGainAngVel

        distanceToPerson = personPoseRobotCentric.pose.position.x
        distanceError = distanceToPerson - followDistance
        pGainVel = 1.0
        maxVel = 1.0
        correctionVel = distanceError * pGainVel
        #apply max/min
        correctionVel = min(correctionVel, maxVel)
        correctionVel = max(correctionVel, -maxVel)
        
        #cmdVel.linear.x = correctionVel
        cmdVel.linear.x = 0

        publisher.publish(cmdVel)



if __name__ == '__main__':
    
    try:
        rospy.init_node('person_follower')
        
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        cmdPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        controlCallback = lambda trackingMsg : publishMotionCmd(trackingMsg, tfBuffer, cmdPub)
        trackingSub = rospy.Subscriber('tracked_persons', TrackedPersons, controlCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
