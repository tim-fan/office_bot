#!/usr/bin/env python

#node to subscribe to a tf topic, and pass on odom -> base link transforms to an output 
#topic 

# Python expression to filter messages that are printed.
# Expression can use Python builtins as well as m (the
# message) and topic (the topic name).

import rospy
import argparse
from rostopic import get_topic_class


def apply_filter(msg, out_pub):
    """
    Extract only odom tf from given message, and republish
    """
    
    isOdom = lambda tf : tf.child_frame_id == 'base_link' and tf.header.frame_id == 'odom'    
    msg.transforms = filter(isOdom, msg.transforms)    
    if msg.transforms:
        out_pub.publish(msg)



if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Publish messages from one topic to another if they pass a given filter function')
    parser.add_argument('in_topic', help="the topic to listen to")
    parser.add_argument('out_topic', help="the topic to publish filtered messages to")
    args = parser.parse_args()    
        
    try:
        rospy.init_node('tf_odom_filter', anonymous=True)            
        msg_class, in_topic, _ = get_topic_class(args.in_topic, blocking=True)            
        out_pub = rospy.Publisher(args.out_topic, msg_class, queue_size=10)            
        callback = lambda msg : apply_filter(msg, out_pub)            
        rospy.Subscriber(in_topic, msg_class, callback)
        rospy.spin()
            
    except rospy.ROSInterruptException:
        pass
