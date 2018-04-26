#!/usr/bin/env python

#node to subscribe to a certain topic, and pass on messages to an output 
#topic if they meet a given criteria

# Python expression to filter messages that are printed.
# Expression can use Python builtins as well as m (the
# message) and topic (the topic name).

import rospy
import argparse
from rostopic import get_topic_class


def apply_filter(msg, filter_fn, out_pub):
    """
    Check if the given message passes the filter function. If so, publish it
    """
    if filter_fn(msg):
        out_pub.publish(msg)

def expr_eval(expr):
    def eval_fn(m):
        return eval(expr)
    return eval_fn

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Publish messages from one topic to another if they pass a given filter function')
    parser.add_argument('in_topic', help="the topic to listen to")
    parser.add_argument('out_topic', help="the topic to publish filtered messages to")
    parser.add_argument('filter_fn', help="""Python expression to filter messages that are printed.
                        Expression can use Python builtins as well as m (the
                        message) and topic (the topic name).""")
    args = parser.parse_args()
    
        
    try:
            #scanFilter = DistortedScanFilter()
            rospy.init_node('topic_filter', anonymous=True)
            
            msg_class, in_topic, _ = get_topic_class(args.in_topic, blocking=True)
            
            filter_fn = expr_eval(args.filter_fn)
            out_pub = rospy.Publisher(args.out_topic, msg_class, queue_size=10)
            
            callback = lambda msg : apply_filter(msg, filter_fn, out_pub)
            
            rospy.Subscriber(in_topic, msg_class, callback)

            rospy.spin()
    except rospy.ROSInterruptException:
        pass
