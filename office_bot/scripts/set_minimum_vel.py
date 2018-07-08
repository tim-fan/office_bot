#!/usr/bin/env python
# node to enforce a minimum absolute velocity on commands from a twist topic
#
# Used because the nav. stack was sending velocities to robot which were 
# too small to cause movement.
# This should probably be fixed at a lower level (i.e. in the motor controller)
# but it is convenient/easy to just alter the commanded velocities.
#
# The limit is on absolute speed, and zero-speed is left untouched. 
# For example, some examples of input->output pairs if a limit of 0.3 is used:
#  0.4 ->  0.4
#  0.2 ->  0.3
#  0.0 ->  0.0
# -0.2 -> -0.3
# -0.4 -> -0.4
#
# publishes to topic cmd_vel_boosted (because small velocity values are 'boosted' up to the
# given limit)

import rospy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from office_bot.cfg import CmdVelLimitsConfig

import functools

def cfgCallback(config, level):
    rospy.loginfo("""Reconfigure Request: {vel_x}, {vel_y}, {vel_z}\ 
          {ang_vel_x}, {ang_vel_y}, {ang_vel_z}""".format(**config))
    limits = config
    return config

def sign(x):
    if x > 0:
         return 1
    elif x < 0:
        return -1
    else:
        return 0

def applyLimit(x, limit):
    if limit == 0:
        return x
    elif x == 0:
        return 0
    else:
        absVal = max(limit, abs(x))
        return absVal * sign(x)

class MsgProcessor:
    """
    Processes messages based on velocity limits
    limits are configurable through dynamic reconfigure
    """
    def __init__(self):
        self.config = CmdVelLimitsConfig.defaults
        cfgSrv = Server(CmdVelLimitsConfig, self.updateCfg)
        
        
    def updateCfg(self, config, level):
        print('test')
        rospy.loginfo("""Reconfigure Request: {vel_x}, {vel_y}, {vel_z}\ 
          {ang_vel_x}, {ang_vel_y}, {ang_vel_z}""".format(**config))
        self.config = config
        return config
    
    def processMsg(self, twistMsg):
        twistMsg.linear.x = applyLimit(twistMsg.linear.x, self.config.vel_x)
        twistMsg.linear.y = applyLimit(twistMsg.linear.y, self.config.vel_y)
        twistMsg.linear.z = applyLimit(twistMsg.linear.z, self.config.vel_z)
        twistMsg.angular.x = applyLimit(twistMsg.angular.x, self.config.ang_vel_x)
        twistMsg.angular.y = applyLimit(twistMsg.angular.y, self.config.ang_vel_y)
        twistMsg.angular.z = applyLimit(twistMsg.angular.z, self.config.ang_vel_z)
        return twistMsg

        
def runLimiter():
    """
    Subscribe to twist topic and apply vel limits
    """
    pub = rospy.Publisher('cmd_vel_boosted', Twist, queue_size=10)
    rospy.init_node('apply_minimum_vel')

    processor = MsgProcessor()
    callback = lambda msg : pub.publish(processor.processMsg(msg))        
    rospy.Subscriber("cmd_vel", Twist, callback)
    
    rospy.spin()

if __name__ == '__main__':
    runLimiter()
