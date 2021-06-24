#!/usr/bin/env python3
# Copied then modified from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

def parse_waypoints(waypoint_file):
    with open(waypoint_file, "r") as f: 
        yaml_contents = yaml.safe_load(f)
    waypoints = []
    for waypoint in yaml_contents:
        waypoint_name = waypoint['name']
        waypoint_pose = Pose()
        waypoint_pose.position.x = waypoint['position']['x']
        waypoint_pose.position.y = waypoint['position']['y']
        waypoint_pose.position.z = waypoint['position']['z']
        waypoint_pose.orientation.x = waypoint['orientation']['x']
        waypoint_pose.orientation.y = waypoint['orientation']['y']
        waypoint_pose.orientation.z = waypoint['orientation']['z']
        waypoint_pose.orientation.w = waypoint['orientation']['w']
        waypoints.append((waypoint_name, waypoint_pose))
    return waypoints


def movebase_client(target_pose):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = target_pose

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        waypoint_file = rospy.get_param("~waypoint_file")
        waypoints = parse_waypoints(waypoint_file)

        while not rospy.is_shutdown():
        
            for waypoint_name, waypoint_pose in waypoints:
                print(waypoint_name)
                print(waypoint_pose)
                print()
                result = movebase_client(waypoint_pose)
                if result:
                    rospy.loginfo("Goal execution done!")
            # repeat in the opposite direction
            waypoints.reverse()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")