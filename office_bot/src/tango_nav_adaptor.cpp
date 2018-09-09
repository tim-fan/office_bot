#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

// node for connecting tango_ros_streamer outputs to nav stack.
// responsible for:
//  updating timestamps to current time (tango timestamps have been way off - haven't debugged yet)
//  renaming frame ids
//
// see tango_nav_adaptor.py for a first implementation, which is rewritten here in cpp
// for efficiency reasons.


geometry_msgs::TransformStamped project_to_2d(geometry_msgs::TransformStamped tf3d)
{
    //given a 3d transform, return that transform projected down onto the z=0 plane
    double yaw = tf::getYaw(tf3d.transform.rotation);
    geometry_msgs::Quaternion quaternionYawOnly = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::TransformStamped tfProjected = geometry_msgs::TransformStamped(tf3d);
    tfProjected.transform.translation.z = 0;
    tfProjected.transform.rotation = quaternionYawOnly;
    // tfProjected.transform.rotation = tf3d.transform.rotation;
    return tfProjected;
}

void adf_t_sos_callback(const geometry_msgs::TransformStamped msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped adaptedTf;

    adaptedTf = project_to_2d(msg);
    adaptedTf.header.stamp = ros::Time::now();
    adaptedTf.child_frame_id = "odom";
    adaptedTf.header.frame_id = "map";

    br.sendTransform(adaptedTf);
}

void sos_t_device_callback(const geometry_msgs::TransformStamped msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped adaptedTf;

    adaptedTf = project_to_2d(msg);
    adaptedTf.header.stamp = ros::Time::now();
    adaptedTf.child_frame_id = "tango";
    adaptedTf.header.frame_id = "odom";

    br.sendTransform(adaptedTf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tango_nav_adaptor");
    ros::NodeHandle node;
    ros::Subscriber subAdfTSos = node.subscribe("tango/transform/area_description_T_start_of_service", 10, &adf_t_sos_callback);
    ros::Subscriber subSosTDevice = node.subscribe("tango/transform/start_of_service_T_device", 10, &sos_t_device_callback);
    ros::spin();
    return 0;
}
