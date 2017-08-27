#!/usr/bin/env python

'''james_ar_localization ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
import tf
import tf2_ros

def get_pose_for_marker(marker_id):
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 1
    pose.position.z = 3

def ar_tag_callback(markers):
    for marker in markers:
        id = marker.id
        trans = tfBuffer.lookup_transform("odom", marker.pose.header.frame_id, rospy.Time())
        odom_pose = tf2_geometry_msgs.tf2_geometry_msgs.do_transform_pose(marker.pose, trans)
        map_pose = get_pose_for_marker(id)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = map_pose.position.x - odom_pose.pose.position.x
        t.transform.translation.y = map_pose.position.y - odom_pose.pose.position.y
        t.transform.translation.z = map_pose.position.z - odom_pose.pose.position.z
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    pass

def init():
    '''james_ar_localization Publisher'''
    rospy.init_node('james_ar_localization', anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_tag_callback)
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
