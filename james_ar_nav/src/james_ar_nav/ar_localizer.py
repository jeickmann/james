#!/usr/bin/env python

'''james_ar_localization ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
import geometry_msgs
import tf
import tf2_kdl
import tf2_ros
from tf import transformations

def get_pose_for_marker(marker_id):
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 1
    pose.position.z = 3

#def matrix(transform):
#    trans = transformations.translation_matrix(transform.transform.translation.)
#    rot = transformations.quaternion_matrix(map_to_marker.transform.rotation)
#    return transformations.concatenate_matrices(trans, rot)

def ar_tag_callback(markers):
    for marker in markers.markers:
        rospy.logwarn("Found marker ")
        
        id = marker.id
        found_frame_id = "ar_marker_" + str(id)
        position_frame_id = "marker_" + str(id)
        map_to_marker = tfBuffer.lookup_transform("map", position_frame_id, rospy.Time(0))
        odom_to_marker = tfBuffer.lookup_transform("odom", found_frame_id, rospy.Time(0))
        marker_to_odom = tfBuffer.lookup_transform(found_frame_id, "odom", rospy.Time(0))
        rospy.logwarn("Translation: " + str(marker_to_odom.transform.translation))
        #result_mat = transformations.concatenate_matrices(matrix(map_to_marker), matrix(marker_to_odom))
        map2mark_kdl = tf2_kdl.tf2_kdl.transform_to_kdl(map_to_marker)
        mark2odom_kdl = tf2_kdl.tf2_kdl.transform_to_kdl(marker_to_odom)
        map2odom_kdl = map2mark_kdl * mark2odom_kdl


        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        #tf2_ros.convert(map2odom_kdl, geometry_msgs.msg.TransformStamped())
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = map2odom_kdl.p.x()
        t.transform.translation.y = map2odom_kdl.p.y() #map_to_marker.transform.translation.y - odom_to_marker.transform.translation.y
        t.transform.translation.z = 0
        
        
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = map2odom_kdl.M.GetQuaternion()


        #t.transform.rotation = odom_to_marker.transform.rotation
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
