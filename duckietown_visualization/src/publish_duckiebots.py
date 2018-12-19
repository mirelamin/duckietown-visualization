#!/usr/bin/env python
import rospy
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray
from flock_simulator.msg import FlockState


def callback(state):
    marker_array = MarkerArray()
    counter = 0

    for it in state.duckie_states:
        duckie_id = it.duckie_id.data
        try:
            (trans, rot) = listener.lookupTransform('/duckiebot_link',
                                                    duckie_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue
        marker_array.markers.append(
            get_duckiebot_marker(counter, trans[0], trans[1], rot))
        counter += 1

    for request in state.requests:
        request_id = '%s' % request.request_id.data
        if not request.duckie_id.data:
            try:
                (trans_start, rot_start) = listener.lookupTransform(
                    '/request_link', request_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
            marker_array.markers.append(
                get_request_marker(counter, trans_start[0], trans_start[1],
                                   rot_start, True))
            counter += 1
        else:
            try:
                (trans_end, rot_end) = listener.lookupTransform(
                    '/request_link', request_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
            marker_array.markers.append(
                get_request_marker(counter, trans_end[0], trans_end[1],
                                   rot_end, False))
            counter += 1

    pub.publish(marker_array)


def get_request_marker(marker_id, x, y, q, isStart):
    marker = Marker()

    marker.header.frame_id = "/request_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.05

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    if isStart:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    else:
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    return marker


def get_duckiebot_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/duckiebot_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot.dae"
    marker.mesh_use_embedded_materials = True

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


if __name__ == '__main__':
    rospy.init_node('duckiebot_marker_publisher')

    tfCacheDuration = rospy.Duration.from_sec(0.1)
    listener = tf.TransformListener(tfCacheDuration)

    pub = rospy.Publisher('duckiebots_markers', MarkerArray, queue_size=10)

    rospy.Subscriber('/flock_simulator/state', FlockState, callback)

    rospy.spin()
