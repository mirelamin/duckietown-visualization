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
            get_duckiebot_marker(counter, it.status.data, trans[0], trans[1],
                                 rot))
        counter += 1

    for request in state.requests:
        request_id = '%s' % request.request_id.data
        is_start = False if request.duckie_id.data else True
        try:
            (trans_mark, rot_mark) = listener.lookupTransform(
                '/request_link', request_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue
        marker_array.markers.append(
            get_request_marker(counter, trans_mark[0], trans_mark[1], rot_mark,
                               is_start))
        counter += 1

        if request_id not in previous_marker_ids:
            previous_marker_ids.append(request_id)

    for _ in previous_marker_ids:
        marker_array.markers.append(get_empty_marker(counter))
        counter += 1

    pub.publish(marker_array)


def get_empty_marker(marker_id):
    marker = Marker()

    marker.header.frame_id = "/request_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    return marker


def get_request_marker(marker_id, x, y, q, is_start):
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

    if is_start:
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


def get_duckiebot_marker(marker_id, status, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/duckiebot_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.mesh_use_embedded_materials = True
    marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot_blue.dae"
    if status == 'REBALANCING':
        marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot_green.dae"
    elif status == 'DRIVINGTOCUSTOMER':
        marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot_yellow.dae"
    elif status == 'DRIVINGWITHCUSTOMER':
        marker.mesh_resource = "package://duckietown_visualization/meshes/duckiebot/duckiebot_orange.dae"

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

    listener = tf.TransformListener()

    pub = rospy.Publisher('duckiebots_markers', MarkerArray, queue_size=10)

    previous_marker_ids = []

    rospy.Subscriber('/flock_simulator/state', FlockState, callback)

    rospy.spin()
