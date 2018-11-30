#!/usr/bin/env python
import rospy
import math
import tf
from visualization_msgs.msg import Marker, MarkerArray
from flock_simulator.msg import FlockState


def callback(bots):
    marker_array = MarkerArray()
    counter = 0

    for it in bots.duckie_states:
        duckie_id = it.duckie_id.data
        try:
            (trans,rot) = listener.lookupTransform('/duckiebot_link', \
                duckie_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue
        marker_array.markers.append(
            get_marker(counter, trans[0], trans[1], rot))
        counter += 1

    pub.publish(marker_array)


def get_marker(marker_id, x, y, q):
    marker = Marker()

    marker.header.frame_id = "/duckiebot_link"
    marker.id = marker_id
    marker.ns = "duckiebots"

    marker.type = marker.CUBE
    marker.action = marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.scale.x = 0.2
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


if __name__ == '__main__':
    rospy.init_node('duckiebot_marker_publisher')

    listener = tf.TransformListener()

    duckiebot_list = rospy.get_param('~duckiebot_list')
    pub = rospy.Publisher('duckiebots_markers', MarkerArray, queue_size=10)

    rate = rospy.Rate(10.0)
    rospy.Subscriber('/flock_simulator/state', FlockState, callback)

    rospy.spin()