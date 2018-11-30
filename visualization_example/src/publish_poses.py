#!/usr/bin/env python
import rospy
import tf
import math
import random
from flock_simulator.msg import FlockState


def callback(bots):
    for it in bots.duckie_states:
        t = rospy.Time.now()
        theta = it.pose.theta
        x = it.pose.x
        y = it.pose.y

        transform_broadcaster.sendTransform((x, y, 0), \
            tf.transformations.quaternion_from_euler(0, 0, theta), \
            t, it.duckie_id.data, "duckiebot_link")


if __name__ == '__main__':
    rospy.init_node('duckiebot_pose_publisher', anonymous=False)

    rate = rospy.Rate(10)  # 10hz

    transform_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('/flock_simulator/state', FlockState, callback)

    rospy.spin()
