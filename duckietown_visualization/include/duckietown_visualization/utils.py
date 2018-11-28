import os
import math
import rospkg
import rospy

from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def get_single_marker(x, y, marker_type, marker_pose, 
    marker_id, marker_class, tile_size):

    marker = Marker()

    marker.header.frame_id = "/map"
    marker.id = marker_id
    marker.ns = marker_class

    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD

    marker.pose.position.x = x * tile_size
    marker.pose.position.y = y * tile_size
    marker.pose.position.z = 0

    marker.scale.x = tile_size
    marker.scale.y = tile_size
    marker.scale.z = 1

    marker.mesh_resource = "package://duckietown_visualization/meshes/" + \
        marker_type + ".dae"

    marker.mesh_use_embedded_materials = True

    q = quaternion_from_euler(0, 0, 0)

    if marker_pose == "W":
        q = quaternion_from_euler(0, 0, math.pi)
    elif marker_pose == "N":
        q = quaternion_from_euler(0, 0, math.pi/2)
    elif marker_pose == "S":
        q = quaternion_from_euler(0, 0, -math.pi/2)

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    if marker_type == "curve_left" or marker_type == "curve_right":
        marker.scale.x = tile_size * 1.025  # looks better
        marker.scale.y = tile_size * 1.025

    return marker


def get_marker_array_from_map(m, tile_size, known_types):
    
    marker_array = MarkerArray()
    
    # Add all tiles
    tiles = m['tilemap'].children
    marker_class = 'tiles'
    marker_size = tile_size
    marker_id = 0

    for tile_id, tile in tiles.items():
        _, c, r = tile_id.split('-')
        c = int(c)
        r = int(r)
        marker_type = tile.kind
        marker_pose = "E"

        if marker_type not in known_types:
            rospy.logerr("Unknown tile type: %s", marker_type)

        marker = get_single_marker(c, r, marker_type, marker_pose, 
            marker_id, marker_class, marker_size)

        rospy.loginfo("Created marker for %s", marker_type)
        marker_array.markers.append(marker)
        marker_id += 1

    return marker_array


def get_list_of_supported_types():
    rospack = rospkg.RosPack()
    path = rospack.get_path('duckietown_visualization')
    file_list = os.listdir(os.path.join(path, 'meshes'))
    types = set([f.split('.')[0] for f in file_list])
    return types
