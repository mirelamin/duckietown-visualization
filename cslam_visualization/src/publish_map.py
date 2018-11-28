#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
import yaml
import math

GRID_SIZE = 0.61
known_types = ["curve_left","curve_right","straight","4way","3way_left","3way_right","4way","grass"]
'''
map_data_yaml = """

# 3x3 tiles with left turns at the corners going in a counter-clockwise loop

tiles:
- [curve_left/W , straight/W , curve_left/N]
- [straight/S   , grass      , straight/N  ]
- [curve_left/S , straight/E , curve_left/E]

"""
'''

map_data_yaml = """

tiles:
- [curve_right/N , straight/E, 3way_right/E, straight/E, curve_right/E]
- [straight/N   , grass     , straight/S , grass     , straight/S  ]
- [3way_right/N  , straight/W, 4way       , straight/W, 3way_right/S ]
- [straight/N   , grass     , straight/S , grass     , straight/S  ]
- [curve_right/W , straight/W, 3way_right/W, straight/W, curve_right/S]

"""


def getSingleMarker(x,y,marker_type,marker_pose,marker_id):
    
    marker = Marker()
    
    marker.header.frame_id = "/map"
    marker.id = marker_id
    
    marker.type =  marker.MESH_RESOURCE
    marker.action = marker.ADD

    marker.pose.position.x = x * GRID_SIZE
    marker.pose.position.y = y * GRID_SIZE
    marker.pose.position.z = 0
    
    marker.scale.x = GRID_SIZE
    marker.scale.y = GRID_SIZE
    marker.scale.z = 1
    
    marker.mesh_resource = "package://cslam_visualization/meshes/"+marker_type+".dae"
    marker.mesh_use_embedded_materials = True
    
    q = quaternion_from_euler(0,0,0)
    
    if marker_pose == "W":
        q = quaternion_from_euler(0,0,math.pi)
    elif marker_pose == "N":
        q = quaternion_from_euler(0,0,math.pi/2)
    elif marker_pose == "S":
        q = quaternion_from_euler(0,0,-math.pi/2)   
    
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    
    if marker_type == "curve_left" or marker_type == "curve_right":
        marker.scale.x = GRID_SIZE * 1.025 #looks better
        marker.scale.y = GRID_SIZE * 1.025

    return marker

def getMarkerArrayFromYAML(map_data_yaml):
    markerArray = MarkerArray()
    map_data = yaml.load(map_data_yaml)
    rospy.loginfo("Loaded map!")
    tiles = map_data['tiles']
    rows = len(tiles)
    cols = len(tiles[0])
    for r in range(rows):
        for c in range(cols):
            marker_info = tiles[r][c].split('/')
            
            marker_type = marker_info[0]
            marker_pose = ""
            if len(marker_info) == 2:
                marker_pose = marker_info[1]
            
            if marker_type not in known_types:
                rospy.logerr("Unknown tile type: %s",marker_type)
            
            marker = getSingleMarker(c-(cols-1)/2,(rows-1)/2-r,marker_type,marker_pose,r*cols+c)
            rospy.loginfo("Created marker for %s", marker_type)
            markerArray.markers.append(marker)

    return markerArray
                
if __name__ == '__main__':
    
    rospy.init_node('duckietown_map_publisher', anonymous=False)
    
    pub = rospy.Publisher('duckietown_map', MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    marker_array = getMarkerArrayFromYAML(map_data_yaml)
    rospy.loginfo("Marker array created. Now publishing!")
    
    while not rospy.is_shutdown():
        pub.publish(marker_array)
        rate.sleep()

