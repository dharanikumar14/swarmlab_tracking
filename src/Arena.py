#!/usr/bin/python2.7

import rospy, math
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_about_axis

class Arena:
    walls = (
        Point(x=-1.0, y= 3.0),
        Point(x= 2.0, y= 3.0),
        Point(x= 2.0, y=-1.0),
        Point(x=-1.0, y=-1.0),
        Point(x=-1.0, y= 3.0))
    def __init__(self):
        self.pub = rospy.Publisher("arena", MarkerArray, queue_size=1)
        self.marker_msg = MarkerArray()
        walls = Marker()
        walls.header.frame_id = "map"
        walls.type = Marker.LINE_STRIP
        walls.action = Marker.ADD
        walls.id = 0
        walls.ns = "arena"
        walls.color.r = 1.0
        walls.color.g = 1.0
        walls.color.b = 1.0
        walls.color.a = 1.0
        walls.pose.orientation.w = 1.0
        walls.scale.x = 0.1
        zero = Marker()
        zero.header.frame_id = "map"
        zero.type = Marker.SPHERE
        zero.action = Marker.ADD
        zero.id = 1
        zero.ns = "arena"
        zero.color.g = 1.0
        zero.color.a = 1.0
        zero.pose.orientation.w = 1.0
        zero.scale.x = 0.1
        zero.scale.y = 0.1
        zero.scale.z = 0.1
        x = Marker()
        x.header.frame_id = "map"
        x.type = Marker.ARROW
        x.action = Marker.ADD
        x.id = 2
        x.ns = "arena"
        x.color.r = 1.0
        x.color.a = 1.0
        x.pose.orientation.w = 1.0
        x.scale.x = 0.5
        x.scale.y = 0.03
        x.scale.z = 0.03
        y = Marker()
        y.header.frame_id = "map"
        y.type = Marker.ARROW
        y.action = Marker.ADD
        y.id = 3
        y.ns = "arena"
        y.color.b = 1.0
        y.color.a = 1.0
        yQuat = quaternion_about_axis(math.pi/2, (0,0,1))
        y.pose.orientation.x = yQuat[0]
        y.pose.orientation.y = yQuat[1]
        y.pose.orientation.z = yQuat[2]
        y.pose.orientation.w = yQuat[3]
        y.scale.x = 0.5
        y.scale.y = 0.03
        y.scale.z = 0.03
        for point in Arena.walls:
            walls.points.append(point)
        for i in range(0,  len(Arena.walls)):
            walls.colors.append(walls.color)
        self.marker_msg.markers.append(walls)
        self.marker_msg.markers.append(zero)
        self.marker_msg.markers.append(x)
        self.marker_msg.markers.append(y)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_cb)

    def publish_cb(self,  timeEvent):
        for marker in  self.marker_msg.markers:
            marker.header.stamp = rospy.Time.now()
        self.pub.publish(self.marker_msg)

if __name__=="__main__":
    rospy.init_node("arena")
    arena = Arena()
    while not rospy.is_shutdown():
        rospy.spin()
