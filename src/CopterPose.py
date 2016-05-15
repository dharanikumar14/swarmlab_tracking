#!/usr/bin/python2

import rospy, math
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Pose2D

from numpy import array, dot, argmin, absolute as abs, arctan2 as atan2, pi
from numpy.linalg import norm

class CopterPose(object):

  def update(self):
    if(not (self.markers[0]!=None and self.markers[1]!=None and self.markers[2]!=None)):
      return
    m0 = self.markers[0]
    m1 = self.markers[1]
    m2 = self.markers[2]
    m01 = m1 - m0
    m12 = m2 - m1
    m02 = m2 - m0
    a = array([abs(dot(m01, m02))/norm(m01)/norm(m02), abs(dot(m01, m12))/norm(m01)/norm(m02), abs(dot(m12, m02))/norm(m12)/norm(m02)])
    x = array([-m01, -m12, m02])
    p = array([m1+0.5*m12, m0+0.5*m02, m0+0.5*m01])
    i = argmin(a)
    pose = Pose2D()
    pose.x     = p[i][0]
    pose.y     = p[i][1]
    pose.theta = atan2(x[i][1], x[i][0])
    if(i==0):
      n = dot(m1-m0, array([[0, 1], [-1, 0]])) - (m2-m0)
    if(i==1):
      n = dot(m2-m1, array([[0, 1], [-1, 0]])) - (m0-m1)
    if(i==2):
      n = dot(m0-m2, array([[0, 1], [-1, 0]])) - (m1-m2)
    #print "i: %i, n: %f"%(i, norm(n))
    if(norm(n)>5):
      pose.theta -= math.pi/2
    thetaQuat = quaternion_about_axis(pose.theta, (0,0,1))
    self.marker.pose.position.x    = pose.x
    self.marker.pose.position.y    = pose.y
    self.marker.pose.position.z    = 0.6
    self.marker.pose.orientation.x = thetaQuat[0]
    self.marker.pose.orientation.y = thetaQuat[1]
    self.marker.pose.orientation.z = thetaQuat[2]
    self.marker.pose.orientation.w = thetaQuat[3]
    self.markerPub.publish(self.marker)
    self.posePub.publish(pose)

  def handleMarkers(self, markers):
    self.markers = []
    output = "Got Markers:\n"
    for marker in markers.markers:
      markerPose = array([marker.pose.position.x, marker.pose.position.y])
      self.markers.append(markerPose)
      output += "\t" + str(markerPose) + "\n"
      rospy.logdebug(output)
    self.update()

  def die(self, agent, id):
    print "Dying because %r(%d) told us so"%(agent, id)
  
  def __init__(self):
    print "Creating Subscribers"
    self.marker  = Marker()
    self.marker.type = Marker.ARROW
    self.marker.action = Marker.ADD
    self.marker.id = 0
    self.marker.ns = "copters"
    self.marker.color.r = 1
    self.marker.color.a = 1
    self.marker.lifetime = rospy.Duration(1.0)
    self.marker.header.frame_id = "map"
    self.marker.header.stamp = rospy.Time.now()
    self.marker.scale.x = 0.2
    self.marker.scale.y = 0.05
    self.marker.scale.z = 0.05
    self.markerPub = rospy.Publisher("marker", Marker, queue_size=1)
    self.posePub = rospy.Publisher("pose", Pose2D, queue_size=1)
    self.sub = rospy.Subscriber("markers", MarkerArray, self.handleMarkers)
    print "Subscribers created"

if __name__=="__main__":
  rospy.init_node("PoseEstimator")
  poseEstimator = CopterPose()
  rospy.spin()
