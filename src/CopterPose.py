#!/usr/bin/python2

import rospy
from geometry_msgs.msg import Pose2D

from numpy import array, dot, argmin, absolute as abs, arctan2 as atan2, pi
from numpy.linalg import norm

from ivy.std_api import *

class CopterPose(object):

	def update(self):
		pose = Pose2D()
		p = None
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
		pose.x     = p[i][0]
		pose.y     = p[i][1]
		pose.theta = 180*atan2(x[i][1], x[i][0])/pi
		if(i==0):
			n = dot(m1-m0, array([[0, 1], [-1, 0]])) - (m2-m0)
		if(i==1):
			n = dot(m2-m1, array([[0, 1], [-1, 0]])) - (m0-m1)
		if(i==2):
			n = dot(m0-m2, array([[0, 1], [-1, 0]])) - (m1-m2)
		if(norm(n)>5):
			pose.theta = pose.theta - 90
		self.pub.publish(pose)
		IvySendMsg("Test")

	def handleMarker0(self, markerPose):
		self.markers[0] = array([markerPose.x, markerPose.y])
		self.update()

	def handleMarker1(self, markerPose):
		self.markers[1] = array([markerPose.x, markerPose.y])
		self.update()

	def handleMarker2(self, markerPose):
		self.markers[2] = array([markerPose.x, markerPose.y])
		self.update()

	def handleConnect(self, agent, event):
		if event == IvyApplicationDisconnected:
			info('Ivy application %r has disconnected', agent)
		else:
			info('Ivy application %r has connected', agent)
		info('Ivy applications currently on the bus: %s', ','.join(IvyGetApplicationList()))


	def die(self, agent, id):
		print "Dying because %r(%d) told us so"%(agent, id)
	
	def __init__(self):
		IvyInit("PoseEstimator", "PoseEstimator is ready", 0, self.handleConnect, self.die)
		print "Creating Subscribers"
		self.markers = [ None, None, None ]
		self.subs    = [ None, None, None ]
		self.pub     = rospy.Publisher("pose", Pose2D, queue_size=10)
		self.subs[0] = rospy.Subscriber("Marker0", Pose2D, self.handleMarker0)
		self.subs[1] = rospy.Subscriber("Marker1", Pose2D, self.handleMarker1)
		self.subs[2] = rospy.Subscriber("Marker2", Pose2D, self.handleMarker2)
		print "Subscribers created"

if __name__=="__main__":
	rospy.init_node("PoseEstimator")
	poseEstimator = CopterPose()
	rospy.spin()
