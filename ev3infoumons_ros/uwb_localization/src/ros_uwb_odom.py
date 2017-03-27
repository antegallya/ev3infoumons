#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Quaternion, Vector3
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from ranging.engine import Engine
from ranging.tag import Tag
import random

# Variance for a known value.
known = 1e-4
# Variance for an unknown value.
unknown = 1e6
# Default variance on the position's axes.
pos_var = 1e-2
topic = 'odom'
# FIXME: this should be a parameter.
child_frame_id = 'base_footprint'

connected = False

def idx6x6(i, j): return i * 6 + j

class UWBRanging:
  def __init__(self):
    self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('uwb_odom', anonymous = True)
    rospy.loginfo("Loaded UWB Ranging node")
    self.seq = 0
    self.engine = Engine()


    self.engine.addAnchor(0x09, -2.35, -2.35, 1.04)
    self.engine.anchors[0].rx_delay = 32709 # 0x09 
    self.engine.addAnchor(0x0A, -4.61, 2.21, 1.60)
    self.engine.anchors[1].rx_delay = 33243 # 0x0A
    self.engine.addAnchor(0x08, -0.4, -0.59, 1.15)
    self.engine.anchors[2].rx_delay = 32691 # 0x08
    self.engine.addAnchor(0x06, -0.45, 2.31, 1.54)
    self.engine.anchors[3].rx_delay = 32739 # 0x06

    self.tag = Tag(0x07, "Tag boite", self.engine.anchors)
    self.tag.rx_delay = 32835 # 0x07
    # posx, posy, posz = Tag.computPos(tag.distances)
    # tag.setPos(posx, posy, posz)
    self.engine.addTag(self.tag)

    if(connected):
      self.engine.initComm("/dev/ttyUSB0")
      self.engine.initAntennaDelay()

      self.engine.updatePosTag()

  def pubLoop(self):
    rate = rospy.Rate(1) # Hz
    rospy.loginfo("Started UWB Ranging")
    while not rospy.is_shutdown():
      if(connected):
        self.engine.updatePosTag()
      else:
        self.tag.randomPos()
      self.publishTagPos(self.tag.x, self.tag.y)
      rate.sleep()
    rospy.loginfo("Stopped UWB Ranging")

  def publishTagPos(self, x, y):
    now = rospy.Time.now()
    header = rospy.Header(seq = self.seq, stamp = now, frame_id = topic)
    self.seq += 1

    # The position is encoded here.
    position = Point(x = x, y = y, z = 0)

    # We have no orientation information with UWB ranging.
    orientation = Quaternion(x = 0, y = 0, z = 0, w = 1)
    pose = Pose(position = position, orientation = orientation)
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z
    # axis)
    covariance = [0,] * 36
    # Variances for X, Y position.
    covariance[idx6x6(0, 0)] = pos_var
    covariance[idx6x6(1, 1)] = pos_var
    # Our z, roll and pitch should be constant 0 (it's a car on flat ground).
    covariance[idx6x6(2, 2)] = known
    covariance[idx6x6(3, 3)] = known
    covariance[idx6x6(4, 4)] = known
    # We don't know the yaw. Might be anything.
    covariance[idx6x6(5, 5)] = unknown
    pose_w_cov = PoseWithCovariance(pose = pose, covariance = covariance)
    # Fill the Twist, but we actually have no velocity information. That's thus
    # all zeros.
    zero = Vector3(0, 0, 0)
    twist = Twist(zero, zero)
    covariance = [0,] * 36
    # X linear twist variance might be anything.
    # FIXME: we could have a more fine-tuned value by taking max linear
    # velocity into consideration.
    covariance[idx6x6(0, 0)] = unknown
    # Y linear twist variance
    covariance[idx6x6(1, 1)] = known
    # Z linear twist variance
    covariance[idx6x6(2, 2)] = known
    # X angular twist variance
    covariance[idx6x6(3, 3)] = known
    # Y angular twist variance
    covariance[idx6x6(4, 4)] = known
    # Z angular twist variance. Might be anything.
    # FIXME: we could have a more fine-tuned value by taking max rotational
    # velocity into consideration.
    covariance[idx6x6(5, 5)] = unknown
    twist_w_cov = TwistWithCovariance(twist = twist, covariance = covariance)
    self.pub.publish(header = header, child_frame_id = child_frame_id,

                     pose = pose_w_cov, twist = twist_w_cov)


if __name__ == '__main__':
  try:
    UWBRanging().pubLoop()
  except rospy.ROSInterruptException:
    pass
