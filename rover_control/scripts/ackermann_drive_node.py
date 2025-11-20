# rover_control/scripts/ackermann_drive_node.py
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from rover_arduino_drive.msg import DriveCommand
import math

WHEEL_BASE = 0.5   # distance between front & rear axle [m]
TRACK_WIDTH = 0.4  # distance between left & right wheels [m]
MAX_STEER_ANGLE = math.radians(40)
MAX_WHEEL_SPEED  = 10.0  # rad/s

pub = None

def twist_cb(msg):
  v = msg.linear.x
  w = msg.angular.z

  cmd = DriveCommand()

  if abs(w) < 1e-3:
    # Straight
    omega = v / 0.1  # wheel radius ~0.1
    cmd.fl_velocity = cmd.ml_velocity = cmd.rl_velocity = omega
    cmd.fr_velocity = cmd.mr_velocity = cmd.rr_velocity = omega
    cmd.fl_steer_angle = cmd.fr_steer_angle = 0.0
    cmd.rl_steer_angle = cmd.rr_steer_angle = 0.0
  else:
    R = v / w
    R_left  = R - TRACK_WIDTH/2.0
    R_right = R + TRACK_WIDTH/2.0
    # Steering angles (front & rear Ackermann-like)
    delta_fl = math.atan2(WHEEL_BASE/2.0, R_left)
    delta_fr = math.atan2(WHEEL_BASE/2.0, R_right)
    delta_rl = -delta_fl
    delta_rr = -delta_fr

    # Clamp
    for a in [delta_fl, delta_fr, delta_rl, delta_rr]:
      a = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, a))

    # Wheel angular velocity magnitude
    v_fl = w * (R_left  - WHEEL_BASE/2.0)
    v_fr = w * (R_right - WHEEL_BASE/2.0)
    v_rl = w * (R_left  + WHEEL_BASE/2.0)
    v_rr = w * (R_right + WHEEL_BASE/2.0)
    r = 0.1
    cmd.fl_velocity = v_fl / r
    cmd.ml_velocity = (v_fl + v_rl)/(2*r)
    cmd.rl_velocity = v_rl / r
    cmd.fr_velocity = v_fr / r
    cmd.mr_velocity = (v_fr + v_rr)/(2*r)
    cmd.rr_velocity = v_rr / r

    cmd.fl_steer_angle = delta_fl
    cmd.fr_steer_angle = delta_fr
    cmd.rl_steer_angle = delta_rl
    cmd.rr_steer_angle = delta_rr

  # Saturate speeds
  for name in ['fl_velocity','ml_velocity','rl_velocity',
               'fr_velocity','mr_velocity','rr_velocity']:
    val = getattr(cmd, name)
    if val > MAX_WHEEL_SPEED: val = MAX_WHEEL_SPEED
    if val < -MAX_WHEEL_SPEED: val = -MAX_WHEEL_SPEED
    setattr(cmd, name, val)

  pub.publish(cmd)

if __name__ == "__main__":
  rospy.init_node("ackermann_drive_node")
  pub = rospy.Publisher("rover/drive_cmd", DriveCommand, queue_size=10)
  rospy.Subscriber("cmd_vel", Twist, twist_cb)
  rospy.spin()
