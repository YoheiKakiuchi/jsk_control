import imp
try:
  imp.find_module("geometry_msgs")
  from std_msgs.msg import String
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')
  from std_msgs.msg import String

import rospy
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from joy_cmd_vel import JoyCmdVel

class JoyWRS2020(JoyCmdVel):
  '''
Usage:
up/down/left/right and +L2 or +R2 (12 pattern)
circle/cross/triangle/square and +L2 or +R2 (12 pattern)
(L3:)
select:
start:
Args:
output_topic
  '''
  def __init__(self, name, args):
    JoyCmdVel.__init__(self, name, args)
    self.output_topic = self.getArg("command_topic", "command_text")
    ##self.walk_joy_pub = rospy.Publisher(self.output_topic, Joy)
    self.command_text_pub = rospy.Publisher(self.output_topic, String, queue_size=1)

  def joyCB(self, status, history):
      JoyCmdVel.joyCB(self, status, history)
      st = String()
      st.data = ':none'
      ##
      while history.length() > 0:
        latest = history.latest()
        if (not latest.circle) and status.circle:
          ## circle(B) pressed
          if status.L2:
            st.data = ':stop-auto-balancer'
          elif status.R2:
            st.data = ':circle_r'
          else:
            st.data = ':circle'
          break
        if (not latest.cross) and status.cross:
          ## cross(A) pressed
          if status.L2:
            st.data = ':start-auto-balancer'
          elif status.R2:
            st.data = ':cross_r'
          else:
            st.data = ':cross'
          break
        if (not latest.triangle) and status.triangle:
          ## triangle(Y) pressed
          if status.L2:
            st.data = ':standup-to-init-pose'
          elif status.R2:
            st.data = ':triangle_r'
          else:
            st.data = ':triangle'
          break
        if (not latest.square) and status.square:
          ## square pressed
          if status.L2:
            st.data = ':square_l'
          elif status.R2:
            st.data = ':fall-front'
          else:
            st.data = ':square'
          break
        ###
        if (not latest.up) and status.up:
          ## up pressed
          if status.L2:
            st.data = ':swarm-front'
          elif status.R2:
            st.data = ':standup-from-front'
          else:
            st.data = ':up'
          break
        if (not latest.down) and status.down:
          ## down pressed
          if status.L2:
            st.data = ':swarm-back'
          elif status.R2:
            st.data = ':standup-from-back'
          else:
            st.data = ':down'
          break
        if (not latest.right) and status.right:
          ## right pressed
          if status.L2:
            st.data = ':right_l'
          elif status.R2:
            st.data = ':standup-from-right-side'
          else:
            st.data = ':right'
          break
        if (not latest.left) and status.left:
          ## left pressed
          if status.L2:
            st.data = ':left_l'
          elif status.R2:
            st.data = ':standup-from-left-side'
          else:
            st.data = ':left'
          break
        break
      ###
      if not (st.data == ':none'):
        self.command_text_pub.publish(st)

## buttons
## misc
# .center
# .select
# .start
## cross keypad
# .up
# .down
# .left
# .right
## right buttons
# .circle   B
# .cross    A
# .triangle Y
# .square   X
## shoulder buttons
# .L1
# .R1
# .L2
# .R2
## stick buttons
# .L3
# .R3
## analog axis
# .left_analog_x = 0.0
# .left_analog_y = 0.0
# .right_analog_x = 0.0
# .right_analog_y = 0.0
## analog as buttons
# .left_analog_up = False
# .left_analog_down = False
# .left_analog_left = False
# .left_analog_right = False
