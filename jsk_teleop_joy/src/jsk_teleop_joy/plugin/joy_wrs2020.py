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
  def __init__(self, name, args):
    JoyCmdVel.__init__(self, name, args)
    self.output_topic = self.getArg("output_topic", "command_text")
    ##self.walk_joy_pub = rospy.Publisher(self.output_topic, Joy)
    self.command_text_pub = rospy.Publisher(self.output_topic, String)

  def joyCB(self, status, history):
      JoyCmdVel.joyCB(self, status, history)
      st = String()
      st.data = 'none'
      ##
      if   (not latest.circle) and status.circle:
        ## circle pressed
        st.data = ''
      elif (not latest.circle) and status.circle:
      ##
      self.command_pub.publish(st)

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
# .circle
# .cross
# .triangle
# .square
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
