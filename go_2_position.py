"""
This Function get a series of directions and just executes them
"""
import rospy
from geometry_msgs.msg import Twist
from math import radians

class goto_2_position(float distance float angle):
      def __init__(self):
         # initializing node
         rospy.init_node('goto2position', anonymous=False)
         
         # on control+c
         rospy.on_shutdown(self.shutdown)

         self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
         
         # refresh rate
         r=rospy.Rate(5);

         move_cmd=Twist()
         move_cmd.linear.x = 0.2
         turn_cmd =Twist()
         turn_cmd.linear.x=0
         turn_cmd.angular.z=radians(45)# 45 radians per second
         
         seconds_to_turn=angle/45
         n_iterations_turn=r*seconds_to_turn
         seconds_to_move=distance/0.2
         n_iterations_move=r*seconds_to_move

         rospy.loginfo("turning")
         for theta in range(0,n_iterations_turn):
         	 self.cmd_vel.publish(turn_cmd)
         	 r.sleep()
         rospy.loginfo("Moving")
         for x in range(0,n_iterations_move):
         	 self.cmd_vel.publish(move_cmd)
         	 r.sleep()
      def shutdown(self):
          #stops turtlebot
          rospy.loginfo("Stop")
          self.cmd_vel.publish(Twist())
          rospy.sleep(1)
if __name__ == '__main__':
    try:
        goto_2_position(1.0,45) 


