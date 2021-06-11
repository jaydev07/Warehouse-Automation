cd #!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def move_circle():
	rospy.init_node('turtle_revolve', anonymous=True)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
	
	print("Let's move your robot")
	speed =70 
        radius=10

	vel_msg.linear.x=abs(speed)	
	vel_msg.linear.y=0
	vel_msg.linear.z=0

        vel_msg.angular.x=0
	vel_msg.angular.y=0
	vel_msg.angular.z=speed/radius

	while not rospy.is_shutdown():
		pub.publish(vel_msg)

	vel_msg.linear.x=0
	vel_msg.linear.z=0
	pub.publish(vel_msg)

        rospy.spin()


if __name__ == "__main__":
	move_circle_sever()		
