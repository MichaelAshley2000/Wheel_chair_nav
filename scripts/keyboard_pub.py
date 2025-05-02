#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
import select
from geometry_msgs.msg import Twist

def get_key_nonblocking(timeout=0.1):
    """Checks for key press in a non-blocking manner."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def keyboard_control():
    rospy.init_node("keyboard_teleop", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    speed_linear = 0.5
    speed_angular = 0.5
    rate = rospy.Rate(20)  # 20Hz loop rate

    rospy.loginfo("Press & hold W/A/S/D to move, E to light LED, release to stop. Ctrl+C to quit.")

    try:
        while not rospy.is_shutdown():
            key = get_key_nonblocking(timeout=0.05)
            twist = Twist()

            if key is not None:
                key = key.lower()
                rospy.loginfo(f"Key pressed: {repr(key)}")

                if key == 'w':
                    twist.linear.x = speed_linear
                elif key == 's':
                    twist.linear.x = -speed_linear
                elif key == 'd':
                    twist.angular.z = speed_angular
                elif key == 'a':
                    twist.angular.z = -speed_angular
                elif key == 'e':
                    # <-- Light up LED on Arduino
                    twist.angular.x = 1.0
                    rospy.loginfo("Turning LED ON on Arduino!")
                elif key == '\x03':  # Ctrl+C
                    rospy.loginfo("Exiting teleop...")
                    break

            # Publish the command
            pub.publish(twist)

            # Stop motors & LED if no key is pressed
            if key is None:
                stop_msg = Twist()
                pub.publish(stop_msg)

            rate.sleep()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        rospy.loginfo("Shutting down teleop...")

if __name__ == "__main__":
    keyboard_control()
