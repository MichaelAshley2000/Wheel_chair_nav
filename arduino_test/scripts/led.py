#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import sys
import termios
import tty

def get_key():
    """Reads a single character from keyboard."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def keyboard_publisher():
    rospy.init_node("keyboard_publisher", anonymous=True)
    pub = rospy.Publisher("led_toggle", Bool, queue_size=10)

    rospy.loginfo("Press 'W' to toggle LED. Press 'Q' to exit.")
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        key = get_key()
        if key.lower() == "q":
            rospy.loginfo("Exiting...")
            break
        elif key.lower() == "w":
            rospy.loginfo("W pressed! Sending message...")
            pub.publish(True)

        rate.sleep()

if __name__ == "__main__":
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass
