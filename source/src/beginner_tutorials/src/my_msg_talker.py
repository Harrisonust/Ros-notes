#!/usr/bin/env python
# license removed for brevity
import rospy
from beginner_tutorials.msg import my_msg


def talker():
    pub = rospy.Publisher('chatter', my_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    count = 1
    while not rospy.is_shutdown():
        msg = my_msg()
        msg.id = count
        msg.title = "hello"
        msg.content = "hello from python"
        rospy.loginfo(msg.id)
        pub.publish(msg)
        count = count + 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
