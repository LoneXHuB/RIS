import rospy
from std_msgs.msg import String

rospy.init_node('message_sender')
pub = rospy.Publisher('/my_topic', String, queue_size=10)

rate = rospy.Rate(1) # publish once per second
while not rospy.is_shutdown():
    message = String()
    message.data = "Hello, world!"
    pub.publish(message)
    rospy.loginfo("Published message: %s", message.data)
    rate.sleep()
