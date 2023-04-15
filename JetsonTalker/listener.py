import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard: %s", data.data)

rospy.init_node('message_listener')
rospy.Subscriber('/my_topic', String, callback)

rospy.spin()