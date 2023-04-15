import rospy
from std_msgs.msg import String

rospy.init_node('message_sender') # Initialize the node with a name

# Create a publisher object that will publish messages to the '/my_topic' topic
pub = rospy.Publisher('/my_topic', String, queue_size=10)

# Create a message object
msg = String()
msg.data = "Hello, world!"

# Publish the message to the topic
pub.publish(msg)

# Spin the node to keep it running
rospy.spin()