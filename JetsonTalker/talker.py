import rospy
from std_msgs.msg import String

def string_to_hex(string):
    # Convert input to a string if it is not already a string
    string = str(string)
    
    # Convert string to binary integer
    binary_int = int(string, 2)
    
    # Convert binary integer to hex
    hexcode = hex(binary_int)[2:].upper()
    
    return hexcode
    
rospy.init_node('message_sender')
pub = rospy.Publisher('/my_topic', String, queue_size=10)

rate = rospy.Rate(1) # publish once per second
while not rospy.is_shutdown():
    message = String()
    message.data = input("enter RIS config : ")
    message.data = "0x"+ string_to_hex(message.data)
    pub.publish(message)
    rospy.loginfo("Published message: %s", message.data)
    rate.sleep()
