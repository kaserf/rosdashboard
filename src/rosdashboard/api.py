import rospy
from std_msgs.msg import String, Int32, Float32, Bool

def log(tag, data):
    """
    publishes data to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    
    The method first determines if you want to publish a string, integer, float
    or boolean and then redirects the call to logstring, logint, logfloat or logbool.
    """
    if type(data) == int:
        logint(tag, data)
    elif type(data) == str:
        logstring(tag, data)
    elif type(data) == float:
        logfloat(tag, data)
    elif type(data) == bool:
        logbool(tag, data)
    else:
        print "Could not determine the type of data: " + str(type(data))

def logstring(tag, msg):
    """
    publishes a string message to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    """
    pub = rospy.Publisher('/rosdashboard/' + tag, String)
    pub.publish(String(msg))

def logint(tag, value, msg_type=Int32):
    """
    publishes a integer value to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    
    The default integer message type is std_msg.msg.Int32
    """
    pub = rospy.Publisher('/rosdashboard/' + tag, msg_type)
    pub.publish(msg_type(value))
    
def logfloat(tag, value, msg_type=Float32):
    """
    publishes a float value to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    
    The default float message type is std_msg.msg.Float32
    """
    pub = rospy.Publisher('/rosdashboard/' + tag, msg_type)
    pub.publish(msg_type(value))

def logbool(tag, value):
    """
    publishes a boolean value to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    
    The message type is std_msg.msg.Bool
    """
    pub = rospy.Publisher('/rosdashboard/' + tag, Bool)
    pub.publish(value)

def logdata(tag, data, msg_type):
    """
    publishes arbitrary data to the /rosdashboard/<tag> topic
    Be careful to use only valid tags, ros does not allow dashes and dots in
    the topic name.
    """
    pub = rospy.Publisher('/rosdashboard/' + tag, msg_type)
    pub.publish(data)
