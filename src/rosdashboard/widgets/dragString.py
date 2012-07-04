from PyQt4 import QtGui
from rosdashboard.modules.props import WidgetProperty
import rostopic
import rospy
from rosdashboard.modules.dashboardWidgets import DashboardWidget

class DragString(DashboardWidget):
    """ draggable text field """
    DATASOURCE = 'datasource'
    DATAFIELD = 'datafield'
    
    def __init__(self, parent):
        super(DragString, self).__init__(parent)
        self.setTitle('DragString')
        self.initUI()
        self.initSubscriptions()
        
    def initUI(self):
        
        self.textField = QtGui.QLineEdit(self)
        self.textField.setDisabled(True)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.textField)
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.DATASOURCE] = WidgetProperty('text', '/turtle1/pose')
        self.props[self.DATAFIELD] = WidgetProperty('text', 'linear_velocity')
    
    def propertiesDialogAccepted(self):
        #re-setup the subscription
        self.subscriber.unregister()
        self.initSubscriptions()
        
    def initSubscriptions(self):
        #FIXME: the cast to string is a workaround because subscriber only accepts python strings and not QStrings
        #introspect the type class of the message
        
        data_class = rostopic.get_topic_class(str(self.props[self.DATASOURCE].value), blocking=False)[0]
        print str(data_class)
        if data_class:
            self.subscriber = rospy.Subscriber(str(self.props[self.DATASOURCE].value), data_class, self.subscriptionCallback)
        else:
            print "ERROR: could not find data class for this topic: " + self.props[self.DATASOURCE].value
        
    def subscriptionCallback(self, data):
        #FIXME: remove cast to string
        datafield = getattr(data, str(self.props[self.DATAFIELD].value))
        self.textField.setText(str(datafield))
