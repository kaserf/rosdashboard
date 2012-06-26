from PyQt4 import QtGui
from modules.props import WidgetProperty
import rostopic
import rospy
from modules.dashboardWidgets import DashboardWidget

class DragDial(DashboardWidget):
    """ draggable dial and lcd display """
    MIN = 'minimum'
    MAX = 'maximum'
    DATASOURCE = 'datasource'
    DATAFIELD = 'datafield'
    
    def __init__(self, parent):
        super(DragDial, self).__init__(parent)
        self.setTitle('DragDial')
        self.initUI()
        self.initSubscriptions()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.dial = QtGui.QDial(self)
        self.dial.setSliderPosition(0)
        self.dial.setDisabled(True)
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.dial.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.dial)
        self.layout.addWidget(self.lcd)
        
        #update widget according to properties
        self.updateWidget()
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.MIN] = WidgetProperty('numeric', -2)
        self.props[self.MAX] = WidgetProperty('numeric', 2)
        self.props[self.DATASOURCE] = WidgetProperty('text', '/turtle1/pose')
        self.props[self.DATAFIELD] = WidgetProperty('text', 'linear_velocity')
    
    def propertiesDialogAccepted(self):
        self.updateWidget()
        
        #re-setup the subscription
        self.subscriber.unregister()
        self.initSubscriptions()
        
    def updateWidget(self):
        #update the widget properties
        self.dial.setMinimum(self.props[self.MIN].value)
        self.dial.setMaximum(self.props[self.MAX].value)
        
    def initSubscriptions(self):
        #FIXME: the cast to string is a workaround because subscriber only accepts python strings and not QStrings
        #introspect the type class of the message
        data_class = rostopic.get_topic_class(self.props[self.DATASOURCE].value, blocking=False)[0]
        if data_class:
            self.subscriber = rospy.Subscriber(str(self.props[self.DATASOURCE].value), data_class, self.subscriptionCallback)
        
    def subscriptionCallback(self, data):
        #FIXME: remove cast to string
        datafield = getattr(data, str(self.props[self.DATAFIELD].value))
        self.dial.setValue(int(datafield))