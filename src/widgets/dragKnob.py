from modules.dashboardWidgets import DashboardWidget
from PyQt4 import QtGui
from PyQt4.Qwt5 import Qwt
from modules.props import WidgetProperty
import rostopic
import rospy

class DragKnob(DashboardWidget):
    
    MIN = 'minimum'
    MAX = 'maximum'
    DATASOURCE = 'datasource'
    DATAFIELD = 'datafield'
    
    def __init__(self, parent):
        super(DragKnob, self).__init__(parent)
        self.setTitle('DragKnob')
        self.initUI()
        self.initSubscriptions()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtKnob = Qwt.QwtKnob(self)
        self.qwtKnob.setRange(-5,5)
        self.qwtKnob.setDisabled(True)
        #TODO Make widgets resizeable
        self.qwtKnob.setFixedSize(150, 150)
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.qwtKnob.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.qwtKnob)
        self.layout.addWidget(self.lcd)
        
        #update widget according to properties
        self.updateWidget()
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.MIN] = WidgetProperty('numeric', -4)
        self.props[self.MAX] = WidgetProperty('numeric', 4)
        self.props[self.DATASOURCE] = WidgetProperty('text', '/turtle1/pose')
        self.props[self.DATAFIELD] = WidgetProperty('text', 'linear_velocity')
    
    def propertiesDialogAccepted(self):
        self.updateWidget()
        
        #re-setup the subscription
        self.subscriber.unregister()
        self.initSubscriptions()
        
    def updateWidget(self):
        #update the widget properties
        self.qwtKnob.setRange(self.props[self.MIN].value,
                              self.props[self.MAX].value)
        
    def initSubscriptions(self):
        #FIXME: the cast to string is a workaround because subscriber only accepts python strings and not QStrings
        #introspect the type class of the message
        data_class = rostopic.get_topic_class(self.props[self.DATASOURCE].value, blocking=False)[0]
        if data_class:
            self.subscriber = rospy.Subscriber(str(self.props[self.DATASOURCE].value), data_class, self.subscriptionCallback)
        else:
            print "ERROR: could not find data class for this topic: " + self.props[self.DATASOURCE].value
        
    def subscriptionCallback(self, data):
        #FIXME: remove cast to string
        datafield = getattr(data, str(self.props[self.DATAFIELD].value))
        self.qwtKnob.setValue(datafield)