from PyQt4 import QtGui
from rosdashboard.modules.props import WidgetProperty
import rostopic
import rospy
from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt

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
        self.qwtDial = Qwt.QwtDial(self)
        self.qwtDial.setRange(-5,5)
        self.qwtDial.setDisabled(True)
        #TODO Make widgets resizeable
        self.qwtDial.setFixedSize(150, 150)
        self.qwtDial.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Ray))
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.qwtDial.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.qwtDial)
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
        self.qwtDial.setRange(self.props[self.MIN].value,
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
        self.qwtDial.setValue(datafield)
