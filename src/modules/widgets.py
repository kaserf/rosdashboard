
from turtlesim.msg import Pose
from PyQt4 import QtGui, QtCore
import rospy
from modules.props import WidgetProperty, WidgetPropertiesDialog

class DashboardWidget(QtGui.QGroupBox):
    """ base class for draggable widgets """
    
    def __init__(self, parent):
        super(DashboardWidget, self).__init__(parent)
        
        self.props = list()
        self.initProps()

    def mouseMoveEvent(self, e):

        mimeData = QtCore.QMimeData()

        drag = QtGui.QDrag(self)
        drag.setMimeData(mimeData)
        drag.setHotSpot(e.pos() - self.rect().topLeft())

        dropAction = drag.start(QtCore.Qt.MoveAction)
        
    
    def mouseReleaseEvent(self, e):
        if e.button() == QtCore.Qt.RightButton:
            self.showConfigDialog()
    
    def initProps(self):
        """ this method should be overwritten in the subclass
            if properties are needed.
            Examplecode: self.props.append(name, type, value) """
        raise NotImplementedError('initProps must be implemented in a subclass!')
    
    def showConfigDialog(self):
        dialog = WidgetPropertiesDialog(self, self.props)
        dialog.exec_()
    
    def getProperties(self):
        return self.props
       
   
class DragDial(DashboardWidget):
    """ draggable dial and lcd display """
    def __init__(self, parent):
        super(DragDial, self).__init__(parent)
        self.setTitle('DragDial')
        self.initUI()
        self.initSubscriptions()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.dial = QtGui.QDial(self)
        self.dial.setMinimum(-2)
        self.dial.setMaximum(2)
        self.dial.setSliderPosition(0)
        self.dial.setDisabled(True)
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.dial.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.dial)
        self.layout.addWidget(self.lcd)
        
        self.setLayout(self.layout)
        
    def initProps(self):
        pass
        
    def initSubscriptions(self):
        rospy.Subscriber("turtle1/pose", Pose, self.subscriptionCallback)
        
    def subscriptionCallback(self, data):
        self.dial.setValue(int(data.linear_velocity))
        
class DragButton(DashboardWidget):
    """ drag button wrapper """
    def __init__(self, title, parent):
        super(DragButton, self).__init__(parent)
        self.setTitle('DragButton')
        
        self.initUI(title)
        
    def initProps(self):
        
        self.props.append(WidgetProperty('datasource', 'text', '/turtle1/pose'))
        self.props.append(WidgetProperty('datafield', 'text', 'linear_velocity'))
        
    def initUI(self, title):
        
        self.button = QtGui.QPushButton(title, self)
        self.button.setDisabled(True)
        #self.button.clicked.connect(self.buttonClicked)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.button)
        
        self.setLayout(self.layout)
        
    def buttonClicked(self):
        #if event.button() == QtCore.Qt.LeftButton:
        print 'button clicked'