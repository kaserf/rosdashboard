#!/usr/bin/env python
import roslib; roslib.load_manifest('rosdashboard')
import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose

import sys
from PyQt4 import QtGui, QtCore

from modules.props import DashboardProperties, WidgetProperties

"""
def callback(data, l):
    l.setText(data.data)
    l.resize(l.sizeHint())
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
"""

class DashboardWidget(QtGui.QGroupBox):
    """ base class for draggable widgets """
    
    """ signal to be emitted when the widget was selected """
    selected = QtCore.pyqtSignal()
    
    def __init__(self, parent):
        super(DashboardWidget, self).__init__(parent)

    def mouseMoveEvent(self, e):

        mimeData = QtCore.QMimeData()

        drag = QtGui.QDrag(self)
        drag.setMimeData(mimeData)
        drag.setHotSpot(e.pos() - self.rect().topLeft())

        dropAction = drag.start(QtCore.Qt.MoveAction)
        
    
    def mouseReleaseEvent(self, e):
        self.selected.emit()
        print('mouseReleaseEvent on DashboardWidget')
        #QtGui.QPushButton.mousePressEvent(self, e)
        #if e.button() == QtCore.Qt.LeftButton:
        #    print 'press'
       
   
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
        
    def initUI(self, title):
        
        self.button = QtGui.QPushButton(title, self)
        self.button.setDisabled(True)
        self.button.clicked.connect(self.buttonClicked)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.button)
        
        self.setLayout(self.layout)
        
    def buttonClicked(self):
        #if event.button() == QtCore.Qt.LeftButton:
        print 'button clicked'    


class Dashboard(QtGui.QGroupBox):
    """ canvas where widgets can be positioned """ 
    
    #selectionChanged = QtCore.pyqtSignal()
      
    def __init__(self, parent):
        super(Dashboard, self).__init__(parent)

        self.initUI()
        self.initProperties()
        
    def initUI(self):

        self.setTitle('Dashboard')
        self.setAcceptDrops(True)

        self.button = DragButton('Button', self)
        self.button.move(100, 65)
        
        self.button.selected.connect(self.selectionChanged)
        
        self.dial = DragDial(self)
        self.dial.move(100, 150)
        
        self.dial.selected.connect(self.selectionChanged)
        
        #self.multiple = Multiple('multi', self)
        #self.multiple.move(100, 100)
        
    def initProperties(self):
        self.props = DashboardProperties()
        widgetProps = WidgetProperties()
        widgetProps.setProperty('datasource', '/turtle1/pose')
        widgetProps.setProperty('datafield', 'linear_velocity')
        self.props.setProperties(1, widgetProps)
        
    def selectionChanged(self):
        print('selection changed from ' + str(self.sender()))
        #TODO: lookup widget (by id?) and update currentWidget (and properties?)
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        e.source().move(e.pos())

        e.setDropAction(QtCore.Qt.MoveAction)
        e.accept()
        
        
def main():
    
    rospy.init_node('python_test', anonymous=True)
    
    app = QtGui.QApplication(sys.argv)
    
    w = QtGui.QWidget()
    w.resize(800, 600)
    w.move(100, 100)
    w.setWindowTitle('Dashboard')
    
    layout = QtGui.QHBoxLayout()
    
    dashboard = Dashboard(w)
    layout.addWidget(dashboard)
    
    #dashboard.selectionChanged.connect(properties.widgetSelected)
    
    w.setLayout(layout)
    w.show()

    #rospy.Subscriber("chatter", String, callback, l)
    #rospy.spin()
    
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
