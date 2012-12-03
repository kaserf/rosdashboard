from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from rosdashboard.modules.props import WidgetProperty
from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt

class DragDial(DashboardWidget):
    """ draggable dial and lcd display """
    MIN = 'minimum'
    MAX = 'maximum'
    
    def __init__(self, parent = None):
        super(DragDial, self).__init__(parent)
        self.setTitle('DragDial')
        self.initUI()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtDial = Qwt.QwtDial(self)
        self.qwtDial.setRange(-5,5)
        self.qwtDial.setDisabled(True)
        self.qwtDial.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Ray))
        
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        self.qwtDial.valueChanged.connect(self.lcd.display)
        
        self.layout.addWidget(self.qwtDial)
        self.layout.addWidget(self.lcd)
        
        #initial size
        self.resize(200,200)
        
        #update widget according to properties
        self.updateWidget()
        
        self.setLayout(self.layout)
        
    def initProps(self):
        self.props[self.MIN] = WidgetProperty('numeric', -4)
        self.props[self.MAX] = WidgetProperty('numeric', 4)
    
    def propertiesDialogAccepted(self):
        self.updateWidget()

    def updateWidget(self):
        #update the widget properties
        self.qwtDial.setRange(self.props[self.MIN].value,
                              self.props[self.MAX].value)
        
    def updateValue(self, value):
        self.qwtDial.setValue(float(value))
