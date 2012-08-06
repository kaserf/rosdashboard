from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt

class DragCompass(DashboardWidget):
    """ draggable compass"""
    
    def __init__(self, parent = None):
        super(DragCompass, self).__init__(parent)
        self.setTitle('DragCompass')
        self.initUI()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtCompass = Qwt.QwtCompass(self)
        self.qwtCompass.setDisabled(True)
        self.qwtCompass.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Ray))
        
        #initial size
        self.resize(200,200)
        
        self.layout.addWidget(self.qwtCompass)
        
        self.setLayout(self.layout)
        
    def initProps(self):
        pass
    
    def updateValue(self, value):
        self.qwtCompass.setValue(value)
