from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt

class DragCompass(DashboardWidget):
    """ draggable compass"""
    
    def __init__(self, parent):
        super(DragCompass, self).__init__(parent)
        self.setTitle('DragCompass')
        self.initUI()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtCompass = Qwt.QwtCompass(self)
        self.qwtCompass.setDisabled(True)
        self.qwtCompass.setNeedle(Qwt.QwtDialSimpleNeedle(Qwt.QwtDialSimpleNeedle.Ray))
        #TODO Make widgets resizeable
        self.qwtCompass.setFixedSize(150, 150)
        
        self.layout.addWidget(self.qwtCompass)
        
        self.setLayout(self.layout)
        
    def initProps(self):
        pass
    
    def updateValue(self, value):
        self.qwtCompass.setValue(value)
