from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from rosdashboard.modules.props import WidgetProperty
from rosdashboard.modules.dashboardWidgets import DashboardWidget
from PyQt4.Qwt5 import Qwt

class DragThermo(DashboardWidget):
    """ draggable thermo """
    MIN = 'minimum'
    MAX = 'maximum'
    
    def __init__(self, parent):
        super(DragThermo, self).__init__(parent)
        self.setTitle('DragThermo')
        self.initUI()
        
    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.qwtThermo = Qwt.QwtThermo(self)
        self.qwtThermo.setRange(-5,5)
        self.qwtThermo.setDisabled(True)
        #TODO Make widgets resizeable
        self.qwtThermo.setFixedHeight(150)
        
        self.layout.addWidget(self.qwtThermo)
        
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
        self.qwtThermo.setRange(self.props[self.MIN].value,
                                self.props[self.MAX].value)

    def updateValue(self, value):
        self.qwtThermo.setValue(value)
