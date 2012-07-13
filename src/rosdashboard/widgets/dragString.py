from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport

from rosdashboard.modules.dashboardWidgets import DashboardWidget

class DragString(DashboardWidget):
    """ draggable text field """
    
    def __init__(self, parent):
        super(DragString, self).__init__(parent)
        self.setTitle('DragString')
        self.initUI()
        
    def initUI(self):
        
        self.textField = QtGui.QLineEdit(self)
        self.textField.setDisabled(True)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.textField)
        
        self.setLayout(self.layout)
        
    def initProps(self):
        pass

    def updateValue(self, value):
        self.textField.setText(str(value))
