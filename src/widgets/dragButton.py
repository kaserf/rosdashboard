from modules.props import WidgetProperty
from PyQt4 import QtGui
from modules.dashboardWidgets import DashboardWidget

class DragButton(DashboardWidget):
    """ drag button wrapper """
    def __init__(self, parent):
        super(DragButton, self).__init__(parent)
        self.setTitle('DragButton')
        
        self.initUI()
        
        #initially configure the widget from settings
        self.updateWidget()
        
    def initProps(self):
        
        self.props['datasource'] = WidgetProperty('text', '/turtle1/pose')
        self.props['datafield'] = WidgetProperty('text', 'linear_velocity')
        self.props['numeric'] = WidgetProperty('numeric', 17)
        self.props['float'] = WidgetProperty('float', 17.9)
        self.props['buttonText'] = WidgetProperty('text', "Push Me!")
        
    def initUI(self):
        
        self.button = QtGui.QPushButton(self)
        self.button.setDisabled(True)
        #self.button.clicked.connect(self.buttonClicked)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.button)
        
        self.setLayout(self.layout)
        
    def buttonClicked(self):
        #if event.button() == QtCore.Qt.LeftButton:
        print 'button clicked'
    
    def propertiesDialogAccepted(self):
        self.updateWidget()
        
    def updateWidget(self):
        self.button.setText(self.props['buttonText'].value)