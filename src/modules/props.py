""" module for all property classes """
from PyQt4 import QtGui

class WidgetProperty():
    
    def __init__(self, name, propertyType, value = None):
        self.name = name
        self.propertyType = propertyType
        self.value = value
        
class TextField(QtGui.QWidget):
    def __init__(self, parent, name, default = None):
        super(TextField, self).__init__(parent)
        
        if default == None:
            default = ''
        
        self.initUI(name, default)
        
    def initUI(self, name, default):
        """ the text field widget consists of a label and a textfield to enter text """
        self.layout = QtGui.QHBoxLayout()
        
        self.label = QtGui.QLabel(name, self)
        self.layout.addWidget(self.label)
        self.text = QtGui.QLineEdit(default, self)
        self.layout.addWidget(self.text)
        
        self.setLayout(self.layout)
    
class WidgetPropertiesDialog(QtGui.QDialog):
    """ allows to set the properties for a dashboard widget """
    
    defaultWidgets = {
        'text': TextField
    }
    
    def __init__(self, parent, properties=None):
        super(WidgetPropertiesDialog, self).__init__(parent)
        
        self.props = properties
        self.initUI()
        
    def initUI(self):
        self.buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Cancel | 
                                                QtGui.QDialogButtonBox.Save, 
                                                parent=self)
        
        self.buttonBox.accepted.connect(self.dialogAccepted)
        self.buttonBox.rejected.connect(self.dialogRejected)
        
        self.layout = QtGui.QVBoxLayout()
        
        for prop in self.props:
            """ we add a property field for each property, depending on the type """
            widget = self.defaultWidgets[prop.propertyType](self, prop.name, prop.value)
            self.layout.addWidget(widget)
        
        
        # The buttons belong to the bottom of the dialog
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        #TODO: check data and safe to properties
        self.accept()
        
    def dialogRejected(self):
        self.reject()
        