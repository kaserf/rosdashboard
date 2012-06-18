""" module for all property classes """
from PyQt4 import QtGui

class WidgetProperty():
    
    def __init__(self, name, propertyType, value = None):
        self.name = name
        self.propertyTtype = propertyType
        self.value = value
        
class WidgetPropertiesDialog(QtGui.QDialog):
    """ allows to set the properties for a dashboard widget """
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
        
        
        for prop in self.props:
            print prop.name
            #add a label and a text field per entry in the properties
        
        self.layout = QtGui.QVBoxLayout()
        self.label = QtGui.QLabel('testlabel', self)
        self.layout.addWidget(self.label)
        self.text = QtGui.QTextEdit(self)
        self.layout.addWidget(self.text)
        
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        #TODO: check data and safe to properties
        self.accept()
        
    def dialogRejected(self):
        self.reject()
        