""" module for all property classes """
from PyQt4 import QtGui

class WidgetProperties():
    """ object to store the key value pairs of a widget """
    def __init__(self):        
        #key value store for the properties.
        self.props = dict()
        
    def getAllProperties(self):
        return self.props
    
    def getKeys(self):
        return self.props.keys()
    
    def getProperty(self, key):
        print 'requested value for ' + str(key)
        return self.props[key]
    
    def setProperty(self, key, value):
        print 'setting ' + str(key) + ' to ' + str(value)
        self.props[key] = value
        
class DashboardProperties():
    """ object to store the properties of all the widgets on the dashboard """
    def __init__(self):
        #props is a dictionary where the key is the widget ID
        self.props = dict()
    
    def getProperties(self, widgetID):
        print 'requested props for widget ' + str(widgetID)
        return self.props[widgetID]
    
    def setProperties(self, widgetID, props):
        print 'setting props for widget ' + str(widgetID) + ' with props ' + str(props)
        self.props[widgetID] = props
        
class WidgetPropertiesDialog(QtGui.QDialog):
    """ allows to set the properties for a dashboard widget """
    def __init__(self, parent, properties=None):
        super(WidgetPropertiesDialog, self).__init__(parent)
        
        self.props = properties
        self.initUI()
        self.exec_()
        
    def initUI(self):
        self.buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Cancel | 
                                                QtGui.QDialogButtonBox.Save, 
                                                parent=self)
        
        self.buttonBox.accepted.connect(self.dialogAccepted)
        self.buttonBox.rejected.connect(self.dialogRejected)
        
        
        for propKey in self.props.getKeys():
            print propKey
            #add a label and a text field per entry in the properties
        
        self.layout = QtGui.QVBoxLayout()
        self.label = QtGui.QLabel('testlabel', self)
        self.layout.addWidget(self.label)
        self.text = QtGui.QTextEdit(self)
        self.layout.addWidget(self.text)
        
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        #check data and safe to properties
        self.accept()
        
    def dialogRejected(self):
        self.reject()
        