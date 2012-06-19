""" module for all property classes """
from PyQt4 import QtGui

class WidgetProperty():
    
    def __init__(self, propertyType, value = None):
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
        
    def getValue(self):
        return self.text.text()
        
class NumericField(QtGui.QWidget):
    def __init__(self, parent, name, default = None):
        super(NumericField, self).__init__(parent)
        
        if default == None:
            default = 0
        
        self.initUI(name, default)
        
    def initUI(self, name, default):
        """ the numeric field widget consists of a label and a spin box to enter a number """
        self.layout = QtGui.QHBoxLayout()
        
        self.label = QtGui.QLabel(name, self)
        self.layout.addWidget(self.label)
        self.spinBox = QtGui.QSpinBox(self)
        #the range is hardcoded here due to QSpinBox restrictions.
        #If you need a higher range you should subclass the dialog
        self.spinBox.setRange(-1000, 1000)
        self.spinBox.setValue(default)
        self.layout.addWidget(self.spinBox)
        
        self.setLayout(self.layout)
        
    def getValue(self):
        return self.spinBox.value()
    
class FloatField(QtGui.QWidget):
    def __init__(self, parent, name, default = None):
        super(FloatField, self).__init__(parent)
        
        if default == None:
            default = 0.0
        
        self.initUI(name, default)
        
    def initUI(self, name, default):
        """ the float field widget consists of a label and a double spin box to enter a float """
        self.layout = QtGui.QHBoxLayout()
        
        self.label = QtGui.QLabel(name, self)
        self.layout.addWidget(self.label)
        self.spinBox = QtGui.QDoubleSpinBox(self)
        #the range is hardcoded here due to QDoubleSpinBox restrictions.
        #If you need a higher range you should subclass the dialog
        self.spinBox.setRange(-1000, 1000)
        self.spinBox.setSingleStep(0.5)
        self.spinBox.setValue(default)
        self.layout.addWidget(self.spinBox)
        
        self.setLayout(self.layout)
        
    def getValue(self):
        return self.spinBox.value()
    
class PropertyException(Exception):
    def __init__(self, message):
        self.message = message
        
    def __str__(self):
        return repr(self.value)
    
class WidgetPropertiesDialog(QtGui.QDialog):
    """ allows to set the properties for a dashboard widget """
    
    defaultWidgets = {
        'text': TextField,
        'numeric': NumericField,
        'float': FloatField
    }
    
    def __init__(self, parent, properties=None):
        super(WidgetPropertiesDialog, self).__init__(parent)
        
        self.props = properties
        self.propLookup = dict()
        self.initUI()
        
    def initUI(self):
        self.buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Cancel | 
                                                QtGui.QDialogButtonBox.Save, 
                                                parent=self)
        
        self.buttonBox.accepted.connect(self.dialogAccepted)
        self.buttonBox.rejected.connect(self.dialogRejected)
        
        self.layout = QtGui.QVBoxLayout()
        
        for propKey, prop in self.props.iteritems():
            """ we add a property field for each property, depending on the type """
            widget = self.defaultWidgets[prop.propertyType](self, propKey, prop.value)
            #save a reference to the widget, to retrieve the data at dialog.save
            self.propLookup[propKey] = widget
            self.layout.addWidget(widget)
        
        
        # The buttons belong to the bottom of the dialog
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        """ we iterate over the properties and fill them with the new values """
        for propKey, prop  in self.props.iteritems():
            prop.value = self.propLookup[propKey].getValue()
            
        self.accept()
        
    def dialogRejected(self):
        self.reject()
        
class WidgetRenameDialog(QtGui.QDialog):
    """ allows to set a name for a dashboard widget """
    def __init__(self, parent, defaultName = ''):
        super(WidgetRenameDialog, self).__init__(parent)
        
        #save a reference to the parent, to set the title after accept
        self.parent = parent
        
        self.initUI(defaultName)
        
    def initUI(self, defaultName):
        self.buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Cancel | 
                                                QtGui.QDialogButtonBox.Save, 
                                                parent=self)
        
        self.buttonBox.accepted.connect(self.dialogAccepted)
        self.buttonBox.rejected.connect(self.dialogRejected)
        
        self.textField = TextField(self, 'Widget name: ', defaultName)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.textField)
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        if self.textField.getValue() != '':
            self.parent.setTitle(self.textField.getValue())
            self.accept()
        
    def dialogRejected(self):
        self.reject()