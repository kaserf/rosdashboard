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
        
        for prop in self.props:
            """ we add a property field for each property, depending on the type """
            if self.propLookup.has_key(prop.name):
                raise PropertyException('duplicate key in properties for this widget.')
            
            widget = self.defaultWidgets[prop.propertyType](self, prop.name, prop.value)
            #save a reference to the widget, to retrieve the data at dialog.save
            self.propLookup[prop.name] = widget
            self.layout.addWidget(widget)
        
        
        # The buttons belong to the bottom of the dialog
        self.layout.addWidget(self.buttonBox)
        
        self.setLayout(self.layout)
        
    def dialogAccepted(self):
        """ we iterate over the properties and fill them with the new values """
        for prop in self.props:
            prop.value = self.propLookup[prop.name].getValue()
            
        self.accept()
        
    def dialogRejected(self):
        self.reject()
        