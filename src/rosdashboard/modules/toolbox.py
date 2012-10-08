from python_qt_binding.QtBindingHelper import QT_BINDING, QT_BINDING_VERSION #@UnresolvedImport @UnusedImport
import QtGui #@UnresolvedImport
import QtCore #@UnresolvedImport

from rosdashboard.widgets.dragDial import DragDial
from rosdashboard.widgets.dragKnob import DragKnob
from rosdashboard.widgets.dragCompass import DragCompass
from rosdashboard.widgets.dragThermo import DragThermo
from rosdashboard.widgets.dragString import DragString
from rosdashboard.widgets.dragLed import DragLed
from rosdashboard.widgets.dragPlot import DragPlot

class Toolbox(QtGui.QGroupBox):
    """ widget toolbox to choose widgets from """
    
    def __init__(self, parent):
        super(Toolbox, self).__init__(parent)

        self.initUI()
        self.populateList()
        
    def initUI(self):

        self.setTitle('Widgets')
        
        self.listWidget = QtGui.QListWidget(self)
        
        self.listWidget.setDragEnabled(True)
        self.listWidget.setDropIndicatorShown(True)
                
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.listWidget)
        self.setLayout(self.layout)
        
    def populateList(self):
        
        #TODO: iterate over a list of available widgets -> plugins
        #dragButtonItem = QtGui.QListWidgetItem("Button")
        #dragButtonItem.setIcon(style.standardIcon(QtGui.QStyle.SP_ComputerIcon))
        #dragButtonItem.setData(QtCore.Qt.UserRole, DragButton)
        #self.listWidget.addItem(dragButtonItem)
        
        dragDialItem = QtGui.QListWidgetItem("Dial")
        dragDialItem.setData(QtCore.Qt.UserRole, DragDial)
        self.listWidget.addItem(dragDialItem)
        
        dragKnobItem = QtGui.QListWidgetItem("Knob")
        dragKnobItem.setData(QtCore.Qt.UserRole, DragKnob)
        self.listWidget.addItem(dragKnobItem)
        
        dragCompassItem = QtGui.QListWidgetItem("Compass")
        dragCompassItem.setData(QtCore.Qt.UserRole, DragCompass)
        self.listWidget.addItem(dragCompassItem)
        
        dragThermoItem = QtGui.QListWidgetItem("Thermo")
        dragThermoItem.setData(QtCore.Qt.UserRole, DragThermo)
        self.listWidget.addItem(dragThermoItem)
        
        dragStringItem = QtGui.QListWidgetItem("String")
        dragStringItem.setData(QtCore.Qt.UserRole, DragString)
        self.listWidget.addItem(dragStringItem)
        
        dragLEDItem = QtGui.QListWidgetItem("LED")
        dragLEDItem.setData(QtCore.Qt.UserRole, DragLed)
        self.listWidget.addItem(dragLEDItem)
        
        dragPlotItem = QtGui.QListWidgetItem("Plot")
        dragPlotItem.setData(QtCore.Qt.UserRole, DragPlot)
        self.listWidget.addItem(dragPlotItem)
