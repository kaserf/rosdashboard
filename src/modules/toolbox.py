from PyQt4 import QtGui, QtCore
from widgets.dragButton import DragButton
from widgets.dragDial import DragDial

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
        dragButtonItem = QtGui.QListWidgetItem("Button")
        style = QtGui.QApplication.style()
        dragButtonItem.setIcon(style.standardIcon(QtGui.QStyle.SP_ComputerIcon))
        dragButtonItem.setData(QtCore.Qt.UserRole, DragButton)
        self.listWidget.addItem(dragButtonItem)
        dragDialItem = QtGui.QListWidgetItem("Dial")
        dragDialItem.setIcon(style.standardIcon(QtGui.QStyle.SP_BrowserReload))
        dragDialItem.setData(QtCore.Qt.UserRole, DragDial)
        self.listWidget.addItem(dragDialItem)
