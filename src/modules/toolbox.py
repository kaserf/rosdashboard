from PyQt4 import QtGui, QtCore

class Toolbox(QtGui.QGroupBox):
    """ widget toolbox to choose widgets from """
    
    def __init__(self, parent):
        super(Toolbox, self).__init__(parent)

        self.initUI()
        
    def initUI(self):

        self.setTitle('Widgets')
        
        self.listWidget = QtGui.QListWidget(self)
        
        self.listWidget.setDragEnabled(True)
        self.listWidget.setDropIndicatorShown(True)
        
        self.listWidget.addItem("dragbutton")
        self.listWidget.addItem("dragdial")
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.listWidget)
        self.setLayout(self.layout)
        
    def dragEnterEvent(self, e):
        
        e.accept()