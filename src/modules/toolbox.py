from PyQt4 import QtGui, QtCore
from modules.widgets import DragButton

class Toolbox(QtGui.QGroupBox):
    """ widget toolbox to choose widgets from """
    
    def __init__(self, parent):
        super(Toolbox, self).__init__(parent)

        self.initUI()
        
    def initUI(self):

        self.setTitle('Widgets')
        
        #set up scroll area
        self.scrollArea = QtGui.QScrollArea(self)
        
        self.innerLayout = QtGui.QVBoxLayout()
        self.widgetContainer = QtGui.QWidget()
        
        button1 = DragButton("test1", self)
        self.innerLayout.addWidget(button1)
        button2 = DragButton("test2", self)
        self.innerLayout.addWidget(button2)
        button3 = DragButton("test3", self)
        self.innerLayout.addWidget(button3)
        button4 = DragButton("test4", self)
        self.innerLayout.addWidget(button4)
        button5 = DragButton("test5", self)
        self.innerLayout.addWidget(button5)
        button6 = DragButton("test6", self)
        self.innerLayout.addWidget(button6)
        button7 = DragButton("test7", self)
        self.innerLayout.addWidget(button7)
        button8 = DragButton("test8", self)
        self.innerLayout.addWidget(button8)
        button9 = DragButton("test9", self)
        self.innerLayout.addWidget(button9)
        
        self.widgetContainer.setLayout(self.innerLayout)
        
        self.scrollArea.setWidget(self.widgetContainer)
        
        #place the scrollarea in the groupbox
        self.outerLayout = QtGui.QVBoxLayout()
        self.outerLayout.addWidget(self.scrollArea)
        self.setLayout(self.outerLayout)
        
        self.setAcceptDrops(True)
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        e.source().move(e.pos())

        e.setDropAction(QtCore.Qt.MoveAction)
        e.accept()
        