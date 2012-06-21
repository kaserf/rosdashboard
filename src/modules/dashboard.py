from PyQt4 import QtGui, QtCore
from modules.widgets import DragButton, DragDial

class Dashboard(QtGui.QGroupBox):
    """ canvas where widgets can be positioned """ 
    
    #selectionChanged = QtCore.pyqtSignal()
      
    def __init__(self, parent):
        super(Dashboard, self).__init__(parent)

        self.initUI()
        
    def initUI(self):

        self.setTitle('Dashboard')
        self.setAcceptDrops(True)

        self.button = DragButton('Button', self)
        self.button.move(100, 65)
        
        self.dial = DragDial(self)
        self.dial.move(100, 150)
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        e.source().move(e.pos())

        e.setDropAction(QtCore.Qt.MoveAction)
        e.accept()