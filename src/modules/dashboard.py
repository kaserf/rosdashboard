from PyQt4 import QtGui, QtCore
from modules.widgets import DragButton, DragDial, DashboardWidget

class Dashboard(QtGui.QGroupBox):
    """ canvas where widgets can be positioned """ 
    
    #selectionChanged = QtCore.pyqtSignal()
      
    def __init__(self, parent):
        super(Dashboard, self).__init__(parent)

        self.initUI()
        
    def initUI(self):

        self.setTitle('Dashboard')
        self.setAcceptDrops(True)

        self.button = DragButton(self)
        self.button.move(100, 65)
        
        self.dial = DragDial(self)
        self.dial.move(100, 150)
        
    def dragEnterEvent(self, e):
        
        e.accept()

    def dropEvent(self, e):

        if isinstance(e.source(), DashboardWidget):
            #dashboard widgets are moved on the dashboard
            e.source().move(e.pos())

            e.setDropAction(QtCore.Qt.MoveAction)
            e.accept()
        else:
            #this happens if a element from the widget list gets dropped
            #TODO: typecheck for elements coming from the toolbox??
            itemDataInstance = e.source().currentItem().data(QtCore.Qt.UserRole).toPyObject()(self)
            itemDataInstance.move(e.pos())
            itemDataInstance.show()
            print "something else dropped"
        
        